/*
 * Question 4 - UART Communication Protocol: ISR + Parser
 *
 * Frame format:
 *   [START_BYTE] [CMD] [LEN] [PAYLOAD...] [CHECKSUM]
 *
 *   START_BYTE : 0xAA  - fixed marker to detect frame start
 *   CMD        : 1 byte - identifies what this message means
 *   LEN        : 1 byte - number of payload bytes that follow
 *   PAYLOAD    : LEN bytes - data; may contain signed values (int16_t, int32_t)
 *   CHECKSUM   : 1 byte - XOR of CMD, LEN, and all PAYLOAD bytes
 *
 * Signed values: negative numbers are represented in two's complement
 * (standard for int8_t, int16_t, int32_t in C). No manual bit manipulation
 * needed — just cast the received bytes to the correct signed type.
 *
 * Reception strategy:
 *   - ISR (interrupt service routine): receives one byte at a time from UART
 *     hardware, stores it in a circular ring buffer. Does nothing else —
 *     ISRs must be short to avoid blocking higher-priority events.
 *   - Main loop (or a dedicated task): calls processUART() to drain the
 *     ring buffer, reassemble frames, validate checksum, and dispatch
 *     commands. All "heavy" work happens here, outside the ISR.
 *
 * This separation (ISR = collect only, main loop = process) is the standard
 * pattern for robust embedded UART handling.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------
#define START_BYTE      0xAA
#define MAX_PAYLOAD_LEN 16      // Maximum expected payload size (adjust as needed)

// Command IDs — extend as needed
#define CMD_SET_SPEED    0x01   // Payload: int16_t  (signed RPM, -32768..+32767)
#define CMD_SET_POSITION 0x02   // Payload: int32_t  (signed encoder counts)
#define CMD_SET_GAIN     0x03   // Payload: int8_t   (signed gain adjustment)

// ---------------------------------------------------------------
// Ring buffer for ISR -> main loop communication
// ---------------------------------------------------------------
#define RING_BUF_SIZE 64        // Must be a power of 2 for efficient masking

typedef struct {
    volatile uint8_t  buf[RING_BUF_SIZE];
    volatile uint8_t  head;     // Written by ISR
    volatile uint8_t  tail;     // Read by main loop
} RingBuffer_t;

static RingBuffer_t rxRing = {.head = 0, .tail = 0};

// Write one byte into the ring buffer (called from ISR — must be fast).
// If the buffer is full, the byte is silently dropped (overflow protection).
static inline void ringBuffer_push(uint8_t byte) {
    uint8_t nextHead = (rxRing.head + 1) & (RING_BUF_SIZE - 1);
    if (nextHead != rxRing.tail) {          // Buffer not full
        rxRing.buf[rxRing.head] = byte;
        rxRing.head = nextHead;
    }
    // else: buffer full — byte dropped. A production system would set an
    // overflow flag here and report it.
}

// Read one byte from the ring buffer (called from main loop).
// Returns true if a byte was available, false if buffer was empty.
static inline bool ringBuffer_pop(uint8_t *out) {
    if (rxRing.tail == rxRing.head) return false;   // Empty
    *out = rxRing.buf[rxRing.tail];
    rxRing.tail = (rxRing.tail + 1) & (RING_BUF_SIZE - 1);
    return true;
}

// ---------------------------------------------------------------
// ISR — hardware UART receive interrupt
// ---------------------------------------------------------------
// On AVR Arduino (Uno, Mega...) the UART receive ISR vector is USART_RX_vect.
// On other platforms (STM32, ESP32, etc.) the equivalent ISR handler name
// would differ — this is the only platform-specific part of the receiver.
//
// The ISR does ONE thing only: push the received byte into the ring buffer.
ISR(USART_RX_vect) {
    uint8_t receivedByte = UDR0;        // Read byte from hardware register
    ringBuffer_push(receivedByte);      // Store it — never block here
}

// ---------------------------------------------------------------
// Checksum
// ---------------------------------------------------------------
// XOR of CMD + LEN + all payload bytes.
// Simple, fast, and sufficient for detecting single-bit errors.
static uint8_t computeChecksum(uint8_t cmd, uint8_t len, const uint8_t *payload) {
    uint8_t cs = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= payload[i];
    }
    return cs;
}

// ---------------------------------------------------------------
// Command dispatch — called after a valid frame is received
// ---------------------------------------------------------------
// This is where you interpret the payload according to the command.
// Signed values are extracted via explicit casts to the correct signed type
// (the CPU handles two's complement automatically).
static void dispatchCommand(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    switch (cmd) {

        case CMD_SET_SPEED: {
            // Payload: 2 bytes, big-endian, signed (int16_t)
            // Range: -32768 (full reverse) to +32767 (full forward) RPM
            if (len < 2) break;     // Sanity check
            int16_t speed = (int16_t)((uint16_t)payload[0] << 8 | payload[1]);
            // TODO: apply speed setpoint to motor controller
            Serial.print("CMD_SET_SPEED: ");
            Serial.println(speed);  // Will print negative values correctly
            break;
        }

        case CMD_SET_POSITION: {
            // Payload: 4 bytes, big-endian, signed (int32_t)
            // Range: -(2^31) to +(2^31 - 1) encoder counts
            if (len < 4) break;
            int32_t position = (int32_t)(
                (uint32_t)payload[0] << 24 |
                (uint32_t)payload[1] << 16 |
                (uint32_t)payload[2] <<  8 |
                (uint32_t)payload[3]
            );
            // TODO: apply position setpoint
            Serial.print("CMD_SET_POSITION: ");
            Serial.println(position);
            break;
        }

        case CMD_SET_GAIN: {
            // Payload: 1 byte, signed (int8_t)
            // Range: -128 to +127 (gain trim)
            if (len < 1) break;
            int8_t gain = (int8_t)payload[0];   // Cast preserves sign bit
            // TODO: apply gain adjustment
            Serial.print("CMD_SET_GAIN: ");
            Serial.println(gain);
            break;
        }

        default:
            // Unknown command — log and ignore
            Serial.print("Unknown CMD: 0x");
            Serial.println(cmd, HEX);
            break;
    }
}

// ---------------------------------------------------------------
// Frame parser state machine
// ---------------------------------------------------------------
// Called repeatedly from loop(). Drains the ring buffer byte by byte
// and reassembles frames. Uses a state machine so it can be interrupted
// at any point (no blocking) and resume on the next call.
typedef enum {
    PARSE_WAIT_START,   // Waiting for 0xAA
    PARSE_CMD,          // Next byte is CMD
    PARSE_LEN,          // Next byte is LEN
    PARSE_PAYLOAD,      // Collecting LEN payload bytes
    PARSE_CHECKSUM      // Next byte is checksum
} ParseState_t;

static ParseState_t parseState  = PARSE_WAIT_START;
static uint8_t      frameCmd    = 0;
static uint8_t      frameLen    = 0;
static uint8_t      payloadBuf[MAX_PAYLOAD_LEN];
static uint8_t      payloadIdx  = 0;

void processUART(void) {
    uint8_t byte;

    // Drain all available bytes from the ring buffer this call.
    // Because this runs in the main loop, it never blocks.
    while (ringBuffer_pop(&byte)) {

        switch (parseState) {

            case PARSE_WAIT_START:
                // Ignore everything until we see the start marker
                if (byte == START_BYTE) {
                    parseState = PARSE_CMD;
                }
                break;

            case PARSE_CMD:
                frameCmd   = byte;
                parseState = PARSE_LEN;
                break;

            case PARSE_LEN:
                if (byte > MAX_PAYLOAD_LEN) {
                    // Length is unreasonably large — likely corrupted frame
                    // Reset and wait for the next valid start byte
                    parseState = PARSE_WAIT_START;
                    break;
                }
                frameLen   = byte;
                payloadIdx = 0;
                // If LEN == 0 there is no payload; go straight to checksum
                parseState = (frameLen > 0) ? PARSE_PAYLOAD : PARSE_CHECKSUM;
                break;

            case PARSE_PAYLOAD:
                payloadBuf[payloadIdx++] = byte;
                if (payloadIdx >= frameLen) {
                    parseState = PARSE_CHECKSUM;
                }
                break;

            case PARSE_CHECKSUM: {
                uint8_t expectedCs = computeChecksum(frameCmd, frameLen, payloadBuf);
                if (byte == expectedCs) {
                    // Frame is valid — dispatch the command
                    dispatchCommand(frameCmd, payloadBuf, frameLen);
                } else {
                    // Checksum mismatch — frame corrupted, discard silently
                    Serial.println("Checksum error — frame discarded");
                }
                // Either way, reset parser and wait for the next frame
                parseState = PARSE_WAIT_START;
                break;
            }
        }
    }
}

// ---------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------
void setup(void) {
    Serial.begin(115200);
    sei();  // Enable global interrupts (required for ISR to fire)
}

void loop(void) {
    processUART();  // Non-blocking: processes whatever bytes arrived since last call

    // All other application logic runs here freely,
    // unaffected by UART reception timing.
}
