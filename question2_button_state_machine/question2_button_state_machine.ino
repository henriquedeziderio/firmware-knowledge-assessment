/*
 * Question 2 - Button State Machine (ON / PROTECTED / OFF)
 *
 * Logic:
 * - OFF (default): button not pressed.
 * - ON: button is currently pressed.
 * - PROTECTED: button was just released; stays here for 10 seconds
 *              (debounce / re-press protection window).
 *              - If button is pressed again during this window -> back to ON
 *              - If 10 seconds elapse without a press -> goes to OFF
 *
 * Non-blocking implementation: no delay()/while() used for timing.
 * The Arduino loop() naturally acts as the polling loop; millis()
 * is used to track elapsed time without blocking other tasks.
 *
 * Low-level button reading is abstracted in readButton(), so this
 * code can be ported to other microcontroller platforms by only
 * changing that function (and the pin setup in setup()).
 *
 * For simplicity, hardware debouncing was assumed. In production
 * firmware I would implement a software debounce filter 
 * (e.g. 20-50 ms validation window) or use a hardware RC network.
 */

#include <Arduino.h>

// ---------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------
const uint8_t BUTTON_PIN = 2;          // Digital input pin connected to the button
const unsigned long PROTECTED_TIME_MS = 10000UL; // 10 seconds protection window

// Button is wired as active-LOW with internal pull-up:
//   - Not pressed -> pin reads HIGH
//   - Pressed     -> pin reads LOW
const uint8_t BUTTON_PRESSED_LEVEL = LOW;

// Variables to count for how long the button is kept pressed:

static unsigned long pressStartTime = 0;
static unsigned long pressDuration = 0;

// ---------------------------------------------------------------
// State machine definition
// ---------------------------------------------------------------
typedef enum {
    STATE_OFF = 0,
    STATE_ON,
    STATE_PROTECTED
} ButtonState_t;

static ButtonState_t currentState = STATE_OFF;
static unsigned long protectedStartTime = 0; // millis() timestamp when PROTECTED started

// Function to print how long the button was kept pressed:
void reportPressDuration(unsigned long duration)
{
    Serial.print("Button pressed for ");
    Serial.print(duration);
    Serial.println(" ms");
}

// ---------------------------------------------------------------
// Hardware abstraction
// ---------------------------------------------------------------

// Returns true if the button is currently pressed.
// This is the only function that needs to change if porting to
// another platform/framework (ESP32, STM32 HAL, etc.).
bool readButton(void) {
    return (digitalRead(BUTTON_PIN) == BUTTON_PRESSED_LEVEL);
}

// ---------------------------------------------------------------
// State machine transition logic
// Called once per loop() iteration. Non-blocking.
// ---------------------------------------------------------------
void updateButtonStateMachine(void) {
    bool buttonPressed = readButton();

    switch (currentState) {

        case STATE_OFF:
            // Default/idle state. Move to ON as soon as button is pressed.
            if (buttonPressed) {
                currentState = STATE_ON;
                pressStartTime = millis(); // Starting to count for how long the button will be kept pressed
            }
            break;

        case STATE_ON:
            // Button is being held. Once released, enter PROTECTED
            // and start the 10-second window.
            if (!buttonPressed) {
                currentState = STATE_PROTECTED;
                
                unsigned long now = millis();

                pressDuration = now - pressStartTime; // Calculating how long the button was kept pressed

                reportPressDuration(pressDuration); // Printing how long the button was kept pressed
                
                protectedStartTime = now;
            }
            break;

        case STATE_PROTECTED:
            // Two possible exits, checked every loop() pass:
            // 1) Button pressed again -> back to ON (resets the cycle)
            // 2) 10 seconds elapsed without a press -> go to OFF
            if (buttonPressed) {
                currentState = STATE_ON;
                pressStartTime = millis();
            } else if ((millis() - protectedStartTime) >= PROTECTED_TIME_MS) {
                currentState = STATE_OFF;
            }
            // else: still waiting, do nothing this iteration.
            // loop() will run again immediately and re-evaluate.
            break;
    }
}

// ---------------------------------------------------------------
// Optional: visual feedback for debugging (LED or serial)
// ---------------------------------------------------------------
void reportState(void) {
    static ButtonState_t lastReportedState = (ButtonState_t)-1; // force first print

    if (currentState != lastReportedState) {
        switch (currentState) {
            case STATE_OFF:
                Serial.println("State: OFF");
                break;
            case STATE_ON:
                Serial.println("State: ON");
                break;
            case STATE_PROTECTED:
                Serial.println("State: PROTECTED");
                break;
        }
        lastReportedState = currentState;
    }
}

// ---------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------
void setup(void) {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.begin(115200);
}

void loop(void) {
    updateButtonStateMachine();
    reportState();

    // Rest of the application logic can run here freely,
    // since updateButtonStateMachine() never blocks.
}
