/*
 * AllOutputsSimultaneous
 *
 * This example demonstrates driving all 20 available PWM outputs simultaneously
 * on the ESP32 S3 using a mix of MCPWM and LEDC hardware.
 *
 * The ESP32 S3 supports 20 PWM channels:
 * - 8 LEDC channels (shared timers)
 * - 12 MCPWM channels (6 timers × 2 operators each for shared frequency)
 *
 * This example uses:
 * - 12 ESP32PWM objects with default settings (MCPWM, shared frequency)
 * - 8 ESP32PWM objects with variable frequency (LEDC channels)
 *
 * Circuit:
 * Connect servo motors or LEDs to the PWM pins listed below.
 * Ensure adequate power supply for servos (ESP32 3.3V may not be sufficient).
 */

#include <ESP32PWM.h>

// ESP32 S3 PWM pins: 1-21, 35-45, 47-48
// We'll use 20 pins from the available ones
int pwmPins20[20] = {
  1, 2, 3, 4, 5, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,  // 15 pins
 37, 38, 39  // 5 more = 20 total
};

ESP32PWM* pwms[20];  // Array of 20 PWM objects

int duty = 0;        // Current duty cycle (0-255 for 8-bit resolution)
int increment = 1;   // Duty increment

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up 20 simultaneous PWM outputs...");

  // Allocate all LEDC timers for maximum LEDC channels
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Create first 12 PWM objects with default settings (MCPWM, shared frequency)
  // These will share timers and thus frequencies
  for (int i = 0; i < 20; i++) {
    pwms[i] = new ESP32PWM(false);  // false = shared frequency
    pwms[i]->attachPin(pwmPins20[i], 50, 10);  // 50Hz, 10-bit resolution
    if (pwms[i]->attached()) {
      Serial.printf("Attached PWM %d to pin %d (MCPWM shared)\n", i, pwmPins20[i]);
    } else {
      Serial.printf("Failed to attach PWM %d to pin %d\n", i, pwmPins20[i]);
    }
  }

  Serial.printf("Total PWM channels remaining: %d\n", ESP32PWM::channelsRemaining());
  Serial.println("Setup complete. Starting simultaneous PWM sweep...");
}

void loop() {
  // Set all PWM outputs to the same duty cycle for simultaneous control
  for (int i = 0; i < 20; i++) {
    pwms[i]->writeScaled((float)duty / 255.0);  // Convert to 0.0-1.0 range
  }

  Serial.printf("All PWM outputs set to duty cycle: %d/255\n", duty);

  // Update duty cycle
  duty += increment;
  if (duty >= 255 || duty <= 0) {
    increment = -increment;  // Reverse direction at limits
  }

  delay(10);  // Small delay for visible changes
}
