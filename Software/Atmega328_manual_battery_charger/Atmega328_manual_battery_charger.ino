/*  Name: MPPT Solar Charger with Bluetooth communication | Test & callibration code (with objects)
 *  Author: Karol Wojsław
 *  Date: 16/06/2021 (last release)
 *  Description: Code for Atmega328 microcontroller on MPPT charge controller board based on LM2576 switching voltage regulator aimed to test
 *  all of the basic components of the board.
 */

/* Include premade libraries and additional files */
#include <Wire.h>             // I2C bus library
#include "Solar_charger.h"    // Solar charger board library

Solar_charger testBoard;      // Create testBoard object
bool LM_state = ON;


/* ----------------------------------------------------- Setup function ---------------------------------------------------------- */
void setup() {
  Serial.begin(9600);         // Serial port configuration (baud rate: 9600)
  testBoard.ledBlink();       // Built-in LED indicator test
  testBoard.buzzer();         // Built-in buzzer test
  testBoard.LM2576(LM_state); // LM2576 switching regulator enable test

  delay(1000);
  Serial.println("MPPT charge controller | Manual battery charger"); // Standard Serial port test
}


/* ----------------------------------------------------- Loop function ---------------------------------------------------------- */
void loop() {
  Serial.println("\n-------------------------------| Menu |-------------------------------");
  Serial.println("\"R\" (Report), \"V 00.00\" (Vout set), \"L\" (LM ON/OFF), \"B\" (Bat state)");
  bool waiting = true;

  while(waiting) {
    if(Serial.available() > 0) {
      char ch_read = Serial.read();

      if(ch_read == 'R') {
        Serial.println("\nGenerating report on the current state of the charger...");
        testBoard.report(SERIAL_P, 0);
        testBoard.report(BLUETOOTH, 0);
        waiting = false;
      } else if(ch_read == 'V') {
        float voltage = Serial.parseFloat();
        Serial.print("\nChanging voltage to ");
        Serial.print(voltage, 2);
        Serial.println(" V");

        testBoard.setCharging(voltage);
        waiting = false;
      } else if(ch_read == 'L') {
        LM_state = !LM_state;
        testBoard.LM2576(LM_state);
        Serial.print("\nLM2576 state changed to: ");
        Serial.println(LM_state);
        waiting = false;
      } else if(ch_read == 'B') {
        Serial.println("\nDisabling LM2576... ");
        if(LM_state == ON) {
          LM_state == OFF;
        }
        testBoard.LM2576(OFF);
        delay(2000);
        Serial.println("Measuring battery voltage... ");
        float BAT_voltage = (testBoard.get(BAT_VOLTAGE) / (float) 1000.0);  // Calculate Battery voltage
        Serial.print("Current battery voltage = ");
        Serial.print(BAT_voltage, 3);
        Serial.println(" V");
        waiting = false;
      } else {
        Serial.println("\nInvalid command. Try again...");
        waiting = false;
      }

      while(Serial.read() > 0) {} // Clear Serial port buffer
    }
  }
  delay(200); // Wait 0.2 second before the next itteration of the loop
}