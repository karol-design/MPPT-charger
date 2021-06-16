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
  testBoard.ledBlink();       // Built-in LED indicator test
  testBoard.buzzer();         // Built-in buzzer test
  testBoard.LM2576(LM_state); // LM2576 switching regulator enable test

  delay(1000);
  Serial.println("MPPT charge controller | Manual battery charger"); // Standard Serial port test
}


/* ----------------------------------------------------- Loop function ---------------------------------------------------------- */
void loop() {
  Serial.println("---------------------------------| Menu |---------------------------------");
  Serial.println("\"R\" (Report), \"V 00.00\" (Vout set), \"L\" (LM ON/OFF), \"B\" (Bat state)");

  while(waiting) {
    if(Serial.available() > 0) {
      char ch_read = Serial.read();
      switch (ch_read) {
        case 'R':
          Serial.println("Generating report on the current state of the charger...");
          testBoard.report(SERIAL_P, 0);
          testBoard.report(BLUETOOTH, 0);
          waiting = false;
          break;

        case 'V':
          float voltage = Serial.parseFloat();
          Serial.print("Changing voltage to ");
          Serial.print(voltage, 2);
          Serial.println(" V");
          
          testBoard.setCharging(voltage);
          waiting = false;
          break;

        case 'L':
          LM_state = !LM_state;
          testBoard.LM2576(LM_State);
          Serial.print("LM 2576 state changed to: ");
          Serial.println(LM_state);
          waiting = false;
          break;

        case 'B':
          Serial.println("Disabling LM2576... ");
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
          break;

        default:
          Serial.println("Invalid command. Try again...");
          waiting = false;
          break;
      }
    }
  }
  
  // Variables declaration and initialisation
  unsigned long BAT_voltage = 0, PV_voltage = 0, PV_current = 0;
  testBoard.setCharging(40); // Set the feedback voltage to 50 mV

  PV_voltage = testBoard.get(PV_VOLTAGE);    // Calculate PV panel voltage
  PV_current = (((testBoard.get(PV_CURRENT)-PV_cur_Vofset)*5)/4); // Calculate PV current (1 mA = 0.8 mV)

  delay(5000); // Wait 5 seconds before the next itteration of the loop
}