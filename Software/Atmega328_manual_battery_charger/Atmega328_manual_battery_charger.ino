/*  Name: MPPT Solar Charger with Bluetooth communication | Test & callibration code (with objects)
 *  Author: Karol Wojsław
 *  Date: 08/06/2021 (last release)
 *  Description: Code for Atmega328 microcontroller on MPPT charge controller board based on LM2576 switching voltage regulator aimed to test
 *  all of the basic components of the board.
 */

/* Include premade libraries and additional files */
#include <Wire.h>             // I2C bus library
#include "Solar_charger.h"    // Solar charger board library

Solar_charger testBoard;      // Create testBoard object


/* ----------------------------------------------------- Setup function ---------------------------------------------------------- */
void setup() {
  Serial.println("MPPT charge controller test & callibration mode initialised!"); // Standard Serial port test
  testBoard.ledBlink();   // Built-in LED indicator test
  testBoard.buzzer();     // Built-in buzzer test
}


/* ----------------------------------------------------- Loop function ---------------------------------------------------------- */
void loop() {
  // Variables declaration and initialisation
  unsigned long BAT_voltage = 0, PV_voltage = 0, PV_current = 0;
  testBoard.setCharging(40); // Set the feedback voltage to 50 mV
  testBoard.LM2576(ON);   // LM2576 switching regulator enable test

  BAT_voltage = testBoard.get(BAT_VOLTAGE);  // Calculate Battery voltage
  PV_voltage = testBoard.get(PV_VOLTAGE);    // Calculate PV panel voltage
  PV_current = (((testBoard.get(PV_CURRENT)-PV_cur_Vofset)*5)/4); // Calculate PV current (1 mA = 0.8 mV)

  testBoard.report(SERIAL_P, 0);
  testBoard.report(BLUETOOTH, 0);

  delay(5000); // Wait 10 seconds before the next itteration of the loop
}