/*  Name: MPPT Solar Charger with Bluetooth communication | Test & callibration code (with objects)
 *  Author: Karol Wojsław
 *  Date: 08/06/2021 (last release)
 *  Description: Code for Atmega328 microcontroller on MPPT charge controller board based on LM2576 switching voltage regulator aimed to test
 *  all of the basic components of the board.
 */

/* Include premade libraries and additional files */
#include <SoftwareSerial.h>   // Software serial port library
#include <Wire.h>             // I2C bus library
#include "Solar_charger.h"    // Solar charger board library

Solar_charger testBoard;                        // Create testBoard object
SoftwareSerial btModule(BT_TX_PIN, BT_RX_PIN);  // Create SoftwareSerial object and define Software serial port


/* ----------------------------------------------------- Setup function ---------------------------------------------------------- */
void setup() {
  btModule.begin(9600);       // Software Serial port configuration (baud rate: 9600)
  Serial.begin(9600);         // Serial port configuration (baud rate: 9600)
  delay(500);

  Serial.println("MPPT charge controller test & callibration mode initialised!"); // Standard Serial port test
  btModule.println("MPPT charge controller test & callibration mode initialised!"); // Bluetooth module test

  testBoard.ledBlink();   // Built-in LED indicator test
  //testBoard.buzzer();     // Built-in buzzer test
}


/* ----------------------------------------------------- Loop function ---------------------------------------------------------- */
void loop() {
  testBoard.setCharging(40); // Set the feedback voltage to 50 mV
  
  // Variables declaration and initialisation
  unsigned long BAT_voltage = 0, PV_voltage = 0, PV_current = 0, Power_in = 0, Power_out = 0, Bat_SOC = 0;

  testBoard.LM2576(ON);   // LM2576 switching regulator enable test
  //delay(4000);
  //testBoard.LM2576(OFF);  // LM2576 switching regulator disable test

  /* Note: Use this section after completing primary test and connecting power supply to both PV and Battery Port */

  /* Voltage and current measurement test */
  BAT_voltage = testBoard.get(BAT_VOLTAGE);  // Calculate Battery voltage
  PV_voltage = testBoard.get(PV_VOLTAGE);    // Calculate PV panel voltage
  PV_current = (((testBoard.get(PV_CURRENT)-PV_cur_Vofset)*5)/4); // Calculate PV current (1 mA = 0.8 mV)

  Serial.print("\n\n---------------------------------");
  Serial.print("\nBat voltage = ");
  Serial.print(BAT_voltage);
  Serial.print(" mV\nPV voltage = ");
  Serial.print(PV_voltage);
  Serial.print(" mV\nPV current = ");
  Serial.print(PV_current);
  Serial.print(" mA\n");

  Power_in = (PV_current * PV_voltage)/1000;   // Calculate Power in
  Power_out = (PV_current * BAT_voltage)/1000; // Calculate Power out
  Bat_SOC = ((BAT_voltage - 11820)/13);   // Calculate State of charge of the battery

  Serial.print("\nPower in = ");
  Serial.print(Power_in);
  Serial.print(" mW\nPower out = ");
  Serial.print(Power_out);
  Serial.print(" mW\nState of charge = ");
  Serial.println(Bat_SOC);

  /* Note: Use this section after completing primary test and deconnecting power supply */
  // testBoard.setCharging(100); // Setting voltage test

  delay(5000); // Wait 10 seconds before the next itteration of the loop
}