/*  Name: SauleBoard | Charge controller with MPPT and Bluetooth
 *  Author: Karol Wojsław
 *  Date: 23/06/2021 (last release)
 *  Description: Source code for Atmega328 microcontroller on the SauleBoard solar charge controller
 */

/* Include premade library and additional libs/files */
#include <Wire.h>             // I2C bus library
#include "Solar_charger.h"    // Solar charger board library

Solar_charger SauleBoard;   // Create SauleBoard object


/* ----------------------------------------------------- Setup function ---------------------------------------------------------- */
void setup() {
  SauleBoard.LM2576(OFF);       // Switch off LM2576 at startup
  SauleBoard.ledBlink();        // Blink with built-in LED
  SauleBoard.buzzer();          // Board buzzer turn-on signal

  Serial.begin(9600);           // Serial port configuration (baud rate: 9600)
  delay(1000);
  Serial.println("SauleBoard | Charge controller with MPPT and Bluetooth"); // Welcome message
}


/* ----------------------------------------------------- Loop function ---------------------------------------------------------- */
void loop() {
  unsigned long tempTime = millis(), timeDifference = 0;
  uint16_t fdbk_old, fdbk_new, delta = 5;
  long P_old = 0, P_new;
  bool bulk = false;                          // Boolean value that indicates current mode of charging - bulk (true)/float (false)

  if(SauleBoard.get(PV_VOLTAGE) > 16000) {    // Check if the PV panel will be able to charge the battery (PV voltage greater than 16.0 V)
    SauleBoard.LM2576(OFF);
    delay(bulk ? 50000 : 5000);               // Wait 50/5 s (if the last mode was bulk/float) before measuring the battery voltage
    if(SauleBoard.get(BAT_VOLTAGE) < 13100) { // If Battery voltage is below the threshold (13.1 V) enter bulk charging mode with MPPT
      fdbk_old = SauleBoard.setCharging(14.65);          // Set charging voltage to 14.65 V (bulk mode)
      while(timeDifference < RUN_TIME) {      // Run the MPPT loop for RUN_TIME (15 minutes) before repeating measurements
         P_new = ((SauleBoard.get(PV_VOLTAGE) * SauleBoard.get(PV_CURRENT)) / 1000); // Calculate current power (new) in mW
        if((P_new - P_old) < 0)              // If the power value decresed change direction of delta
          delta = -delta;

        fdbk_new = fdbk_old + delta;
        Adafruit_MCP4725 MCP4725_DAC; // Create MCP4725_DAC object
        MCP4725_DAC.begin(0x60);      // MCP4725 digital DAC I2C configuration (Adress: 0x60)
        MCP4725_DAC.setVoltage(fdbk_new, false);
        fdbk_old = fdbk_new;
        P_old = P_new;
        delay(500);                           // Wait for the steady-state
        timeDifference = millis() - tempTime; 
      }
    } else {                                  // If Battery voltage is above the threshold (13.0 V) enter float charging mode
      SauleBoard.setCharging(13.70);          // Set charging voltage to 13.70 V (float mode)
    }
  } else {                                    // If the PV voltage is less than 16.0 V, then turn off the charger
    SauleBoard.LM2576(OFF);                   // Turn off the LM2576 IC
  }

  SauleBoard.report(SERIAL_P, bulk);           // Generate report and send it through standard serial port
  SauleBoard.report(BLUETOOTH, bulk);          // Generate report and send it through bluetooth

  while(timeDifference < RUN_TIME)             // Wait until RUN_TIME elapses
    timeDifference = millis() - tempTime; 
}