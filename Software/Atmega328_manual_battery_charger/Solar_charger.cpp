/*  Name: Solar_charger.cpp
 *  Description: MPPT Solar Charger board with Bluetooth communication | Board library
 *  Author: Karol Wojsław
 *  Date: 16/06/2021 (last release)
 */

#include <Adafruit_MCP4725.h> // Current sensor library
#include <SoftwareSerial.h>   // Software serial port library
#include "Solar_charger.h"    // Library header file

/* ------------------------------- */
/* Solar_charger class constructor */
Solar_charger::Solar_charger() {
  // Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(LM_ENABLE_PIN, OUTPUT);
  pinMode(BT_TX_PIN, INPUT);
  pinMode(BT_RX_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Output state change to default on startup
  digitalWrite(LM_ENABLE_PIN, HIGH);  // Turn off LM2576 switching off buck converter by default
  digitalWrite(BUZZER_PIN, LOW);      // Turn off buzzer by default
  digitalWrite(LED_PIN, LOW);         // Turn off LED indicator by default
}

/* ------------------------------------------------------------------------------------- */
/* Method that measure average of n readings from the input_pin and returns value in mV. */
int Solar_charger::get(int command) {
  if(command == 1) {
    input_pin = BAT_VOLT_PIN;
    V_div_R1 = R4;
    V_div_R2 = R5;
  } else if(command == 2) {
    input_pin = PV_VOLT_PIN;
    V_div_R1 = R2;
    V_div_R2 = R3;
  } else if(command == 3) {
    input_pin = PV_CURRENT_PIN;
    V_div_R1 = 0; // No voltage divider for current sensor
    V_div_R2 = 1;
  } else {
    return -1; // Return error (-1) if command is undefined
  }
  
  int n = 100; // Number of measurements
  unsigned long readings_avr_mV, Vin_avr_mV, readings_sum = 0UL;

  for(int i = 0; i < n; i++) { // Take n measurements
    readings_sum += analogRead(input_pin);  // Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *((long)V_div_R1 + V_div_R2))/(V_div_R2);

  return Vin_avr_mV; //Return the average of n readings in mV
}


/* ----------------------------------------------------------------------- */
/* Method that change the charging voltage (LM2576 Vout) to a given value  */
void Solar_charger::setCharging(unsigned long Voltage_mV) {
  Adafruit_MCP4725 MCP4725_DAC;       // Create MCP4725_DAC object
  MCP4725_DAC.begin(0x60);            // MCP4725 digital DAC I2C configuration (Adress: 0x60)
  delay(200);

  MCP4725_DAC.setVoltage(Voltage_mV, false); //12-bit  resolution: values between 0 - 4095;  1.263mV per LSB (example for Vin 5.17 V)
}


/* ------------------------------------------------- */
/* Method that blink with LED indicator on the board */
void Solar_charger::ledBlink() {
  for(int i = 0; i < 3; i++) { // Blink 3 times with LED indicator
    digitalWrite(LED_PIN, HIGH);  
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}


/* --------------------------------------------------------------- */
/* Method that turn on and off the buzzer on the board three times */
void Solar_charger::buzzer() {
  tone(BUZZER_PIN, 1000, 250); 
  delay(100);
  tone(BUZZER_PIN, 1000, 250);
  delay(100);
  tone(BUZZER_PIN, 1500, 250);
}


/* --------------------------------------------- */
/* Method for turning on/off the LM Switching IC */
void Solar_charger::LM2576(int state){
  digitalWrite(LM_ENABLE_PIN, state); // Turn on/off LM2576
}

/* ------------------------------------------------------------ */
/* Method for debuging the board and reporting its current state*/
void Solar_charger::report(int serialPort, bool Bulk) {
  SoftwareSerial btModule(BT_TX_PIN, BT_RX_PIN);  // Create SoftwareSerial object and define Software serial port

  btModule.begin(9600);       // Software Serial port configuration (baud rate: 9600)
  Serial.begin(9600);         // Serial port configuration (baud rate: 9600)

  unsigned long _BAT_voltage = Solar_charger::get(BAT_VOLTAGE);  // Calculate Battery voltage
  unsigned long _PV_voltage = Solar_charger::get(PV_VOLTAGE);    // Calculate PV panel voltage
  unsigned long _PV_current = (((Solar_charger::get(PV_CURRENT)-PV_cur_Vofset)*5)/4); // Calculate PV current (1 mA = 0.8 mV)
  static int n_serial = 1; 
  static int n_bluetooth = 1; 
  bool _Bulk = Bulk;

  if(serialPort == 1) { // Send report over built-in standard Serial Port
    Serial.print("---------------------------------------------------------------------------------\n");
    Serial.print("MPPT Solar Charger with bluetooth communication\nReport on current status #");
    Serial.print(n_serial);
    Serial.print("\n\n# PV panel illuminated:\t\t");
    Serial.print(_PV_voltage > 10000); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 10.0V)
    Serial.print(" (Illuminated/Shaded)\n# Current charging mode:\t");
    if(_Bulk == 1){
      Serial.print("Bulk mode (MPPT algorithm on)\n# Battery voltage:\t\t");
    }else if(_Bulk == 0){
      Serial.print("Float mode (MPPT algorithm off)\n# Battery voltage:\t\t");
    }
    Serial.print(_BAT_voltage / (float) 1000.0, 3);
    Serial.print(" V \n# Photovoltaic panel voltage:\t");
    Serial.print(_PV_voltage / (float) 1000.0, 3);
    Serial.print(" V \n# Photovoltaic panel current:\t");
    Serial.print(_PV_current / (float) 1000.0, 3);
    Serial.print(" A \n# Power delivered by PV panel:\t");
    Serial.print(((_PV_current * (long) _PV_voltage) / 1000000.0), 3);
    Serial.print(" W \n# Battery SOC (state of charge)\t");
    Serial.print((_BAT_voltage - 11820)/13);
    Serial.print(" % of full capacity \n\n");
    n_serial++; //Increment current report number
  }
  
  else if(serialPort == 2) { // Send report over bluetooth
    btModule.print("---------------------------------------------------------------------------------\n");
    btModule.print("MPPT Solar Charger with bluetooth communication\nReport on current status #");
    btModule.print(n_bluetooth);
    btModule.print("\n\n# PV panel illuminated:\t\t");
    btModule.print(_PV_voltage > 10000); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 10.0V)
    btModule.print(" (Illuminated/Shaded)\n# Current charging mode:\t");
    if(_Bulk == 1){
      btModule.print("Bulk mode (MPPT algorithm on)\n# Battery voltage:\t\t");
    }else if(_Bulk == 0){
      btModule.print("Float mode (MPPT algorithm off)\n# Battery voltage:\t\t");
    }
    btModule.print(_BAT_voltage / (float) 1000.0, 3);
    btModule.print(" V \n# Photovoltaic panel voltage:\t");
    btModule.print(_PV_voltage / (float) 1000.0, 3);
    btModule.print(" V \n# Photovoltaic panel current:\t");
    btModule.print(_PV_current / (float) 1000.0, 3);
    btModule.print(" A \n# Power delivered by PV panel:\t");
    btModule.print(((_PV_current * (long) _PV_voltage) / 1000000.0), 3);
    btModule.print(" W \n# Battery SOC (state of charge)\t");
    btModule.print((_BAT_voltage - 11820)/13);
    btModule.print(" % of full capacity \n\n");
    n_bluetooth++; //Increment current report number
  }
}