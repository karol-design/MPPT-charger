/*  Name: MPPT Solar Charger with Bluetooth communication | Test & callibration code
 *  Author: Karol Wojsław
 *  Date: 03/06/2021 (last release)
 *  Description: Code for Atmega328 microcontroller on MPPT charge controller board...
 *  ...based on LM2576 switching voltage regulator aimed to test all basic components of the board
 */

/* Include premade libraries and additional files */
#include <Adafruit_MCP4725.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/* Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0  // Bat voltage on PC0
#define PV_VOLT_PIN     A1  // PV voltage on PC1
#define PV_CURRENT_PIN  A2  // PV current on PC2
#define LM_ENABLE_PIN   4   // LM enable on PD4
#define BT_TX_PIN       5   // Bluetooth TX (Transmit) on PD5
#define BT_RX_PIN       6   // Bluetooth RX (Receive) on PD6
#define BUZZER_PIN      8   // Buzzer on PB0
#define LED_PIN         9   // LED indicator on PB1

/* Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5020       // Atmega328 supply voltage in mV - increased by 25 mV for more precise calculations (systematic error)
#define PV_cur_Vofset 320 // Voltage in mV to be subtracted from PV_current sensor readings (systematic error)
#define R3  5020          // R3 resistance in Ohms
#define R4  930           // R4 resistance in Ohms
#define R5  5070          // R5 resistance in Ohms
#define R6  980           // R6 resistance in Ohms

Adafruit_MCP4725 MCP4725_DAC; // Create MCP4725_DAC object
SoftwareSerial btModule(BT_RX_PIN, BT_TX_PIN); // Define Software serial port

/* Function declaration */
int measureAverage(int input_pin, int n, int V_div_R1, int V_div_R2);
void SetCharging(unsigned long Voltage_mV);
void generateReport(int BAT_voltage, int PV_voltage, int PV_current, bool Bulk, bool bt);


void setup() {
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

  MCP4725_DAC.begin(0x60); // MCP4725 digital DAC I2C configuration (Adress: 0x60)
  SetCharging(100); // Set the charging voltage to 13.70 V by default

  btModule.begin(9600); // Software Serial port configuration (baud rate: 9600)
  Serial.begin(9600);   // Serial port configuration (baud rate: 9600)
  delay(1000);          // Wait for 1 second before executing loop
}


void loop() {
  // Variables declaration and initialisation
  unsigned long BAT_voltage = 0, PV_voltage = 0, PV_current = 0, Power_in = 0, Power_out = 0, Bat_SOC = 0;

  /* Bluetooth module test */
  Serial.println("MPPT charge controller test & callibration mode initialised!"); // Send test message through standard Serial port

  /* Standard Serial port test */
  btModule.println("MPPT charge controller test & callibration mode initialised!"); // Send test message through bluetooth module

  /* LED indicator test */
  digitalWrite(LED_PIN, HIGH);  // Turn on LED indicator
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);

  /* Buzzer test */
  digitalWrite(BUZZER_PIN, HIGH);  // Turn on Buzzer
  delay(2000);
  digitalWrite(BUZZER_PIN, LOW);

  /* LM enable test */
  digitalWrite(LM_ENABLE_PIN, LOW); // Turn on LM2576
  delay(4000);
  digitalWrite(LM_ENABLE_PIN, HIGH);

  /* ------------------------------------------------------------------------------------------------------------ */
  /* Uncomment this section after completing primary test and connecting power supply to both PV and Battery Port */

  /* BAT voltage measurement test 
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 100, R5, R6);  // Calculate Battery voltage based on 100 measurements
  Serial.print("Bat voltage = ");
  Serial.println(BAT_voltage);

  /* PV voltage measurement test 
  PV_voltage = measureAverage(PV_VOLT_PIN, 100, R3, R4);    // Calculate PV panel voltage based on 100 measurements
  Serial.print("PV voltage = ");
  Serial.println(PV_voltage);

  /* PV current measurement test 
  PV_current = (((measureAverage(PV_CURRENT_PIN, 100, 0, 1)-PV_cur_Vofset)*4)/6); // Calculate PV current (no voltage div, 1 mA = 0.8 mV)
  Serial.print("PV current = ");
  Serial.println(PV_current);

  /* Calculations test 
  Power_in = PV_current * PV_voltage;   // Calculate Power in
  Power_out = PV_current * BAT_voltage; // Calculate Power out
  Bat_SOC = (BAT_voltage - 11820)/13;   // Calculate State of charge of the battery
  Serial.print("Power in = ");
  Serial.println(Power_in);
  Serial.print("Power out = ");
  Serial.println(Power_out);
  Serial.print("State of charge = ");
  Serial.println(Bat_SOC); */

  /* ---------------------------------------------------------------------------------- */
  /* Uncomment this section after completing primary test and deconnecting power supply */
  
  /* Setting voltage test 
  SetCharging(100); // Set the charging voltage to 100 */

  delay(5000); // Wait 5 seconds before the next itteration of the loop
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that measure average of n readings from the input_pin and returns value in mV. Execution time:~5*n ms */
int measureAverage(int input_pin, int n, int V_div_R1, int V_div_R2){
  unsigned long readings_avr_mV, Vin_avr_mV, readings_sum = 0UL;
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(input_pin); //Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *((long)V_div_R1 + V_div_R2))/(V_div_R2);
  return Vin_avr_mV; //Return the average of n readings in mV
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that change the charging voltage (LM2576 Vout) to a given value  */
void SetCharging(unsigned long Voltage_mV){
  MCP4725_DAC.setVoltage(Voltage_mV, false); //12-bit  resolution: values between 0 - 4095;  1.263mV per LSB (example for Vin 5.17 V)
  return;
}