/*  Name: MPPT Solar Charger with Bluetooth communication | Main software file
 *  Author: Karol Wojsław
 *  Date: 02/04/2021 (last release)
 */

#include <SoftwareSerial.h>

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define PWM_PIN         3
#define LM_ENABLE_PIN   4
#define BT_TX_PIN       5
#define BT_RX_PIN       6
#define BUZZER_PIN      8
#define LED_PIN         9

/*Fill in values below after callibration for better performance */
#define Vin_mV 5000 //Atmega328 supply voltage in mV
#define R3  4700 
#define R4  1000
#define R5  4700
#define R6  1000
#define R8  12000
#define R9  9000

/*Bluetooth serial port definition */
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN); // RX, TX

void setup() {
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LM_ENABLE_PIN, OUTPUT);
  pinMode(BT_TX_PIN, OUTPUT);
  pinMode(BT_RX_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  //Software serial and bluetooth serial ports configuration (baud rate: 9600)
  Serial.begin(9600);
  bluetooth.begin(9600);

  //Initialisation of the LM2576 - 13.7 V output on default
  SetCharging(1370); //Set the charging voltage to 13.70 Volts
  delay(5000); //Wait for steady-state on RC filter (connected to PWM output)
  digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576 switching buck converter
}


void loop() {
  int BAT_voltage, PV_voltage, PV_current; //Values of measured voltages and currents in mV
  
  //Measure the average of n = 20 readings from analog pins.
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 20, R5, R6);
  PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);
  PV_current = (measureAverage(PV_CURRENT_PIN, 20, 1, 0) - 2500)*10; //No voltage div, hence R1=0 & R2=1, 2.5 V nominal, then 1mV = 10 mA

  while(BAT_voltage > 1300){ //Bulk charge with MPPT algorithm implemented
    SetCharging(1470); //Set the charging voltage to 14.7 Volts
  }

  //Float-storage charge without MPPT
  SetCharging(1370); //Set the charging voltage to 13.70 Volts
  delay(5000); //Wait for steady-state on RC filter (connected to PWM output)
  digitalWrite(LM_ENABLE_PIN, LOW); //Turn on LM2576 switching buck converter

  //Generate and send a report through Bluetooth and Serial Port
  generateReport(BAT_voltage, PV_voltage, PV_current, 1); //Bluetooth (bt = 1)
  generateReport(BAT_voltage, PV_voltage, PV_current, 0); //Serial Port (bt = 0)
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that measure average of n readings from the input_pin and returns value in mV. Execution time:~5*n ms */
int measureAverage(int input_pin, int n, int V_div_R1, int V_div_R2){
  int readings_avr_mV, Vin_avr_mV, readings_sum = 0;
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(input_pin); //Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *(V_div_R1 + V_div_R2))/(V_div_R2);
  
  return Vin_avr_mV; //Return the average of n readings in mV
}

/* -------------------------------------------------------------------------------------------------------------- */
/* Function that change the charging voltage (LM2576 Vout) to a given value  */
void SetCharging(int Voltage_mV){
  //...
}


/* ---------------------------------------------------------------------------------------------- */
/* Function that send a report on the current status of the charger through bluetooth/serial port */
void generateReport(int BAT_voltage, int PV_voltage, int PV_current, bool bt){
  if(bt == 1){
    bluetooth.print("-----------------------------------------------\n");
    bluetooth.print("MPPT Solar Charger with Bt | Current status: \nBattery voltage: ");
    bluetooth.print(BAT_voltage);
    bluetooth.print(" mV \nPhotovoltaic panel voltage: ");
    bluetooth.print(PV_voltage);
    bluetooth.print(" mV \nPhotovoltaic panel current: ");
    bluetooth.print(PV_current);
    bluetooth.print(" mA \nPower delivered by PV panel: ");
    bluetooth.print(PV_current * PV_voltage);
    bluetooth.print(" mW \nBattery SOC (state of charge) ");
    bluetooth.print((BAT_voltage - 11820)/13);
    bluetooth.print(" % of full capacity \n\n");
  } else if(bt == 0){
    Serial.print("-----------------------------------------------\n");
    Serial.print("MPPT Solar Charger with Bt | Current status: \nBattery voltage: ");
    Serial.print(BAT_voltage);
    Serial.print(" mV \nPhotovoltaic panel voltage: ");
    Serial.print(PV_voltage);
    Serial.print(" mV \nPhotovoltaic panel current: ");
    Serial.print(PV_current);
    Serial.print(" mA \nPower delivered by PV panel: ");
    Serial.print(PV_current * PV_voltage);
    Serial.print(" mW \nBattery SOC (state of charge) ");
    Serial.print((BAT_voltage - 11820)/13);
    Serial.print(" % of full capacity \n\n");
  }
}
