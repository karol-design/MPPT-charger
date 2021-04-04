/*  Name: MPPT Solar Charger with Bluetooth communication | Measurements taking & Report generation test code
 *  Author: Karol Wojsław
 *  Date: 04/04/2021 (last release)
 *  Description: For test purposes only
 */


#include <SoftwareSerial.h>

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define BT_TX_PIN       5
#define BT_RX_PIN       6

/*Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5000 //Atmega328 supply voltage in mV
#define R3  4700 
#define R4  1000
#define R5  4700
#define R6  1000


void setup() {
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(BT_TX_PIN, OUTPUT);
  pinMode(BT_RX_PIN, OUTPUT);
}


void loop() {
  //Variables declaration
  unsigned long BAT_voltage, PV_voltage, PV_current;
  bool Bulk = false;

  /* --------------------------------------------------------------------------- */
  /*Measure the average of n = 1 readings from analog pins and generate reports */
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 1, R5, R6);
  PV_voltage = measureAverage(PV_VOLT_PIN, 1, R3, R4);
  PV_current = ((measureAverage(PV_CURRENT_PIN, 1, 1, 0) - 2500)*10); //No voltage div, hence R1=0 & R2=1, 2.5 V nominal, then 1mV = 10 mA
  
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 1); //Send report over bluetooth (bt = 1)
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port

  delay(1000);

  /* --------------------------------------------------------------------------- */
  /*Measure the average of n = 20 readings from analog pins and generate reports */
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 20, R5, R6);
  PV_voltage = measureAverage(PV_VOLT_PIN, 20, R3, R4);
  PV_current = ((measureAverage(PV_CURRENT_PIN, 20, 1, 0) - 2500)*10); //No voltage div, hence R1=0 & R2=1, 2.5 V nominal, then 1mV = 10 mA
  
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 1); //Send report over bluetooth (bt = 1)
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port

  delay(1000);

  /* --------------------------------------------------------------------------- */
  /*Measure the average of n = 50 readings from analog pins and generate reports */
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 50, R5, R6);
  PV_voltage = measureAverage(PV_VOLT_PIN, 50, R3, R4);
  PV_current = ((measureAverage(PV_CURRENT_PIN, 50, 1, 0) - 2500)*10); //No voltage div, hence R1=0 & R2=1, 2.5 V nominal, then 1mV = 10 mA
  
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 1); //Send report over bluetooth (bt = 1)
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port
  
  delay(5000);
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
/* Function that send a report on the current status of the charger through bluetooth/USB serial port */
void generateReport(int BAT_voltage, int PV_voltage, int PV_current, bool Bulk, bool bt){
  int RX_PIN = 0, TX_PIN = 1; //Define serial port pins for standard Arduino USB Serial port (by default)
  if(bt == true){ //Define serial port pins for bluetooth module if bt == true
    RX_PIN = BT_RX_PIN;
    TX_PIN = BT_TX_PIN;
  }

  SoftwareSerial serialPort(RX_PIN, TX_PIN); //Define Software serial port
  serialPort.begin(9600); //Serial port configuration (baud rate: 9600)

  serialPort.print("---------------------------------------------\n");
  serialPort.print("MPPT Solar Charger with Bt | Current status: \nPV panel status:: ");
  serialPort.print(PV_voltage > 1300); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 13.0V)
  serialPort.print(",Illuminated (1) / Shaded/Night (0)\nCurrent charging mode:");
  if(Bulk == 1){
    serialPort.print("Bulk mode (MPPT algorithm on)\nBattery voltage: ");
  }else if(Bulk == 0){
    serialPort.print("Float mode (MPPT algorithm off)\nBattery voltage: ");
  }
  serialPort.print(BAT_voltage);
  serialPort.print(" mV \nPhotovoltaic panel voltage: ");
  serialPort.print(PV_voltage);
  serialPort.print(" mV \nPhotovoltaic panel current: ");
  serialPort.print(PV_current);
  serialPort.print(" mA \nPower delivered by PV panel: ");
  serialPort.print(PV_current * PV_voltage);
  serialPort.print(" mW \nBattery SOC (state of charge) ");
  serialPort.print((BAT_voltage - 11820)/13);
  serialPort.print(" % of full capacity \n\n\n");
  return;
}
