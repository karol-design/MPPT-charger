/*  Name: MPPT Solar Charger with Bluetooth communication | Measurements taking & Report generation test code
 *  Author: Karol Wojsław
 *  Date: 07/04/2021 (last release)
 *  Description: For test purposes only
 *  
 *  Tests outcomes: Program working properly for both serial ports. Huge impact of calibration on voltage measurements
 *  ...small impact of averaging and moderate accuracy (+/- 100 - 500mV)
 */ 

#include <SoftwareSerial.h>

/*Define pin allocations on Arduino board */
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2
#define BT_TX_PIN       5
#define BT_RX_PIN       6

/*Fill in values below after callibration for improved precision of measurements and charger performance */
#define Vin_mV 5020 //Atmega328 supply voltage in mV - increased by 25 mV for more precise calculations (systematic error)
#define R3  5020 
#define R4  930
#define R5  5070
#define R6  980

unsigned int n = 1; //Variable with current report issue


void setup() {
  //Input-output pin mode definition
  pinMode(BAT_VOLT_PIN, INPUT);
  pinMode(PV_VOLT_PIN, INPUT);
  pinMode(PV_CURRENT_PIN, INPUT);
  pinMode(BT_TX_PIN, INPUT);
  pinMode(BT_RX_PIN, OUTPUT);
}


void loop() {
  //Variables declaration
  unsigned long BAT_voltage, PV_voltage, PV_current;
  bool Bulk = false;

  /* --------------------------------------------------------------------------- */
  /*Measure the average of n = 10 reading from analog pins and generate reports   */
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 10, R5, R6);
  PV_voltage = measureAverage(PV_VOLT_PIN, 10, R3, R4);
  PV_current = (((measureAverage(PV_CURRENT_PIN, 10, 0, 1)-215)*4)/5); //No voltage div, hence R1=0 & R2=1, 1 mA = 0.8 mV
  
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 1); //Send report over bluetooth (bt = 1)
  generateReport(BAT_voltage, PV_voltage, PV_current, Bulk, 0); //Send report over USB serial port
  
  delay(3000);
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
/* Function that send a report on the current status of the charger through bluetooth/USB serial port */
void generateReport(int BAT_voltage, int PV_voltage, int PV_current, bool Bulk, bool bt){
  int RX_PIN = 0, TX_PIN = 1; //Define serial port pins for standard Arduino USB Serial port (by default)
  if(bt == true){ //Define serial port pins for bluetooth module if bt == true
    RX_PIN = BT_RX_PIN;
    TX_PIN = BT_TX_PIN;
  }

  SoftwareSerial serialPort(RX_PIN, TX_PIN); //Define Software serial port
  serialPort.begin(9600); //Serial port configuration (baud rate: 9600)

  serialPort.print("---------------------------------------------------------------------------------\n");
  serialPort.print("MPPT Solar Charger with bluetooth communication\nReport on current status #");
  serialPort.print(n/2);
  serialPort.print("\n\n# PV panel illuminated:\t\t");
  serialPort.print(PV_voltage > 10000); //Print 1 if PV panel is illuminated by the sunlight (PV_V > 10.0V)
  serialPort.print(" (Illuminated/Shaded)\n# Current charging mode:\t");
  if(Bulk == 1){
    serialPort.print("Bulk mode (MPPT algorithm on)\n# Battery voltage:\t\t");
  }else if(Bulk == 0){
    serialPort.print("Float mode (MPPT algorithm off)\n# Battery voltage:\t\t");
  }
  serialPort.print(BAT_voltage / (float) 1000.0, 3);
  serialPort.print(" V \n# Photovoltaic panel voltage:\t");
  serialPort.print(PV_voltage / (float) 1000.0, 3);
  serialPort.print(" V \n# Photovoltaic panel current:\t");
  serialPort.print(PV_current / (float) 1000.0, 3);
  serialPort.print(" A \n# Power delivered by PV panel:\t");
  serialPort.print(((PV_current * (long) PV_voltage) / 1000000.0), 3);
  serialPort.print(" W \n# Battery SOC (state of charge)\t");
  serialPort.print((BAT_voltage - 11820)/13);
  serialPort.print(" % of full capacity \n\n");
  if(bt == false){ //Additional new lines for bluetooth module if bt == true
    serialPort.print("\n\n\n\n\n");
  }
  n++; //Increment current report number
  return;
}
