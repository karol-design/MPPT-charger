/*Define pin allocations on Arduino board*/
#define BAT_VOLT_PIN    A0
#define PV_VOLT_PIN     A1
#define PV_CURRENT_PIN  A2

/*Fill in values below after the callibration for better performance*/
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

  Serial.begin(9600);
}


void loop() {
  int BAT_voltage, PV_voltage, PV_current;
  
  //Measure the average of n = 20 readings from BAT_VOLT_PIN, PV_VOLT_PIN adn PV_CURRENT_PIN. Total execution time: 300 ms
  BAT_voltage = measureAverage(BAT_VOLT_PIN, 20);
  PV_voltage = measureAverage(PV_VOLT_PIN, 20);
  PV_current = measureAverage(PV_CURRENT_PIN, 20);

  Serial.println(map(BAT_voltage, 0, 1024, 0, Vin_mV)) //Print the measured voltage in mV
  Serial.println(map(PV_voltage, 0, 1024, 0, Vin_mV))
  Serial.println(map(PV_vurrent, 0, 1024, 0, Vin_mV))
}


/* -------------------------------------------------------------------------------------------------------------- */
/* Function that measure average of n readings from the input_pin and returns value in mV. Execution time:~5*n ms */
int measureAverage(int input_pin, int n, int V_div_R1, int V_div_R2){
  int readings_avr_mV, Vin_avr_mV, readings_sum = 0;
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(input_pin) //Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  readings_avr_mV = map((readings_sum / n), 0, 1024, 0, Vin_mV);
  Vin_avr_mV = (readings_avr_mV *(V_div_R1 + V_div_R2))/(V_div_R2);
  
  return Vin_avr_mV; //Return the average of n readings in mV
}
