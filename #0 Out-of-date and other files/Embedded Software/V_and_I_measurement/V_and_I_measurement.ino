/*Define pin allocations on Arduino board*/
#define BAT_V_pin A0
#define BAT_V_pin A1
#define BAT_V_pin A2

void setup() {
  //Set BAT_V_pin, PV_V_pin and PV_I_pin as inputs
  pinMode(BAT_V_pin, INPUT);
  pinMode(PV_V_pin, INPUT);
  pinMode(PV_I_pin, INPUT);
}

void loop() {
  int BAT_voltage, PV_voltage, PV_current;
  
  //Measure the average of n = 20 readings from BAT_V_pin, PV_V_pin adn PV_I_pin. Total execution time: 300 ms
  BAT_voltage = measureAverage(BAT_V_pin, 20);
  PV_voltage = measureAverage(PV_V_pin, 20);
  PV_current = measureAverage(PV_I_pin, 20);
}

/* Function that measure average of n readings from the input_pin. Execution time:~5*n ms */
int measureAverage(int input_pin, int n){
  int readings_sum; //Sum of all the redings
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(input_pin) //Add a 10-bit (0-1024) value read to readings_sum
    delay(5); //Atmega328 takes ~0.1 ms to read an analog val. Wait 5 ms between readings for better performance 
  }
  return (readings_sum / n); //Return the average of n readings
}
