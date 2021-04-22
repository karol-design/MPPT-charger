//Test of Digital I/O, Analog Input (ADC), PWM Output, Serial port

#define led_r 10
#define led_b 9
#define pot A0

void setup() {
  pinMode(led_r, OUTPUT);
  pinMode(led_b, OUTPUT);
  pinMode(pot, INPUT);

  digitalWrite(led_r, LOW);
  digitalWrite(led_b, LOW);

  Serial.begin(9600);
  //Serial.println("Analog read, PWM, Serial communication test... :)");
  delay(1000);
}

void loop() {
  int potValue = measureAverage(200);
  int potVoltage = map(potValue, 0, 1024, 0, 4960); //Map the value from potValue (0 - 1024) into voltage in mV (GND - V+)
  uint8_t PWM_val = map(potValue, 0, 1024, 255, 0); //Map the value from potValue (0 - 1024) into PWM 8-bit value (0 - 255)

  delay(1000);

  while(true){
    potValue = measureAverage(250); //Take the average of 250 measurements
    potVoltage = map(potValue, 0, 1024, 0, 4970); //Map the value from potValue (0 - 1024) into voltage in mV (GND - V+)
    PWM_val = map(potValue, 0, 1024, 255, 0); //Map the value from potValue (0 - 1024) into PWM 8-bit value (0 - 255)
    
    Serial.print("Value_10bit: ");
    Serial.print(potValue);
    Serial.print("\nValue_8bit: ");
    Serial.print(PWM_val);
    Serial.print("\nValue_mV: ");
    Serial.print(potVoltage);
    Serial.print(" mV\n----------------------\n\n");
  
    analogWrite(led_b, PWM_val);
    analogWrite(led_r, PWM_val);
    delay(2000);
  }
}

int measureAverage(int n){
  unsigned long readings_avr, readings_sum = 0UL;
  for(int i = 0; i < n; i++){ //Take n measurements
    readings_sum += analogRead(pot); //Add a 10-bit (0-1024) value read to readings_sum
    delay(1); //Atmega328 takes ~0.1 ms to read an analog val. Wait 1 ms between readings for better performance 
  }
  readings_avr = (readings_sum / n);
  return (int) readings_avr; //Return the average of n readings 
}
