#define t 35000

void setup() {
  //TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz on pins D5, D6
  pinMode(3, OUTPUT); //Pin 5 as an output
}

void loop() {
  analogWrite(3, 26); //PWM Pin on Arduino UNO set to 10% duty cycle (509 mV) 526 mV measured
  delay(t);
  analogWrite(3, 20); //PWM Pin on Arduino UNO set to 8%  duty cycle (392 mV) 410 mV measured
  delay(t);
  analogWrite(3, 15); //PWM Pin on Arduino UNO set to 6%  duty cycle (294 mV) 313 mV measured
  delay(t);
  analogWrite(3, 10); //PWM Pin on Arduino UNO set to 4%  duty cycle (196 mV) 217 mV measured
  delay(t);
  analogWrite(3, 5);  //PWM Pin on Arduino UNO set to 2%  duty cycle (98 mV) 120 mV measured
  delay(t);
  analogWrite(5, 0);  //PWM Pin on Arduino UNO set to 0%  duty cycle (0 mV) 4 mV measured
  delay(t);
}
