void setup() {
  pinMode(3, OUTPUT); //Pin 3 as an output
}

void loop() {
  analogWrite(3, 5); //PWM Pin on Arduino UNO set to 5/255 (1.96%)  duty cycle (98 mV)
}
