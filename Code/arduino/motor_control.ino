int Speed;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize serial communication with computer in order to read potentiometer values
  pinMode(9,OUTPUT); // pin 9 set as output
  analogWrite(9,26); // pwm duty cycle set initially to 10% (corresponds to 0 speed)
  delay(2000); // wait 2 seconds

}

void loop() {
  // put your main code here, to run repeatedly:
  Speed = analogRead(A0); // converts analog signal of potentiometer into value between 0 and 1023
  Speed = map(Speed, 0, 1023, 26, 230) // to communicate with ESCON cntroller, pwm duty cycle must be between 10% and 90%
  Serial.println(Speed);
  analogWrite(9,Speed); // desired speed sent as pwm to ESCON controller
  delay(100); // makes things more fluid

}
