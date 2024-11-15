/* ARDUINO CODE 
  by Noah Lugon, last updated April 14 2023


  Main Components :
    - Arduino -> low level processing unit
    - Raspberry Pi -> high level processing unit
    - Maxon motors -> high precision motors for differential drive locomotion 
    - Polulu motors -> motors for turning front brushes
    - Servo motor -> for duplo brick unloading
    - Servo driver -> Adafruit 16-Channel 12-bit PWM/Servo Driver (controlled by Arduino via I2C)
    - Ultrasonic distance senors -> for distance measuring
    - LEDs -> WS2812B Addressable RGB LED Strip with 32 LEDs used for esthetic as well as debugging purposes
        Center LEDs (14 to 17) used to indicate battery level
        Left and right LEDs (0-9 and 22-31) used to indicate if obstacle detected on left or right side (display in pink)
            Left and right LEDs also used to indicate other modes (turquoise for unloading, yellow for pressing button, 
                red for maneuver due to too high Maxon motor current, blue for maneuver if get stuck too long in a corner)

  Features :
  - Maxon motor speed control
  - Maxon motor sensor data reading 
        Speed data for odometry -> if robot keeps detecting an obstacle in the same position, will make maneuver (turn randomly left or right by approx 180 deg)
        Current sensing -> if duplo ramp gets stuck under carpet, robot senses unusually high torque and makes backwards maneuver, followed by random left or right maneuver
  - Polulu motor speed control (tuned constant speed here)
  - Polulu motor encoder reading (if either of the two motors has low speed or gets stuck -> direction reversed for both motors to push duplo brick out)
  - Servo control for unloading
  - Simple and very robust obstacle avoidance by reversing direction of motor when an obstacle on the opposite side of the robot is detected
  - Obstacle avoidance disabled during maneuvers, as well as when Arduino is commanded to run straight to the button (current sensing then used to determine when button is pressed)
      Extra feature of button pressing : when button pressed, robot always turns left (use_random_direction boolean set to false, set back to true once Raspberry Pi sends new motor command "m...") 


  - Serial communication with Raspberry pi
    Motor commands : "m+99-50" (first three characters after the 'm' is for the left motor, then the next three for the right)
        This custom protocal using a '+' or '-' character enables to choose between positive or negative motor speeds
    Unloading command : "u"
    Full arduino autonomous mode : "a" (basically resets default speed to +99 on left and right motors)
    Move forward to push button : "b"
    Emergency STOP command : "s"
    Raspberry pi requests data : "d"
        Upon request, data formats received from arduino to raspberry pi :
            "dxxx xxx xxx xxx xxx xxx " with "xxx" representing distance measured by ultrasonic sensor in cm 
        As soon as low battery detected : "b" sent continuously, indicating low battery voltage -> robot should return to collecting point and stop brushes from turning to save battery      
    Polulu motor command : "D1" to turn off, "D0" to turn on (default) (D for Debug)
        Note : if D0 sent but the polulus are not connected to the power source, there will be very high delayes due to the pulseIn() function awaiting motor encoder data (timeout set to 250ms)


  WARNING ultrasonic sensor sometimes ouptut a value of 0 even when no obstacle is detected
    (solution : ignore 0 values, works very well in practice)

  WARNING When running the full code, the data sent from the Arduino to the Rasbperry Pi caused unexpected crashes of the main program on the Raspbery Pi.
      As we were unable to resolve this issue, no Serial data is sent to the Rasbperry Pi from the Arduino (no Serial.print())




  MAXON LEFT MOTOR CONNECTED TO MAXON CONTROLLER # 4
  Polulu left motor connected to M1 port of the motor driver

*/ 



#include <Servo.h>
#include <FastLED.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

#define LED_PIN     7
#define NUM_LEDS    32

#define ULTRASONIC_INTERVAL 40 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define ULTRASONIC_NUM 5 // Number of sensors




bool DEBUG = 0;
bool use_random_direction = 1;
int randomNumber;
bool use_adafruit_driver = 1;


// FSM state
int fsmState = 0; // when state = 0, simple forward movement and obstacle avoidance
int turnTime = 1500;

// Polulu motor state and default speed
bool polulusOn = 1;
int poluluSpeed = 70; 

// variables used to detect changes of state between obstacle being detected / no longer being detected
bool obstacle = 0;
bool obstacleLeft = 0;
bool obstacleRight = 0;
bool prevObstacle = 0;
bool prevObstacleLeft = 0;
bool prevObstacleRight = 0;
bool NewObstacleState = 0; // if equals 1, indicates a change of state from (no obstacles detected) to (obstacle detected).




// LEDs
CRGB leds[NUM_LEDS];
// table of LED values, we found this is faster than using 
int ledStates[NUM_LEDS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int prevLedStates[NUM_LEDS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


unsigned long currentTime = 0;

unsigned long ultrasonicTimer[ULTRASONIC_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[ULTRASONIC_NUM] = {100,100,100,100,100};         // Where the measured ultrasonic distances are stored.
bool obstacleState[ULTRASONIC_NUM] = {0,0,0,0,0};
//uint8_t currentUltrasonicSensor = 0;          // Keeps track of which sensor is active.
//unsigned int  ultrasonicSensors[ULTRASONIC_NUM];
// ULTRASONIC PINS
// const int ultrasonicTrigPins[ULTRASONIC_NUM] = {30,32,34,36,38,40};
// const int ultrasonicEchoPins[ULTRASONIC_NUM] = {31,33,35,37,39,41};
const int ultrasonicTrigPins[ULTRASONIC_NUM] = {42,32,34,36,38};
const int ultrasonicEchoPins[ULTRASONIC_NUM] = {43,33,35,37,39};



// SERVOMOTOR

// using adafruit driver
uint8_t servonum = 0;

// using arduino analogWrite pin
const int bigAssServoPin = 3;
int pos = 30;
Servo bigAssServo;





// MAXON CONTROLLER PINS
const int leftMotorEnablePin = 22;
const int leftMotorDirectionPin = 23;
const int leftMotorPwmPin = 10;

const int rightMotorEnablePin = 24;
const int rightMotorDirectionPin = 25;
const int rightMotorPwmPin = 11;

// DC MOTOR DRIVER PINS
const int MotorADirectionPin = 52;
const int MotorAPwmPin = 8;
const int MotorAEncoderPin = 50;

const int MotorBDirectionPin = 53;
const int MotorBPwmPin = 9;
const int MotorBEncoderPin = 51;


unsigned long startMillis;  // some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long poluluPeriod = 1000;  // after polulu motors detected to be stuck, turn brushes for 1s in outwards direction to spit out duplos 
const unsigned long poluluPeriodUnstuck = 200; // after polulu motos unstuck, give 200ms time for brushes to turn back inwards


unsigned long fsmStartMillis;
unsigned long fsmCurrentMillis;
const unsigned long fsmPeriod = 1000;

unsigned long obstacleCounter = 0;

unsigned long serialStartMillis;
unsigned long serialCurrentMillis;

unsigned long odometryStartMillis;
unsigned long odometryCurrentMillis;

// Maxon motor speed and drawn current data
int speedLeft = 0;
int currentLeft = 0;
int speedRight = 0;
int currentRight = 0;

// Odometry
long positionLeft;
long positionRight;
long positionForward;
long prevPositionForward;
long positionAngular;
bool not_moving_forward;

// Battery Monitoring
int batteryLevel = 0;

// Polulu motor encoder info
int ontimeA,offtimeA,dutyA;
float freqA, periodA;
int ontimeB,offtimeB,dutyB;
float freqB, periodB;
bool motorAstuck = 0;
bool motorBstuck = 0;

// Variables for ultrasonic sensors
long duration, distance, sonicSensor1;

// Communication with Raspberry Pi : 
String msg = "";

int left_speed = 99; // WARNING value between -99 and 99
int right_speed = 99; // WARNING value between -99 and 99



// FUNCTIONS
void SonarSensor();
void updateMotorCommands();
void getData();
void sendData();
void readSerialPort();
void sensorAvoidance();
void updatePoluluCommands();
void unloading();
void moveServos();
void adafruitCommand(int startPulse, int endPulse);
void analogWriteCommand(int startPos, int endPos);
void checkBatteryLevel();

void checkMotors();
void fsm();
void newFSMstate(int stateNumber = 0);

void updateLEDs();
void updateSideLEDs(int color);









/********************* SETUP *********************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial communication with computer

  // while (!Serial) {
  //   delay(10); // wait for serial port to connect. Needed for native USB port only
  // }


  // LEDs Initialization
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  // FastLED.setBrightness(10);
  for (uint8_t i = 0; i <= 31; i++) {
    leds[i] = CRGB (0, 0, 10); // blue
    FastLED.show();
    delay(40);
  }
  for (uint8_t i = 0; i <= 31; i++) {
    leds[i] = CRGB (0, 0, 0); // blue
    FastLED.show();
    delay(40);
  }

  checkBatteryLevel();
  updateLEDs();


  // SERVO CHECK : tilt duplo container up once upon startup
  if (use_adafruit_driver) {
      pwm.begin();
      pwm.setOscillatorFrequency(27000000);
      pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
      pwm.setPWM(servonum, 0, SERVOMIN);
      delay(1000);
      pwm.setPWM(servonum, 0, SERVOMAX);
      delay(1000);
      pwm.setPWM(servonum, 0, SERVOMIN);
      delay(1000);
      pwm.setPWM(0, 0, 0);  // turn off PWM signal to avoid servo humming
  }
  else {
    // SET PINS FOR SERVOMOTOR
    bigAssServo.attach(bigAssServoPin);
    bigAssServo.write(30);
    delay(1000);
    bigAssServo.write(130);
    delay(1000);
    bigAssServo.write(30);
    delay(1000);
  }


  // Odometry
  positionLeft = 0;
  positionRight = 0;
  positionForward = 0;
  prevPositionForward = positionForward;
  positionAngular = 0; 



  // SET PINMODE OF ULTRASONIC SENSORS
  for (uint8_t i = 0; i < ULTRASONIC_NUM; i++) { 
      pinMode(ultrasonicTrigPins[i], OUTPUT);
      pinMode(ultrasonicEchoPins[i], INPUT);
  }  

  // SET TIME WHEN EACH SENSOR SENDS PULSE (one after the other to avoid interference between sensors)
  ultrasonicTimer[0] = millis() + 1000; // First pulse starts at 1000ms
  for (uint8_t i = 1; i < ULTRASONIC_NUM; i++) { // Set the starting time for each sensor.
      ultrasonicTimer[i] = ultrasonicTimer[i - 1] + ULTRASONIC_INTERVAL;
  }

  
  // SET UP MAXON MOTOR CONTROLLER ********************
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorDirectionPin, OUTPUT);
  pinMode(leftMotorPwmPin, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorDirectionPin, OUTPUT);
  pinMode(rightMotorPwmPin, OUTPUT);

  digitalWrite(leftMotorEnablePin, HIGH); 
  digitalWrite(leftMotorDirectionPin, HIGH); 
  digitalWrite(rightMotorEnablePin, HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);

  analogWrite(leftMotorPwmPin,26); // pwm duty cycle set initially to 10% (corresponds to 0 speed)
  analogWrite(rightMotorPwmPin,26); // pwm duty cycle set initially to 10% (corresponds to 0 speed)

  // SET UP DC MOTOR DRIVER *********************
  pinMode(MotorADirectionPin, OUTPUT);
  pinMode(MotorAPwmPin, OUTPUT);
  pinMode(MotorAEncoderPin, INPUT);
  pinMode(MotorBDirectionPin, OUTPUT);
  pinMode(MotorBPwmPin, OUTPUT);
  pinMode(MotorBEncoderPin, INPUT);

  digitalWrite(MotorADirectionPin, LOW);
  digitalWrite(MotorBDirectionPin, HIGH);
  analogWrite(MotorAPwmPin, 80);
  analogWrite(MotorBPwmPin, 80);



  // Variables for time measurement purposes (to not slow down everything with delay() )
  startMillis = millis(); // for polulu motors
  currentMillis = startMillis;

  fsmStartMillis = millis(); // for FSM
  fsmCurrentMillis = millis();
 
  serialStartMillis = millis(); // for serial communication (if Raspberry Pi no longer communicating to Arduino -> arduino sets speeds back to default)
  serialCurrentMillis = serialStartMillis;

  odometryStartMillis = millis(); // for odometry (get position from motor speeds)
  odometryCurrentMillis = odometryStartMillis;

  turnTime = 1500;
  // Say hello for sake of politeness
  // Serial.println("Hello from Arduinoooo");
  delay(1000); // wait 1 second


}



/********************* LOOP *********************/ 

void loop() {


  getData(); // get data from ultrasonic sensors
  // sendData(); // send data from ultrasonic sensors


  polulusOn = 1;  // WARNING set polulus on by default for each loop
  // polulu may be turned off if : 
  // 1) ordered off by readSerialPort() ("D1" -> DEBUG = true)
  // 2) ordered off by checkMotors() (Maxon motors stalling -> avoid Lipo protection board disconnecting power) 
  // 3) ordered off by checkBatteryLevel() (measured battery level too low -> save energy and avoid Lipo protection board disconnecting power)
  

  readSerialPort(); // this function may set polulus to 0 

  checkMotors(); // this function may set polulus to 0 (used in updatePoluluCommands()) and change TurnTime (used in fsm();)
  
  checkBatteryLevel(); // this function may set polulus to 0

  updatePoluluCommands(); 

  fsm();

  updateLEDs();

}
 



/********************* FUNCTIONS *********************/


void getData() {
  // Get sensor data from ultrasonic distance sensors    
  for (uint8_t i = 0; i < ULTRASONIC_NUM; i++) { // Loop through all the sensors.
      if (millis() >= ultrasonicTimer[i]) {         // Is it this sensor's time to ping?
          SonarSensor(ultrasonicTrigPins[i], ultrasonicEchoPins[i]);
          cm[i] = distance;
          ultrasonicTimer[i] += ULTRASONIC_INTERVAL * ULTRASONIC_NUM;  // Set next time this sensor will be pinged.
      }      
  }
}

void fsm() {

  switch (fsmState) {
      case 0:
        // use_random_direction = 1;
        digitalWrite(leftMotorDirectionPin, HIGH);
        digitalWrite(rightMotorDirectionPin, HIGH);
        updateMotorCommands();
        sensorAvoidance();
        break;
      case 1: // LEFT TURN OF DURATION 
        fsmCurrentMillis = millis();
        digitalWrite(leftMotorDirectionPin, LOW);
        digitalWrite(rightMotorDirectionPin, HIGH);
        analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
        analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));
        // time spent turning should be proportional to wheel speed
        if (fsmCurrentMillis - fsmStartMillis >= turnTime) {
          // fsmState = 0;
          newFSMstate(0);
        }
        break;

      case 2: // RIGHT TURN OF DURATION turnTime
        fsmCurrentMillis = millis();
        digitalWrite(leftMotorDirectionPin, HIGH);
        digitalWrite(rightMotorDirectionPin, LOW);
        analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
        analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));

        // time spent turning should be proportional to wheel speed
        if (fsmCurrentMillis - fsmStartMillis >= turnTime) {
          // fsmState = 0;
          newFSMstate(0);
        }
        break;
  
      case 3: // BACKWARDS
        fsmCurrentMillis = millis();
        digitalWrite(leftMotorDirectionPin, LOW);
        digitalWrite(rightMotorDirectionPin, LOW);
        analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
        analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));
        if (fsmCurrentMillis - fsmStartMillis >= 1500) {
          // fsmStartMillis = millis();
          // fsmState = 1;
          newFSMstate(1);
          if (use_random_direction) {
            randomNumber = (analogRead(A0)%2)+1; // random number between 1 and 2 for deciding whether to turn left or right
            // Serial.println(randomNumber);
            newFSMstate(randomNumber);
          }
        }
        break;
      case 4: // FORWARDS
        fsmCurrentMillis = millis();
        digitalWrite(leftMotorDirectionPin, HIGH);
        digitalWrite(rightMotorDirectionPin, HIGH);
        analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
        analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));

        if (fsmCurrentMillis - fsmStartMillis >= 500) {
          newFSMstate(0); 
        }
        break;

      case 5: // FORWARD and press button
        use_random_direction = 0;
        fsmCurrentMillis = millis();
        digitalWrite(leftMotorDirectionPin, HIGH);
        digitalWrite(rightMotorDirectionPin, HIGH);
        analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
        analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));

        if (fsmCurrentMillis - fsmStartMillis >= 10000) {
          newFSMstate(1); // always turn left after hitting button
          turnTime = 1500;
        }
        break;

      default:
        fsmState = 0;
        break;
    }

}

void updatePoluluCommands() {

  if (polulusOn == 1 && !DEBUG) {

      // COMMANDS TO DC MOTOR CONTROLLER
      analogWrite (MotorAPwmPin, poluluSpeed);      //PWM Speed Control   
      analogWrite (MotorBPwmPin, poluluSpeed); 

      // MEASURE POLULU MOTOR SPEED
      ontimeA = pulseIn(MotorAEncoderPin,HIGH, 250000); // timeout set to 250 ms 
      ontimeB = pulseIn(MotorBEncoderPin,HIGH, 250000);
      offtimeA = pulseIn(MotorAEncoderPin,LOW, 250000);
      offtimeB = pulseIn(MotorBEncoderPin,LOW, 250000);
      periodA = ontimeA+offtimeA;
      freqA =   1000000.0/periodA;
      periodB = ontimeB+offtimeB;
      freqB =   1000000.0/periodB;


      // Note : if value is inf (meaning period is 0), then motor is not turning (powered off or stuck)

      if (motorAstuck == 0){
        currentMillis = millis();  //get the current time 
        if (currentMillis - startMillis >= poluluPeriodUnstuck) {
          if (freqA < 200 || freqB < 200 || freqA > 5000 || freqB > 5000) {
            // Values manually tuned
            // under 200 means brushes are spinning too slow, very high value (set to 5000 instead of "inf") means brushes not moving 
            digitalWrite(MotorADirectionPin, HIGH);
            digitalWrite(MotorBDirectionPin, LOW);
            motorAstuck = 1;
            startMillis = millis();  //initial start time when motors considered stuck
          }          
        }
      }

      if (motorAstuck == 1){
        currentMillis = millis();  //get the current time 
        if (currentMillis - startMillis >= poluluPeriod) {
            digitalWrite(MotorADirectionPin, LOW);
            digitalWrite(MotorBDirectionPin, HIGH);
            motorAstuck = 0;
            // delay(100); // avoid delays using millis() below :
            startMillis = millis();  //initial start time when motors considered unstuck
        }
      }
  }

  else {
      analogWrite (MotorAPwmPin,0);      // if polulusOn == 0 or DEBUG == 1, Stop Polulu motors   
      analogWrite (MotorBPwmPin,0);
  }  
}

void SonarSensor(int trigPin,int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration/58.2; // convert measurement data to cm
}

void sensorAvoidance() {
  obstacleLeft = 0;
  obstacleRight = 0;
  int threshold = 20; // consider an obstacle to be detected when sensor ouptuts distance under 20cm (manually tuned)

  if ( (cm[2] <= threshold && cm[2]>0) || (cm[1] <= threshold && cm[1]>0) ){
    // If obstacle detected on LEFT, reverse RIGHT motor direction 
    obstacleLeft = 1;
    digitalWrite(rightMotorDirectionPin, LOW);
    analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));
    // LED displaying
    for (int i = 22; i <= 31; i++) {
      if (i%2 == 1) {
        ledStates[i] = 4; // purple
      }
    }
  }
  else {
    obstacleLeft = 0;
    for (int i = 22; i <= 31; i++) {
      if (i%2 == 1) {
        ledStates[i] = 0; // off
      }
    }
  }


  if ( (cm[3] <= threshold && cm[3]>0) || (cm[4] <= threshold && cm[4]>0) ){
    // If obstacle detected on RIGHT, reverse LEFT motor direction 
    obstacleRight = 1;
    digitalWrite(leftMotorDirectionPin, LOW);
    analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
    // LED displaying    
    for (int i = 0; i <= 9; i++) {
      if (i%2 == 1) {
        ledStates[i] = 4; // purple
      }
    }
  }
  else {
    obstacleRight = 0;
    for (int i = 0; i <= 9; i++) {
      if (i%2 == 1) {
        ledStates[i] = 0; // off
      }
    }
  }


  if (cm[0] <= 12 && cm[0]>0) { // If obstacle detected at the very front (happens very rarely in practice but still in case)
    digitalWrite(leftMotorDirectionPin, LOW);
    analogWrite(leftMotorPwmPin, map(99, 0, 99, 26, 230));
    digitalWrite(rightMotorDirectionPin, LOW);
    analogWrite(rightMotorPwmPin, map(99, 0, 99, 26, 230));
    obstacleLeft = 1;
    obstacleRight = 1;
  }




  obstacle = obstacleLeft || obstacleRight;

  if ( (prevObstacleLeft != obstacleLeft) || (prevObstacleRight != obstacleRight) ) {
    // Change of state (new obstacle detection or no more obstacle detection
    if (obstacle) {
      // new obstacle detection
      if (abs(positionForward - prevPositionForward) <= 200) { // Manually tuned
        // This means after the obstacle avoidance maneuver, robot is still back where it startet
        // -> Increment a counter, when counter reaches 5 then make big turning maneuver

        obstacleCounter++;
        if (obstacleCounter >= 5) {
          obstacleCounter = 0;
          updateSideLEDs(3);
          newFSMstate(1);
          if (use_random_direction) {
            randomNumber = (analogRead(A0)%2)+1; // random number between 1 and 2 for deciding whether to turn left or right
            newFSMstate(randomNumber);
          }
          turnTime = 1500;
          if (currentLeft >= 460 || currentRight >= 460) {
            // These values indicate robot is on carpet. As locomotion is harder, give robot more time to make maneuver
            turnTime = 2000;
          }      
        }
      }
      prevPositionForward = positionForward;   
    }   
  }
  prevObstacleLeft = obstacleLeft;
  prevObstacleRight = obstacleRight;

}

void updateMotorCommands() {

  if (left_speed < 0) {
    digitalWrite(leftMotorDirectionPin, LOW);
  }
  else {
    digitalWrite(leftMotorDirectionPin, HIGH);
  }
  if (right_speed < 0) {
    digitalWrite(rightMotorDirectionPin, LOW);
  }
  else {
    digitalWrite(rightMotorDirectionPin, HIGH);
  }
  // NOTE : to communicate with ESCON MAXON cntroller, pwm duty cycle must be between 10% and 90% (corresponds to 26 and 230)
  // left_speed and right_speed between 0 and 99
  analogWrite(leftMotorPwmPin, map(abs(left_speed), 0, 99, 26, 230));
  analogWrite(rightMotorPwmPin , map(abs(right_speed), 0, 99, 26, 230)); // desired speed sent as pwm to ESCON controller
}


void readSerialPort() {

  // Code taken from : https://www.aranacorp.com/en/serial-communication-between-raspberry-pi-and-arduino/
  // Our own protocol:
  // send motor commands as for example "m+08-99" (total string lenght of 7)
  // send unloading command as "u"
  // set back default speed values : "a" ("alone" mode, arduino does its own thing)
  // send emergency STOP command as "s"
  // disable use of brushes thru "D1" for debugging purposes (polulus set back on with "D0")
  
  msg = "";
  while (Serial.available()) {
    msg = Serial.readStringUntil('\n');
  }
  if (msg != "") {

    if (msg[0] == 'm') { // m for motor commands
      // Serial.println("MOTOR DATA");
      if (msg.length() == 7){
        String msg_forward_speed = msg.substring(1, 4);
        String msg_angular_speed = msg.substring(4,7);
        left_speed = msg_forward_speed.toInt();
        right_speed = msg_angular_speed.toInt();
        digitalWrite(leftMotorEnablePin, HIGH);
        digitalWrite(rightMotorEnablePin, HIGH);
        use_random_direction = 1;
      }
    }
    else if (msg[0]=='a') {
      // ARDUINO IN CHARGE OF EVERYTHING (speed set back to default (nominal speed))      
      left_speed = 99;
      right_speed = 99;
      digitalWrite(leftMotorEnablePin, HIGH);
      digitalWrite(rightMotorEnablePin, HIGH);
    }
    else if (msg[0]=='u'){ // u for duplo unloading
      // Serial.println("UNLOADING");
      updateSideLEDs(6);
      updateLEDs();
      unloading();
    }
    else if (msg[0]=='b') {
      // press button
      newFSMstate(5);
      updateSideLEDs(5);

    }
    else if (msg[0]=='s'){
      // Serial.println("STOP NOW!!");
      digitalWrite(leftMotorEnablePin, LOW);
      digitalWrite(rightMotorEnablePin, LOW);
    }
    else if (msg[0]=='r'){
      // resume (start) (basically undo the STOP command)
      digitalWrite(leftMotorEnablePin, HIGH);
      digitalWrite(rightMotorEnablePin, HIGH);
    }

    else if (msg[0]=='D'){
      if (msg == "D0") {
        DEBUG = 0;
      }
      else if (msg == "D1") {
        DEBUG = 1;
      }
    }
    else if (msg[0]=='d'){
      sendData();
    }
    // Serial.flush();
    serialStartMillis = millis();
  }
  else {
    serialCurrentMillis = millis();
    if (serialCurrentMillis - serialStartMillis >= 20000) { 
      // robot has 20 seconds to push button
      // gets out of this state when current sensing indicates unusually high torque (button pressed)
      // speed set back to default (nominal speed)
      left_speed = 99;
      right_speed = 99;
      use_random_direction = 1;
    }
  }
}

void sendData() {
  // send ultrasonic sensor data to Raspberry Pi
  Serial.print("d"); // start with 'd' to indicate start of message
  for(uint8_t i = 0; i < ULTRASONIC_NUM; i++) {
    Serial.print(cm[i]);
    Serial.print(" ");
  }
  Serial.println(""); // return to line indicates end of message

}

void checkBatteryLevel() {
  // Battery Level Monitoring
  // In theory 100% : >= 838, 75% : >= 798, 50% : >= 757, 25% : >= 716
  // correspond to voltages per cell ( >=4.1V, >= 3.9V, >= 3.7V, >= 3.5V )
  // Above values changed a bit below to better take into account voltage drop  

  batteryLevel = analogRead(A15);

  if (batteryLevel >= 815) {

    ledStates[14] = 2;
    ledStates[15] = 2;
    ledStates[16] = 2;
    ledStates[17] = 2;
  }

  else if (batteryLevel < 815 && batteryLevel >= 788) {

    ledStates[14] = 2;
    ledStates[15] = 2;
    ledStates[16] = 2;
    ledStates[17] = 1;
  }
  else if (batteryLevel < 788 && batteryLevel >= 751) {

    ledStates[14] = 2;
    ledStates[15] = 2;
    ledStates[16] = 1;
    ledStates[17] = 1;
  }
  else if (batteryLevel < 751 and batteryLevel >= 715) {

    ledStates[14] = 2;
    ledStates[15] = 1;
    ledStates[16] = 1;
    ledStates[17] = 1;
  }

  else if (batteryLevel < 715) {
    // NEED TO WARN RASPBERRY PI
    // Serial.println("b");
    polulusOn = 0; // TURN OFF FRONT BRUSHES TO AVOID VOLTAGE DROP THAT MAY CAUSE LIPO PROTECTION BOARD TO FULLY CUT POWER
 
    ledStates[14] = 1;
    ledStates[15] = 1;
    ledStates[16] = 1;
    ledStates[17] = 1;
  }
}

void checkMotors() {
  speedLeft = analogRead(A9);
  currentLeft = analogRead(A8);
  speedRight = analogRead(A4);
  currentRight = analogRead(A5);

  // ODOMETRY ------------------------------------------------
  odometryStartMillis = odometryCurrentMillis;
  odometryCurrentMillis = millis();
  long deltaT = odometryCurrentMillis - odometryStartMillis;

  // MAXON MOTOR ENCODER OUPTUTS VOLTAGE BETWEEN 0 AND 4V
  // 0V corresponds to full negative speed/current, 4V to full positive speed/current, 2V for zero speed/current
  // from analyzing data received by Arduino thru analogRead, zero speed/current corresponds to around a value of 408
  positionLeft = (speedLeft - 407) * deltaT + positionLeft; 
  positionRight = (speedRight - 408) * deltaT + positionRight;

  positionForward = (positionRight + positionLeft)/1000; // global variable used later in sensorAvoidance();
  // positionAngular = (positionRight - positionLeft)/1000;
 
  // CURRENT SENSING ------------------------------------------------
  if (currentLeft >= 560 || currentRight >= 580) {
    // values manually tuned
    // motors stuck going forwards
    // turn off polulu motors to avoid too much current draw
    polulusOn = 0;
    newFSMstate(3); 
    updateSideLEDs(1); 
    turnTime = 800; 
  }
  else if ( (currentLeft <= 260 && currentLeft >= 10) || (currentRight <= 260 && currentRight >= 10) ) {
    // values manually tuned
    // motors stuck going backwards
    // added >= 10 to avoid polulus turning off when no proper analog input detected
    // turn off polulu motors to avoid too much current draw
    polulusOn = 0;
    newFSMstate(4);
    updateSideLEDs(1);
  }
}

void newFSMstate(int stateNumber = 0) {
  fsmStartMillis = millis();
  fsmState = stateNumber;
}

void updateLEDs() {
  // ONLY UPDATES LEDS WHEN CHANGE OF STATE -> avoids unnecessary delays and makes robot more responsive
  for (uint8_t i = 0;  i < NUM_LEDS; i++) {
    if (prevLedStates[i] != ledStates[i]) {
      switch (ledStates[i]) {
        case 0:
          leds[i] = CRGB ( 0, 0, 0); // off
          break;
        case 1:
          leds[i] = CRGB ( 10, 0, 0); // red (for battery level indication and maneuver due to unusually high torque)
          break;
        case 2:
          leds[i] = CRGB ( 0, 10, 0); // green (for battery level)
          break;
        case 3:
          leds[i] = CRGB ( 0, 0, 10); // blue (for maneuver indication due if robot detected being stuck)
          break;
        case 4: 
          leds[i] = CRGB ( 10, 0, 10); // pink (leds turn pink when obstacle detected on corresponding side)
          break;
        case 5: 
          leds[i] = CRGB ( 10, 8, 0); // yellow (for "run straight into button" mode)
          break;
        case 6: 
          leds[i] = CRGB ( 0, 10, 10); // turquoise
          break;
        default:
          leds[i] = CRGB ( 0, 0, 0); // off
          break;
      }

      FastLED.show(); 
      prevLedStates[i] = ledStates[i];     
    }
  }
}

void updateSideLEDs(int color) {
    for (int i = 0; i <= 9; i++) {
      if (i%2 == 1) {
        ledStates[i] = color; 
      }
    }
    for (int i = 22; i <= 31; i++) {
      if (i%2 == 1) {
        ledStates[i] = color; // purple
      }
    }
}

void analogWriteCommand(int startPos, int endPos) {

  for (pos = startPos; pos <= endPos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    bigAssServo.write(pos);              // tell servo to go to position in variable 'pos'
    // the following code inverses motor direction speed to shake the duplo container
    if (pos%30 == 0) {
      int pos_divided_by_30 = pos/30;
      if (pos_divided_by_30%2 == 0) {
        digitalWrite(leftMotorDirectionPin, HIGH); 
        digitalWrite(rightMotorDirectionPin, LOW); // 
      }
      else {
        digitalWrite(leftMotorDirectionPin, LOW);
        digitalWrite(rightMotorDirectionPin, HIGH);
      }
    }
    delay(15);                       // waits 15 ms for the servo to reach the position
  }

  digitalWrite(leftMotorDirectionPin, HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);

}

void adafruitCommand(int startPulse, int endPulse) {

  for (uint16_t pulselen = startPulse; pulselen < endPulse; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    if (pulselen%100 == 0) {
      int pos_divided_by_100 = pulselen/100;
      if (pos_divided_by_100%2 == 0) {
        digitalWrite(leftMotorDirectionPin, HIGH);
        digitalWrite(rightMotorDirectionPin, LOW);
      }
      else {
        digitalWrite(leftMotorDirectionPin, LOW);
        digitalWrite(rightMotorDirectionPin, HIGH);
      }
    }
    delay(3);
  }
  delay(3);
  digitalWrite(leftMotorDirectionPin, HIGH);
  digitalWrite(rightMotorDirectionPin, HIGH);
}

void moveServos() {
  // two cases (for debugging and modularity) : if using adafruit servo driver or if using analogWrite pin directly on Arduino

  if (use_adafruit_driver) {
    adafruitCommand(SERVOMIN, SERVOMAX); // tilt duplo container up to make them fall out
    adafruitCommand(SERVOMAX, SERVOMIN); // tilt duplo container back to flat position
    pwm.setPWM(0, 0, 0);  // turn off PWM signal to channel 0 to avoid servo motor humming 
  }

  else {
    analogWriteCommand(30, 165);
    analogWriteCommand(165, 30);
  }
}

void unloading() {
      digitalWrite(leftMotorEnablePin, HIGH);
      digitalWrite(rightMotorEnablePin, HIGH);
      
      digitalWrite(MotorADirectionPin, HIGH); // front brushes turn in outwards direction to push any duplos out that remain too close to the brushes
      digitalWrite(MotorBDirectionPin, LOW);

      moveServos();
      delay(100);
      moveServos(); // tilt duplo container up and down a second time to make sure all duplos fall out

      
      digitalWrite(MotorADirectionPin, LOW); // front brushes turning back inwards for duplo collection
      digitalWrite(MotorBDirectionPin, HIGH);
      digitalWrite(leftMotorDirectionPin, HIGH);
      digitalWrite(rightMotorDirectionPin, HIGH);
}


