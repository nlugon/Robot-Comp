# Interdisciplinary Robot Competition


![leroboteny](https://github.com/nlugon/Robot-Comp/assets/112929907/da167349-9591-4af0-8201-b0f38015a03d)


Competition website [here](https://robot-competition.epfl.ch) 

Video of the robot [here (youtube)](https://www.youtube.com/watch?v=vqCzc3zlJ8U)

Robot Competition Report [here](https://drive.google.com/file/d/1sISDkwHlUgYIt9PJVt_OxYXLAwWDXRC6/view)


## Authors
- [Noah Lugon](https://github.com/nlugon)
- [Clarence Linden](https://gitlab.epfl.ch/linden)
- [Adrien Chevallier](https://gitlab.epfl.ch/achevall)


## Electrical Components
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

## Features
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

