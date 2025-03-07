#include <Arduino.h>
#include "Arm.h"
#include "Chassis.h"
#include "Gripper.h"

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

//Define robot parts
Arm robotArm(5,6,18,14,4,0,1);
Chassis chassis;
Gripper gripper(11);
BlueMotor motor;


int count = 0;
//Mode used to switch between controlling arm via degrees and via IK (degrees needed to calibrate blue motor)
int mode = 1;
long int comsTimer = 0;

void getCommand();

void setup() {
  Serial.begin(9600);
  robotArm.setup();
  gripper.setup();

  // robotArm.servoTarget = 0;
  robotArm.servo.write(0);
  robotArm.servoTarget = 0;
}

void loop() {
  getCommand();

  //Update Motor Positions
  chassis.moveMotors();
  gripper.moveMotors();
  robotArm.moveMotors();

}

//Get commands from serial
void getCommand(){
  if(Serial.available() && millis()-comsTimer>15){
    comsTimer = millis();
    char input = Serial.read();

    switch(input){
      //Motor Driving
      case 's':
        chassis.drive(1,1); //Random velocity, should fix
        break;

      case 'w':
        chassis.drive(-1,1);
        break;

      case 'd':
        chassis.turn(1,5);
        break;

      case 'a':
        chassis.turn(-1,5);
        break;

      //Arm
      
      case 'i':
        if(mode == 1){
          Serial.println(robotArm.servo.read());
          robotArm.setAngleServo(robotArm.servoTarget+5);
        }
        else{
          robotArm.setPos(robotArm.targetX,robotArm.targetY+5);
        }
        
        break;

      case 'k':
        // 
        if(mode == 1){
          robotArm.setAngleServo(robotArm.servoTarget-5);
        }
        else{
          robotArm.setPos(robotArm.targetX,robotArm.targetY-5);
        }
        
        break;

      case 'j':
        if(mode == 1){
          robotArm.motor.setAngle(robotArm.motor.targetAngle+1);
        }
        else{
          robotArm.setPos(robotArm.targetX-5,robotArm.targetY);
        }

        break;

      case 'l':
        if(mode == 1){
          robotArm.motor.setAngle(robotArm.motor.targetAngle-1);
        }
        else{
          robotArm.setPos(robotArm.targetX+5,robotArm.targetY);
        }
        break;

      //Gripper

      case '[':
          gripper.setAngle(gripper.targetServo-1);
          Serial.println(gripper.targetServo);
          break;
      case ']':
          gripper.close();
          gripper.setAngle(gripper.targetServo+1);
          Serial.println(gripper.targetServo);
          break;


    
      //reset blue motor encoder
      case 'r':
        robotArm.motor.encoder->readAndReset();
        robotArm.motor.targetCount = 0;
        break;

      //emergency break
      case 'b':
        robotArm.stopMotors();
        gripper.stopMotor();
        break;
      
      //change mode to testing
      case 't':
        mode = 1;
        break;

      //change mode to operation
      case 'o':
        mode = 0;
        break;
    }
  }

}



// #include <Arduino.h>
// #include <Romi32U4.h>
// #include <Servo.h>

// Romi32U4Encoders encoders;
// Romi32U4LCD lcd;
// Romi32U4Buzzer buzzer;
// Romi32U4Motors motors;
// Romi32U4ButtonA buttonA;
// Romi32U4ButtonC buttonC;

// const char encoderErrorLeft[] PROGMEM = "!<c2";
// const char encoderErrorRight[] PROGMEM = "!<e2";

// char report[80];
// int val;
// double Kp = 0.7; //Kp value
// // double Ki = 0.01;
// int targetLeft;
// int targetRight;
// int wheelD = 7;
// double ticks_per_cm = 60.57; // ticks/cm
// double ticks_per_deg = 1450/360 * 2.5; // ticks/degree
// double errorSumL = 0;
// double errorSumR = 0;
// long timer = millis();
// int ledState = 0;
// int servoPos = 90;
// int servoStage = 0;

// Servo myservo;
// void countUp();

// void setup() {
//   // put your setup code here, to run once:
//     Serial.begin(9600);
//     myservo.attach(11);
// }

// void loop() {
//   countUp();
//   if(millis() - timer >= 1000){
//     timer = millis();
//     digitalWrite(LED_BUILTIN,ledState);
//     ledState = !ledState;
//   }
//   static uint8_t lastDisplayTime;

//   if ((uint8_t)(millis() - lastDisplayTime) >= 100)
//   {
//     lastDisplayTime = millis();

//     int16_t countsLeft = encoders.getCountsLeft();
//     int16_t countsRight = encoders.getCountsRight();

//     bool errorLeft = encoders.checkErrorLeft();
//     bool errorRight = encoders.checkErrorRight();

    
//     // Send the information to the serial monitor also.
//     snprintf_P(report, sizeof(report),
//         PSTR("%6d %6d %1d %1d"),
//         countsLeft, countsRight, errorLeft, errorRight);
//     Serial.println(report);
//   }


//   if(Serial.available()) 
//   {
//     int16_t countsLeft = encoders.getCountsLeft();
//     int16_t countsRight = encoders.getCountsRight();

//     val = Serial.read();
//     if (val == 'd')
//     {
//       Serial.print("10 degrees CW");
//        targetLeft =  countsLeft + (ticks_per_deg * 10);
//        targetRight = countsRight - (ticks_per_deg * 10);
//     }
//     if (val == 'a')
//     {
//       Serial.print("10 degrees CCW");
//        targetLeft = countsLeft - (ticks_per_deg * 10);
//        targetRight = countsRight + (ticks_per_deg * 10);      
//     }
//     if (val == 'w')
//     {
//       Serial.print("Forward for 1 cm");
//        targetLeft = countsLeft + (ticks_per_cm);
//        targetRight =  countsRight + (ticks_per_cm);
//     }
//     if (val == '2')
//     {
//       Serial.print("Forward for 5 cm");
//        targetLeft = countsLeft + (ticks_per_cm * 5);
//        targetRight =  countsRight + (ticks_per_cm * 5);
//     }
//     if (val == 's')
//     {
//       Serial.print("Backwards for 1 cm");
//        targetLeft = countsLeft - (ticks_per_cm);
//        targetRight =  countsRight - (ticks_per_cm);
//     }

//     if (val == 'x')
//     {
//       Serial.print("Backwards for 5 cm");
//        targetLeft = countsLeft - (ticks_per_cm * 5);
//        targetRight =  countsRight - (ticks_per_cm * 5);
//     }
//   }
    
//     double errorL = targetLeft - encoders.getCountsLeft();
//     double errorR = targetRight - encoders.getCountsRight();



//     errorSumL+=errorL;
//     errorSumR+=errorR;
//     // motors.setSpeeds(Kp * errorL+Ki*errorSumL, Kp * errorR + Ki*errorSumR);
//     motors.setSpeeds(Kp * errorL, Kp * errorR );

    
 
// }

// void countUp() {
//   if(millis()-timer< 15){
// //    Serial.println("Hello");
//     return;
//   }
//   timer = millis();


//   if(servoStage == 0) { // goes from 0 degrees to 180 degrees
//     // Serial.println("Hello");
//     // in steps of 1 degree
//     Serial.println(servoPos);
//     myservo.write(servoPos);              // tell servo to go to position in variable 'pos'
//     servoPos++;
//   }
//   if(servoPos == 180){
//     servoStage = 1;
//   }
//   if(servoStage == 1) { // goes from 180 degrees to 0 degrees
//     Serial.println(servoPos);
//     myservo.write(servoPos);              // tell servo to go to position in variable 'pos'
//     servoPos-=1;
//   }
//   if(servoPos == 90){
//     servoStage = 0;
//   }
// }

