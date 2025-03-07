#include "Gripper.h"
#include <Arduino.h>

void Gripper::setAngle(int deg){
    if(deg>=90 && deg<=180){
        targetServo = deg;
    }


}

void Gripper::moveMotors(){
    if(stopped){
        return;
    }
    if(millis()-timer > 15){
        timer = millis();
        int currentServo = servo.read();
        if(targetServo > currentServo){
            servo.write(currentServo + 1);
        }
        else if(targetServo < currentServo){
            servo.write(currentServo - 1);
        }
    }
}

void Gripper::open(){
    setAngle(openPos);
}

void Gripper::close(){
    setAngle(closedPos);
}

void Gripper::setup(){
    servo.attach(servoPin);
}

void Gripper::stopMotor(){
    stopped = true;
}