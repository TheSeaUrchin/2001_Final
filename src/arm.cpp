#include "Arm.h"
#include <Encoder.h>

//Arm Commands

Arm::Arm(int servoPin, int PWM, int INPUT1,int INPUT2, int STBY, int ENCODER1, int ENCODER2){
    this->servoPin = servoPin;
    motor.setValues(PWM, INPUT1,INPUT2,STBY,ENCODER1,ENCODER2);
}

void Arm::setup(){
    motor.setup();
    servo.attach(servoPin);
}



//Blue Motor Commands

void BlueMotor::setValues(int PWM, int INPUT1,int INPUT2, int STBY, int ENCODER1, int ENCODER2){
    this->PWM = PWM;
    this->INPUT1 = INPUT1;
    this->INPUT2 = INPUT2;
    this->STBY = STBY;
    this->ENCODER1 = ENCODER1;
    this->ENCODER2 = ENCODER2;
}
void BlueMotor::setup(){
    pinMode(PWM,OUTPUT);
    pinMode(STBY,OUTPUT);
    pinMode(INPUT1,OUTPUT);
    pinMode(INPUT2,OUTPUT);
    // digitalWrite(STBY,HIGH);
    encoder = new Encoder(ENCODER1, ENCODER2);
}

void BlueMotor::setDirection(int direction){
    if(direction > 0){
        digitalWrite(INPUT1,LOW);
        digitalWrite(INPUT2,HIGH);
    }
}

void BlueMotor::setAngle(int theta){
    targetCount = (countPerDegree) * theta;
    int currentCount = encoder->read();
}

//Move motor 1 degree at a time, may reduce if movement to jerky
void BlueMotor::moveMotor(){
    int tolerance = 1;
    encoderCount = encoder->read();
    if(targetCount>=encoderCount){
        setDirection(1);
    }
    else{
        setDirection(-1);
    }
    if(abs(targetCount-encoderCount) > tolerance){
        digitalWrite(STBY,HIGH);
    }
    else{
        digitalWrite(STBY,LOW);
    }
    
}

//May need to add PID if too jerky
void BlueMotor::setSpeed(int speed){
    analogWrite(PWM,speed);
}

