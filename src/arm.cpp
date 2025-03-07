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
    // servo.writeMicroseconds(degreesToMicros(0));

    delay(1000);
}


//IK calculations
void Arm::setPos(float x, float y){

    Serial.println("Called SetPos");

    Serial.print("X: ");
    Serial.println(x);

    Serial.print("Y: ");
    Serial.println(y);
    //Limits:

    if(x<30 || x > 300){
        return;
    }

    if(y<10 || y > 270){
        return;
    }

    




    //Theta 1
    float AC = sqrt(pow(x,2) + pow(y,2));
    float temp = float(pow(L2,2)-pow(L1,2)-pow(AC,2))/float(-2*L1*AC);
    if(abs(temp)>1){
        return;
    }
    float a = acos(temp);
    float b = acos(x/AC);
    int theta1 = (a+b)*(180/3.14);

    //Theta 2
    float EC = sqrt(pow(base-x,2) + pow(y,2));
    float temp2 = float(pow(L3,2)-pow(L4,2)-pow(EC,2))/float(-2*L4*EC);
    float a2 = acos(temp2);
    float b2 = acos((base-x)/EC);
    if(abs(temp2)>1){
        return;
    }
    int theta2 = (a2+b2)*(180/3.14);

    
    Serial.println(theta1);
    Serial.println(180-theta2);


    //Set motors to move to these positions
    servoTarget = min(max(0,servoOffset - theta1),180);
    motor.setAngle(180-theta2);
    targetX = x;
    targetY = y;



}

void Arm::moveMotors(){
    int stepSize = 1;

    //Dont move if emergency stopped
    if(stopped){
        return;
    }
    if(millis()-timer>20){
        timer = millis();
        int currentServo = servo.read();
        if(servoTarget > currentServo){
            //servo.writeMicroseconds(degreesToMicros(currentServo + stepSize));
            servo.write(currentServo + stepSize);
        }
        else if(servoTarget < currentServo){
            //servo.writeMicroseconds(degreesToMicros(currentServo-stepSize));
            servo.write(currentServo-stepSize);
        }
    }
    motor.moveMotor();
}

//Emergrncy stop all motors
void Arm::stopMotors(){
    stopped = true;
    motor.stopMotor();
}

//Mainly for testing, set target servo angle
void Arm::setAngleServo(int angle){
    if(angle <= 180 && angle >= 0){
        servoTarget = angle;
    }
    Serial.println(servoTarget);
}

int Arm::degreesToMicros(int degrees){
    degrees = degrees - 90;
    return (1500 + (degrees / 90) * 1000);
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
    //Configure pins and encoder
    pinMode(PWM,OUTPUT);
    pinMode(STBY,OUTPUT);
    pinMode(INPUT1,OUTPUT);
    pinMode(INPUT2,OUTPUT);
    digitalWrite(STBY,HIGH);
    encoder = new Encoder(ENCODER1, ENCODER2);
}

void BlueMotor::setDirection(int direction){
    //Set direction of blue motor
    if(direction > 0){
        digitalWrite(INPUT1,LOW);
        digitalWrite(INPUT2,HIGH);
    }
    if(direction < 0){
        digitalWrite(INPUT1,HIGH);
        digitalWrite(INPUT2,LOW);
    }
}

void BlueMotor::setAngle(int theta){
    //set target encoder count using angle
    targetAngle = theta;
    targetCount = (countPerDegree) * theta;
    Serial.println(targetCount);
}

//Use proportional control to move motor
void BlueMotor::moveMotor(){
    if(motorStopped){
        return;
    }
    int tolerance = 3; //Encoder error tolerance
    encoderCount = encoder->read();
    if(targetCount>=encoderCount){
        setDirection(1);
    }
    else{
        setDirection(-1);
    }
    int error = abs(targetCount-encoderCount);
    if(error > tolerance){
        int speed = min(error*Kp,255);
        digitalWrite(STBY,HIGH);
        analogWrite(PWM,speed);
    }
    else{
        analogWrite(PWM,0);
        digitalWrite(STBY,LOW);
    }
    
}

void BlueMotor::setSpeed(int speed){
    currentSpeed = speed;
}

//Get encoder count (Mainly for testing)
int BlueMotor::testEncoder(){
    return encoder->read();

}

//Emergency Stop Motor
void BlueMotor::stopMotor(){
    digitalWrite(INPUT1,LOW);
    digitalWrite(INPUT2,LOW);
    digitalWrite(PWM,0);
    digitalWrite(STBY,LOW);
    motorStopped = true;
}