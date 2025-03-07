#include "Chassis.h"

#include <Arduino.h>
#include <Romi32U4.h>

//Update motor position
void Chassis::moveMotors(){
    int errorL = targetLeft - encoders.getCountsLeft();
    int errorR = targetRight - encoders.getCountsRight();
    motors.setSpeeds(Kp * errorL, Kp * errorR );
}

//Set driving forwards/backwards and distance
void Chassis::drive(int direction,int distance){
    targetLeft = encoders.getCountsLeft() + ticks_per_cm * direction * distance;
    targetRight = encoders.getCountsRight()+ ticks_per_cm * direction * distance;
}

//Set turning parameters
void Chassis::turn(int direction, int distance){
    targetLeft = encoders.getCountsLeft() + (ticks_per_deg * distance * direction);
    targetRight = encoders.getCountsRight() - (ticks_per_deg * distance * direction);
}