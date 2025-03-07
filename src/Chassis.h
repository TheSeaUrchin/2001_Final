#include <Romi32U4Encoders.h>
#include <Romi32U4Motors.h>
class Chassis{
    public:
    Romi32U4Encoders encoders;
    Romi32U4Motors motors;

    double Kp = 0.7; //Kp value
    double Ki = 0; //Ki value
    int targetLeft;
    int targetRight;
    int wheelD = 7;
    double ticks_per_cm = 60.57; // ticks/cm
    double ticks_per_deg = 1450/360 * 2.5; // ticks/degree

    //Was for integral control, not being used
    double errorSumL = 0;
    double errorSumR = 0;

    Chassis(void){};
    void drive(int direction, int distance);
    void turn(int direction, int distance);
    void moveMotors();
};