#include <Servo.h>
class Gripper{
    public:
        int openPos = 130;
        int closedPos = 180;
        int targetServo = 130;
        int servoPin;
        Servo servo;
        long timer = 0;
        bool stopped = false;

        Gripper(int servoPin){
            this->servoPin = servoPin;
        };
        
        void setAngle(int deg);
        void setup();
        void moveMotors();
        void open();
        void close();
        void stopMotor();
};