#include <Arduino.h>
#include <Servo.h>

class BlueMotor{
    public:
        int PWM;
        int INPUT1;
        int INPUT2;
        int STBY;
        int ENCODER1;
        int ENCODER2;
        Encoder* encoder;
        int encoderCount = 0;
        const int countPerDegree = 10; //PlaceHolder value, fix later
        int targetCount = 0;

        BlueMotor(void);
        void setValues(int PWM, int INPUT1,int INPUT2, int STBY, int ENCODER1, int ENCODER2);
        void setup();
        void setDirection(int direction);
        void setAngle(int theta);
        void moveMotor();
        void setSpeed(int speed);
        
};




class Arm{
    public:
        Servo servo;
        BlueMotor motor;
        int servoPin;

        Arm(int servoPin, int PWM, int INPUT1,int INPUT2, int STBY, int ENCODER1, int ENCODER2);
        void setup();
        void setPos(float x, float y);
        void moveMotors();
};