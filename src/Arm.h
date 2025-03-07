#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

class BlueMotor{
    public:
        bool motorStopped = false;
        int PWM;
        int INPUT1;
        int INPUT2;
        int STBY;
        int ENCODER1;
        int ENCODER2;
        Encoder* encoder;
        long encoderCount = 0;
        const float countPerDegree = 1.65 *20;
        int targetCount = 0;
        int targetAngle = 0;
        int currentSpeed = 0;
        float Kp = 1.5;

        BlueMotor(void){};
        void setValues(int PWM, int INPUT1,int INPUT2, int STBY, int ENCODER1, int ENCODER2);
        void setup();
        void setDirection(int direction);
        void setAngle(int theta);
        void moveMotor();
        void setSpeed(int speed);
        int testEncoder();
        void stopMotor();
        
};




class Arm{
    public:
        bool stopped = false;
        Servo servo;
        BlueMotor motor;
        int servoPin;

        int currentX = 200;
        int currentY = 125;
        int targetX = 200;
        int targetY = 125;

        //Link Constants:
        const int L1 = 155;
        const int L2 = 150;
        const int L3 = 130;
        const int L4 = 135;
        const int base = 100;
        int servoTarget = 0;
        long timer = 0;
        int prevServo = 0;
        int servoOffset = 130;


        Arm(int servoPin, int PWM, int INPUT1,int INPUT2, int STBY, int ENCODER1, int ENCODER2);
        void setup();
        void setPos(float x, float y);
        void setPos();
        void moveMotors();
        void stopMotors();
        void setAngleServo(int angle);
        int degreesToMicros(int degrees);
};