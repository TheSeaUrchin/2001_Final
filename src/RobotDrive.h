class RobotDrive{
    public:
        RobotDrive(void){

        };
        void drive(int direction, int velocity);
        void turn(int direction, int velocity);
        int getInputSerial();
        int getInputBT();
};