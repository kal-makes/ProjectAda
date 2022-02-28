#ifndef _PID_H_
#define _PID_H_


class PID{
  private:
    int turnError;
    int prevTurnError = 0;
    int total_turnError = 0;
    int turnDerivative;
    bool complete = false;
    int counter = 0;
    float error = 0;
    int prevError=0;
    int totalError = 0;
    int derivative;
    float pctError;

    double kP = 0.03;//0.025
    double kD = 0.9;//0.9
    double kI= 0.0;

    double turnKp = 3.0; //3.0
    double turnKd = 0.1; //0.1
    double turnKi = 0.0;

    double sonar_kP = 7.0;
    double sonar_kD = 0.0;
    double sonar_kI = 0.0;
    
    double lift_kP = 5.5;
    double lift_kI = 0.0;
    double lift_kD = 0.0;

    void drivePID();
    
  public:
    float desiredPos;
    float desiredTurn;
    void startPID( float lat_setpoint, float rot_setpoint);
};
#endif