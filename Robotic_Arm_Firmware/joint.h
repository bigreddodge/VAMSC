#ifndef JOINT_H
#define JOINT_H

//#include <math.h>
#include "Arduino.h"
#include <Servo.h>

class joint
{
    public:
        joint(int uSmin, int uSmax, int anglemin, int anglemax, bool slaveselect, int pin);
        ~joint();
        void Start();
        void Stop();
        void Move(double angle);
        double ReturnAngle();
    private:
        int MinMicros, MaxMicros;
        int MinAngle, MaxAngle;
        int interrupt;
        int slaveoffset;
        int uS;
        bool slave;
        Servo srv;
};

#endif // JOINT_H
