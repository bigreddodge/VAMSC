/** ------------------------------------------------------------------------------------------------------------------------------------------- */
/*
 * Firmware:      Robotic Arm Exhibit
 * Description:   Cylindrical-coordinate control system
 * Location:      Virginia Beach Marine Science Center
 *                "Reaching Out for Clues" Exhibit
 * Author:        Imran A. Uddin
 *                Old Dominion University
 * Includes:      Robotic_Arm_Firmware.ino
 *                joint.h
 *                joint.cpp (this file)
 *                coor.h
 *                coor.cpp
 * Notes:         Please see Robotic_Arm_Firmware.ino for version information.
 */
/** ------------------------------------------------------------------------------------------------------------------------------------------- */

#include "joint.h"

// Class Default Constructor
joint::joint(){};

// Class Constructor
joint::joint(int uSmin, int uSmax, int anglemin, int anglemax, bool slaveselect, int pin) {
  // Initialize
    MinMicros = uSmin;
    MaxMicros = uSmax;
    MinAngle = anglemin;
    MaxAngle = anglemax;
    slave = slaveselect;
    slaveoffset = 3000;
    interrupt = pin;
}

// Class Destructor
joint::~joint(){}

// Attaches the assigned interrupt, powering up the servo
void joint::Start(){srv.attach(interrupt);}

// Detaches the assigned interrupt, powering down the servo
void joint::Stop(){srv.detach();}

// Returns current joint angle
double joint::ReturnAngle(){return map(uS, MinMicros, MaxMicros, MinAngle, MaxAngle);}  

/* Converts the angle argument into a PWM value and writes it to the servo.
 * If the servo is in the slave configuration, the PWM value is adjusted. */
void joint::Move(double angle) {
  //uS = map(angle, MinAngle, MaxAngle, MinMicros, MaxMicros);  //Map angle (degrees) to microseconds
    uS = angle * (MaxMicros - MinMicros) / (MaxAngle - MinAngle) + MinMicros;
    if (slave)                  // Handle slave offset
      uS = slaveoffset - uS;
  srv.writeMicroseconds(uS);
}

