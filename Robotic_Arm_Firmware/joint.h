/** ------------------------------------------------------------------------------------------------------------------------------------------- */
/*
 * Firmware:      Robotic Arm Exhibit
 * Description:   Cylindrical-coordinate control system
 * Location:      Virginia Beach Marine Science Center
 *                "Reaching Out for Clues" Exhibit
 * Author:        Imran A. Uddin
 *                Old Dominion University
 * Includes:      Robotic_Arm_Firmware.ino
 *                joint.h (this file)
 *                joint.cpp
 *                coor.h
 *                coor.cpp
 * Notes:         Please see Robotic_Arm_Firmware.ino for version information.
 */
/** ------------------------------------------------------------------------------------------------------------------------------------------- */

#ifndef JOINT_H
#define JOINT_H

#include "Arduino.h"
#include <Servo.h>

/** joint Object Class
 *  The joint object creates an instance of a data structure encompassing a single servo motor.
 *  The object provides parameters and functions to manipulate the attached servo.
*/
  class joint {
    public:
      joint();
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
