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
 *                joint.cpp
 *                coor.h
 *                coor.cpp (this file)
 * Notes:         Please see Robotic_Arm_Firmware.ino for version information.
 */
/** ------------------------------------------------------------------------------------------------------------------------------------------- */

#include "coor.h"
#include <math.h>

// Class Default Constructor
coor::coor() {
	r = 12;
	t = M_PI / 2;
	z = 12;
	updateCart();
}

// Class Constructor
coor::coor(double ri, double ti, double zi) {
  r = ri;
  t = ti;
  z = zi;
  updateCart();
}

// Class Destructor
coor::~coor(){}

// Modifies the X Cartesian coordinate and updates corresponding Cylindrical coordinates.
void coor::setX(double xc) {
	x = xc;
	updateCyl();
}

// Modifies the Y Cartesian coordinate and updates corresponding Cylindrical coordinates.
void coor::setY(double yc) {
	y = yc;
	updateCyl();
}

// Modifies the Z Cartesian/Cylindrical coordinate.
void coor::setZ(double zc) {z = zc;}

// Modifies the R Cylindrical coordinate and updates corresponding Cartesian coordinates.
void coor::setR(double rc) {
	r = rc;
	updateCart();
}

// Modifies the T Cylindrical coordinate and updates corresponding Cartesian coordinates.
void coor::setT(double tc) {
	t = tc;
	updateCart();
}

// Returns the respective coordinate
double coor::X(){return x;}
double coor::Y(){return y;}
double coor::Z(){return z;}
double coor::R(){return r;}
double coor::T(){return t;}

// Recalculates Cylindrical coordinates from their Cartesian counterparts.
void coor::updateCyl() {
	r = sqrt(pow(x,2) + pow(y,2));
	t = atan2(y, x);
}

// Recalculates Cartesian coordinates from their Cylindrical counterparts.
void coor::updateCart() {
	x = r * cos(t);
	y = r * sin(t);
}
