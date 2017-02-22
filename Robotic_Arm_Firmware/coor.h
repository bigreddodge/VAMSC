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
 *                coor.h (this file)
 *                coor.cpp
 * Notes:         Please see Robotic_Arm_Firmware.ino for version information.
 */
/** ------------------------------------------------------------------------------------------------------------------------------------------- */

#ifndef COOR_H
#define COOR_H

/** coor Object Class
 *  The coor object implements a tandem cylindrical/cartesian coordinate system describing
 *  the position of the claw. Cylindrical coordinates entered are immediately accessible in
 *  cartesian format and vv. The object also describes the actual commanded claw position.
*/
  class coor {
  	public:
  		coor();
      coor(double ri, double ti, double zi);
  		~coor();
  		void setX(double xc);
  		void setY(double yc);
  		void setZ(double zc);
  		void setR(double rc);
  		void setT(double tc);
  		double X();
  		double Y();
  		double Z();
  		double R();
  		double T();
  	private:
  		void updateCyl();
  		void updateCart();
  		double x,y,z,r,t;
  };
#endif // COOR_H
