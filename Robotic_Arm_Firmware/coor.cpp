#include "coor.h"
#include <math.h>
	coor::coor() {
		r = 12;
		t = M_PI / 2;
		z = 12;
		updateCart();
	}
        coor::coor(double ri, double ti, double zi) {
                r = ri;
                t = ti;
                z = zi;
                updateCart();
        }

	coor::~coor(){}

	void coor::setX(double xc) {
		x = xc;
		updateCyl();
	}

	void coor::setY(double yc) {
		y = yc;
		updateCyl();
	}

	void coor::setZ(double zc) {z = zc;}

	void coor::setR(double rc) {
		r = rc;
		updateCart();
	}

	void coor::setT(double tc) {
		t = tc;
		updateCart();
	}

	double coor::X(){return x;}
	double coor::Y(){return y;}
	double coor::Z(){return z;}
	double coor::R(){return r;}
	double coor::T(){return t;}

	void coor::updateCyl() {
		r = sqrt(pow(x,2) + pow(y,2));
		t = atan2(y, x);
	}

	void coor::updateCart() {
		x = r * cos(t);
		y = r * sin(t);
	}
