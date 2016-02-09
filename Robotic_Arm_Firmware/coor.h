#ifndef COOR_H
#define COOR_H

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
#endif
