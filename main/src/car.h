#ifndef CAR_H
#define CAR_H


#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>


using namespace std;




class car
{
public:
    double length;  //car length in m
    double width;   //car width  in m
    double x;       // position x of car in m
    double y;       // position y of car in m
    double ang;     // angle  of car (0---360) in degree
    double sita;    // angle of wheel (-60 --- 60) in degree  right is positive; left is negtive
    double sitaMin;    // min angle of wheel (-60) in degree by default
    double sitaMax;    // max angle of wheel ( 60) in degree by default

    double xc;
    double yc;
	double r;
	double rfront;
	car(double minS = -60, double maxS = 60) {
		setSitaRange(minS, maxS);
	}
    car(double L, double W, double minS , double maxS ) {
		setLW(L, W);
		setSitaRange(minS, maxS);
    }

	void setLW(double L, double W) {
		length = L;
		width = W;
	}
	void setSitaRange(double minS = -60, double maxS = 60) {
		sitaMin = minS;
		sitaMax = maxS;
	}

    void setCarPosAngle(double px, double py, double body, double wheel) {
		setCarPos(px,py);
        setAng(body);
        setSita(wheel);
    }
    void setAng(double a) {
        int r = int(a / 360.0);
        ang = a - 360.0 * r;
        if (ang < 0) ang = ang + 360.0;
    }
	void setCarPos(double px, double py) {
		x = px;
		y = py;
	}

    void setSita(double a) {
        if (a > sitaMax) { sita = sitaMax; return; }
        if (a < sitaMin) { sita = sitaMin; return; }
        sita = a;
    }
	void setPos(double xp,double yp) {
		x = xp;
		y = yp;
	}
	void move(double dis) {

		double a = ang * 2 * M_PI / 360.0;
		double s = sita * 2 * M_PI / 360.0;

		double A;
		double sh;
		double dx;
		double dy;



		if (sita == 0.0) {
			x = x + dis * cos(a);
			y = y + dis * sin(a);
			r = -1;
			xc = -1;
			yc = -1;
			return;
		}

		if (sita > 0) {

			A = M_PI / 2 - s;
			r = length / cos(A);
			
			xc = x - cos(a) * width / 2;
			yc = y - sin(a) * width / 2;

			xc = xc + r * cos(a - M_PI / 2);
			yc = yc + r * sin(a - M_PI / 2);  // calclulate rotate center

			dx = x - xc;
			dy = y - yc;
			//r = sqrt(dx*dx + dy*dy);           // calclulate rotate radius
			sh = -dis / r;

			x = cos(sh) * dx - sin(sh) * dy;
			y = sin(sh) * dx + cos(sh) * dy;
			x = x + xc;
			y = y + yc;                      // update car position
			a = a + sh;
			setAng(a * 360.0 / (2 * M_PI));   // update car body angle



		}
		else {
			A = M_PI / 2 + s;
			r = length / cos(A);
			xc = x - cos(a) * width / 2;
			yc = y - sin(a) * width / 2;

			xc = xc + r * cos(a + M_PI / 2);
			yc = yc + r * sin(a + M_PI / 2);

			dx = x - xc;
			dy = y - yc;
			//r = sqrt(dx * dx + dy * dy);
			sh = dis / r;

			x = cos(sh) * dx - sin(sh) * dy;
			y = sin(sh) * dx + cos(sh) * dy;
			x = x + xc;
			y = y + yc;
			a = a + sh;
			setAng(a * 360.0 / (2 * M_PI));
		}


	}

    void moveCenter(double dis) {

        double a = ang * 2 * M_PI / 360.0;
        double s = sita * 2 * M_PI / 360.0;

        double A;
        double sh;
        double dx;
        double dy;



        if (sita == 0.0) {
        	x = x + dis * cos(a);
        	y = y + dis * sin(a);
            r = -1;
            xc = -1;
            yc = -1;
        	return;
        }

        if (sita > 0) {

            A = M_PI / 2 - s;
            r = length / cos(A);
            xc = x - cos(a) * width / 2;
            yc = y - sin(a) * width / 2;

            xc = xc + r * cos(a - M_PI / 2);
            yc = yc + r * sin(a - M_PI / 2);  // calclulate rotate center

            dx = x - xc;
            dy = y - yc;
            r = sqrt(dx*dx+dy*dy);           // calclulate rotate radius
            sh = -dis / r;

            x = cos(sh) * dx - sin(sh) * dy;
            y = sin(sh) * dx + cos(sh) * dy;
            x = x + xc;
            y = y + yc;                      // update car position
            a = a + sh;
            setAng(a * 360.0 / (2 * M_PI));   // update car body angle



        }
        else {
            A = M_PI / 2 + s;
            r = length / cos(A);
            xc = x - cos(a) * width / 2;
            yc = y - sin(a) * width / 2;

            xc = xc + r * cos(a + M_PI / 2);
            yc = yc + r * sin(a + M_PI / 2);

            dx = x - xc;
            dy = y - yc;
            r = sqrt(dx * dx + dy * dy);
            sh = dis / r;

            x = cos(sh) * dx - sin(sh) * dy;
            y = sin(sh) * dx + cos(sh) * dy;
            x = x + xc;
            y = y + yc;
            a = a + sh;
            setAng(a * 360.0 / (2 * M_PI));
        }


    }
    void print() {
        cout << "car length=" <<this->length << endl;
        cout << "car width=" << width << endl;
        cout << "car pos x=( " << x << " , " << y << ")" << endl;
        cout << "car body angle =" << ang << endl;
        cout << "car wheel angle =" << sita << endl;
    }

    void log() {
        cout << x << "   \t" << y << "   \t" << ang << "   \t" << xc << "   \t" << yc << "   \t" << r << endl;
    }
};

#endif