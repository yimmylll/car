#ifndef __MAIN_H__
#define __MAIN_H__

#include <ros/ros.h>
#include <vector>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include "CSMiddleWare/SPData.h"
#include "CSMiddleWare/TCPData.h"
#include "gpsreader/GPSData.h"
#include "std_msgs/Int8.h"
#include "cloudProtocol/cloudData.h"
#include "car.h"
#include "cloud.h"
#include "bieye.h"

using namespace std;

#define Ultra_THREAD6 50  //右前超声阈值
#define Ultra_THREAD3 50  //左前超声阈值
#define Bieye_THREAD 4
#define WrongCntTHREAD 18 //误判阈值
#define MIS_DIS 1.5

extern int printControl;

extern CSMiddleWare::SPData carDataMsg;
extern double lat;
extern double lon;
extern double gpsAngle;
extern int gpsQF;
extern double backCompassAngle;
extern double steerAngle;
extern double frontCompassAngle;
extern unsigned int carCounter;
extern unsigned int preCarCounter;
extern int block;
extern int CannotFindTarget;
static double G_distance = 0.0;
static double FirstWrongdistance = 0.0;
extern int FirstWrongCnt;

int carSpeed = 0;
double carUltrasonic1 = 0.0;
double carUltrasonic2 = 0.0;
double carUltrasonic3 = 0.0;
double carUltrasonic4 = 0.0;
double carUltrasonic5 = 0.0;
double carUltrasonic6 = 0.0;

extern double G_dx;    // 丢失GPS之前小车到下一个目标x方向的距离，对应经度
extern double G_dy;    // 丢失GPS之前小车到下一个目标y方向的距离，对应纬度
extern double G_angle; // 丢失GPS之前小车到下一个目标的角度
extern bool G_lostGps; // 是否丢失GPS
extern car G_car;      // 当前车辆
extern int G_carCounter;      // 上一次速度编码器计数	
extern float G_steer;  // 当前转向角

extern bool indoorStartUp;
extern int backfinish;//0 未走完 1走完一次倒车坐标，跳出函数刹车-释放
extern int frontfinish;
void delay_1ms(int v);

// 接收到订阅的消息后，会进入消息回调函数，类似中断服务函数，里面执行的内容不能太多
void gpsCallback(const gpsreader::GPSData::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    lat = msg->lat;
    lon = msg->lon;
    gpsQF = msg->QF;
    gpsAngle = msg->heading;

    // ROS_INFO("lat:%f  lon:%f  gpsAngle:%f  QF:%d", msg->lat, msg->lon, msg->heading, msg->QF);
    /*
    if(printControl%2 == 1)
    {
        cout.precision( numeric_limits<double>::digits10 + 1);
        cout << lat << " " << lon << " " << gpsAngle << endl;
    }
    */
}

void carInfoCallback(const CSMiddleWare::SPData::ConstPtr& msg)
{
    carCounter = msg->CounterVal;
    steerAngle = msg->AngleVal;
    carSpeed = msg->SpeedVal;
    carUltrasonic1 = msg->Ultrasonic1;
    carUltrasonic2 = msg->Ultrasonic2;
    carUltrasonic3 = msg->Ultrasonic3;
    carUltrasonic4 = msg->Ultrasonic4;
    carUltrasonic5 = msg->Ultrasonic5;
    carUltrasonic6 = msg->Ultrasonic6;
    // ROS_INFO("carCounter:%d steerAngle:%f", msg->SpeedVal, steerAngle);
}

void frontphoneInfoCallback(const CSMiddleWare::TCPData::ConstPtr& msg)
{
    frontCompassAngle = msg->CompassAngleVal;
    // ROS_INFO("frontCompassAngle:%f ", frontCompassAngle);
}

void backphoneInfoCallback(const CSMiddleWare::TCPData::ConstPtr& msg)
{
    backCompassAngle = msg->CompassAngleVal;
    // ROS_INFO("backCompassAngle:%f ", backCompassAngle);
}

void *serialCallbackThread(void *ptr)
{
    ros::NodeHandle *pn = (ros::NodeHandle *)ptr;
    ros::Publisher carPub = (*pn).advertise<CSMiddleWare::SPData>("/SPWrite", 1000);
    ros::Publisher cloudPub = (*pn).advertise<cloudProtocol::cloudData>("/cloudWrite", 1000);
    ros::Rate loop_rate(10);
    ofstream out;
    out.open("log.txt");
    while(ros::ok())
    {
        carPub.publish(carDataMsg);

        ros::spinOnce();
        loop_rate.sleep();

        if(printControl%2 == 1)
        {
            out.precision( numeric_limits<double>::digits10 + 1);
            out << lat << " " << lon << " " << gpsAngle << endl;
        }

        preCarCounter = carCounter;
    }
    out.close();
}



int getGpsStatus(int c = 0)
{
    /*
    // ^只用于惯导测试使用，输入1模拟真实GPS，输入0模拟丢失GPS
    //  真实使用时需注释掉
    if(getManualModel() == true) {
        G_lostGps = true;
        return 0;
    } else if(c == 1) {
        G_lostGps = false;
        return 4;
    } else {
        G_lostGps = true;
        return 0;
    }
    // v
    */
    if(getManualModel() >= 1) {
        G_lostGps = true;
        return 0;
    } else if(gpsQF == 4) {
        G_lostGps = false;
    } else {
        G_lostGps = true;
    }

    return gpsQF;
}

double cfGpsLat()
{
    return gpsLatLon2WGS84(lat);
}

double cfGpsLon()
{
    return gpsLatLon2WGS84(lon);
}

double cfGpsAngle()
{
    return gpsAngle;
}

double frontCompass()
{
    return frontCompassAngle;
}

double backCompass()
{
    return backCompassAngle;
}

double getSteerAngle()
{
    return steerAngle;
}

unsigned int getCarCounter()
{
    return carCounter;
}

unsigned int speed()
{
    if(carCounter > preCarCounter) {
        return carCounter - preCarCounter;
    } else {
        return carCounter + preCarCounter;
    }
}

double ultrasonic(int n)
{
    if(n == 1) {
        return carUltrasonic1;
    } else if(n == 2) {
        return carUltrasonic2;
    } else if(n == 3) {
        return carUltrasonic3;
    } else if(n == 4) {
        return carUltrasonic4;
    } else if(n == 5) {
        return carUltrasonic5;
    } else if(n == 6) {
        return carUltrasonic6;
    }
}

void steer(float v)
{
    double minAngle = 1.0;
    if(v - G_steer > minAngle) {
        carDataMsg.AngleVal = G_steer + minAngle;
        G_steer = G_steer + minAngle;
    } else if(G_steer - v > minAngle) {
        carDataMsg.AngleVal = G_steer - minAngle;
        G_steer = G_steer - minAngle;
    } else {
        carDataMsg.AngleVal = v;
        G_steer = v;
    }
}

bool isGasBack = false;
void gas(float v)
{   
    if(v < 0.0 && isGasBack == false) {
        isGasBack = true;
        carDataMsg.ThrottleVal = -0.01;
        delay_1ms(500);
    }
    if(v >= 0.0) {
        isGasBack = false;
    }
    carDataMsg.ThrottleVal = v;
    printf("set gas:%f\n", v);
}

void gasDelay()
{
    ros::Rate gas_rate(2);
    gas_rate.sleep();
}

void brake(float v)
{
    carDataMsg.BrakeVal = v;
}

double latToMeterSingel(double origin)
{
    double r = 6371 * 1000 /100 * 3688 / 3724.0;
    double pi = 3.14159265359;

    origin = origin*100;

    return (r*origin * 2 * pi / 360.0);  //in m
}

double lonToMeterSingel(double origin)
{
    double r = 6371 * 1000 /100 * 3133 / 3724.0;
    double pi = 3.14159265359;

    origin = origin*100;

    return (r*origin * 2 * pi / 360.0);  //in m
}

double latToMeter(double origin, double target)
{
    return latToMeterSingel(target) - latToMeterSingel(origin);
}

double lonToMeter(double origin, double target)
{
    return -(lonToMeterSingel(target) - lonToMeterSingel(origin));
}

vector<double> manualPathToMeter(double y, double x, double steer)
{
    double yMax = 5.0;
    double hight = 0.35;
    double fovH = 60.0, fovV = 60.0;
    double PI = 3.1415926;
    double r = 6371 * 1000 * 2 * PI / 360.0;
    vector<double> out;
    // 以米为单位的目标点
    double xM, yM;
    xM = (x-0.5)*fovH/2.0;
    yM = (y-0.5)*fovV/2.0;
    printf("xM:%f yM:%f\n",xM,yM);
    if(yM > 0) {
        yM = yMax;
    } else {
        yM = hight / tan(-yM*PI/180.0);
    }
    if(yM > yMax) {
        yM = yMax;
    }
    
    xM = tan(xM*PI/180.0)*sqrt(yM*yM + hight*hight);
    printf("xM:%f yM:%f\n",xM,yM);

    // 在小车坐标系中的目标点
    double xC, yC; 
    double c = cos(steer/180.0*PI);
    double s = sin(steer/180.0*PI);
    xC = c*xM - s*yM;
    yC = s*xM + c*yM;
    xC = xC/r;
    yC = -yC/r;

    printf("xC:%f yC:%f\n",xC,yC);

    out.push_back(xC);
    out.push_back(yC);

    return out;
}

double gpsAngleToCoordinate(double gpsAngle)
{
    double a = 360.0 - gpsAngle;
    
    if(a >= 0) {
        return a;
    } else {
        return a+360;
    }
}

double coorToCarHAngle(double v)
{
    double a = v + 90.0;
    if(a >= 360) {
        a -= 360;
    }

    return a;
}

double carHToCoorAngle(double v)
{
    double a = v - 90.0;
    if(a < 0) {
        a += 360;
    }

    return a;
}

double vectorAngle(double x, double y)
{
    double pi = 3.14159265359;
    double a = atan2(y, x);

    a = a * 180 / pi;
    if(a < 0) {
        a = a +360;
    }

    return a;
}

double angleToSteer(double gpsAngleNormal, double targetAngle)
{
    double a = targetAngle - gpsAngleNormal;
    
    if(a > 180) {
        return a - 360;
    } else if(a < -180) {
        return a + 360;
    } else {
        return a;
    }
}

double getSteerAngle(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle)
{
    double latMeter;
    double lonMeter;
    double coorCurrentAngle;
    double coorSteerAngle;
    double vectorSteerAngle;

    latMeter = latToMeter(latOrigin, latTarget);
    lonMeter = lonToMeter(lonOrigin, lonTarget);
    coorCurrentAngle = gpsAngleToCoordinate(gpsCurrentAngle);
    vectorSteerAngle = vectorAngle(latMeter, lonMeter);
    coorSteerAngle = angleToSteer(coorCurrentAngle, vectorSteerAngle);
    // printf("%f, %f ,%f, %f", latMeter, lonMeter,coorCurrentAngle, vectorSteerAngle);
    
    if(coorSteerAngle < -35.0) {
        coorSteerAngle = -35.0;
    } if(coorSteerAngle > 35.0) {
        coorSteerAngle = 35.0;
    }

    return coorSteerAngle;
}

double getDistance(double latOrigin, double lonOrigin, double latTarget, double lonTarget)
{
    double latMeter;
    double lonMeter;

    latMeter = latToMeter(latOrigin, latTarget);
    lonMeter = lonToMeter(lonOrigin, lonTarget);
    double distance = sqrt(pow(abs(latMeter), 2)+pow(abs(lonMeter), 2));

    return distance;
}


double getSteerAngleInMeter(car &mCar)
{
    double latMeter;
    double lonMeter;
    double coorCurrentAngle;
    double coorSteerAngle;
    double vectorSteerAngle;

    lonMeter = G_dy + mCar.x;
    latMeter = G_dx - mCar.y;
    coorCurrentAngle = carHToCoorAngle(mCar.ang);
    vectorSteerAngle = vectorAngle(latMeter, lonMeter);
    coorSteerAngle = angleToSteer(coorCurrentAngle, vectorSteerAngle);
    printf("lonMeter:%f, latMeter:%f ,coorCurrentAngle:%f, vectorSteerAngle:%f\n", \
            lonMeter, latMeter,coorCurrentAngle, vectorSteerAngle);
    
    if(coorSteerAngle < -35.0) {
        coorSteerAngle = -35.0;
    } if(coorSteerAngle > 35.0) {
        coorSteerAngle = 35.0;
    }

    return coorSteerAngle;
}

double getDistanceInMeter(car &mCar)
{
    double latMeter;
    double lonMeter;

    lonMeter = G_dy + mCar.x;
    latMeter = G_dx - mCar.y;
    double distance = sqrt(pow(abs(latMeter), 2)+pow(abs(lonMeter), 2));
   
    return distance;
}

void delay_1s(int v)
{
    ros::Rate gas_rate(1);
    while(v--)
        gas_rate.sleep();
}

void delay_1ms(int v)
{
    ros::Rate gas_rate(1000);
    while(v--)
        gas_rate.sleep();
}

double moveTarget(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV, int brakeDealyTime)
{
	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	if(distance > minDistance) {
		brake(0.0);
		gas(gasV);
		steer(coorSteerAngle);
	} else {
		gas(0.0);
		// brake(brakeV);
		// delay_1s(brakeDealyTime);
		// brake(0.0);
	}
}
//test
double moveTargetBack(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV)
{
	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	if(distance > minDistance) {
		brake(0.0);
		gas(gasV);
		steer(-coorSteerAngle);
	} else {
		gas(0.0);
		// brake(brakeV);
		// delay_1s(brakeDealyTime);
		// brake(0.0);
	}
}

double moveTargetWithStop(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV, int brakeDealyTime,float & pGas,float angleDis)
{
	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	if(distance > minDistance) {
		printf("bieyeObs:%f\n",bieyeObstacle());
        if(bieyeObstacle()> Bieye_THREAD)//一直刹车释放？
        {
            gas(0.0);
			pGas=0;
    		brake(brakeV);
    		/*delay_1s(brakeDealyTime);
    		brake(0.0);*/
        }
        else
        {
            brake(0.0);
    		gas(gasV);
			
			if(distance>angleDis){
            steer(coorSteerAngle);				
			}
			else{
            steer(coorSteerAngle*distance/angleDis);				
			}
        }
	} else {
		//gas(0.0);
		//brake(brakeV);
		//delay_1s(brakeDealyTime);
		//brake(0.0);
	}
}

int tofObstacle()
{
    int number;
    int mean;

    FILE*p=fopen("tofData","rb");
    if(p==NULL) return 3500;
    fread(&number,sizeof(int),1,p);
    fread(&mean,sizeof(int),1,p);
    fclose(p);

    if(number>3000) return mean; 
    return 4000;
}

// double moveTargetWithTurn(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV, int brakeDealyTime,float& pGas,float angleDis)
// {
	// double coorSteerAngle, distance;
	// coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	// distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	// if(distance > minDistance) {
		// brake(0.0);
		// gas(gasV);
		// // 增加同时超声在左侧没有障碍
		// if (bieyeObstacle() > 4)  //if (bieyeObstacle() > 4 || tofObstacle() < 1200)
		// {
			// // if 当超声波检测左侧也有障碍时停止 else 左拐
			// if(ultrasonic(2) < Ultra_THREAD3){
				// gas(0.0);
				// pGas = 0;
				// brake(brakeV);
				// delay_1s(brakeDealyTime);
				// brake(0.0);
			// }
			// else{
			// pGas = gasV / 2;
			// steer(35);
			// }
		// }
        // else
		// {
			// if(distance>angleDis){
            // steer(coorSteerAngle);				
			// }
			// else{
            // steer(coorSteerAngle*distance/angleDis);				
			// }
        // }
	// } else {
		// //gas(0.0);
		// //brake(brakeV);
		// //delay_1s(brakeDealyTime);
		// //brake(0.0);
	// }
// }
/***************************************新修改***********************************************************/
//2左前超声  1右前超声
double moveTargetWithTurn(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV, int brakeDealyTime, float& pGas, float angleDis)
{
	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	
	if (G_distance > 0.0 && distance > G_distance)//开始绕远了
	{
		if (FirstWrongCnt == 0) {
			FirstWrongdistance = G_distance;//记录下第一次变远前 离目标距离
		}
		
		FirstWrongCnt++;
	}
	//else if(FirstWrongCnt<3 && distance <= G_distance){//距离增加次数<3后又离目标越来越近认为误判，清零错误计算值
	//	FirstWrongCnt = 0;
	//}
	printf("FirstWrongCnt:%d\n", FirstWrongCnt);
	
	
	//if (FirstWrongdistance > 0.0 && distance - FirstWrongdistance >= MIS_DIS) //偏离大于1.5m
	if(FirstWrongCnt>=WrongCntTHREAD)
	{
		printf("自动 当前离目标点距离:%f   第一次离目标距离：%f  FirstWrongCnt++\n", distance, G_distance);
		printf("distance:%f   FirstWrongdistance:%f\n", distance, FirstWrongdistance);
		CannotFindTarget = 1;
		G_distance = 0;
		FirstWrongCnt = 0;
		//continue;
		return 0.0;
	}

	G_distance = distance;//记录上一个循环离目标点距离

	if (distance > minDistance) {
		if (block == 0) {
			brake(0.0);
			gas(gasV);
		}
		printf("bieye:%f  u6:%8.4f  u3:%8.4f\n", bieyeObstacle(), ultrasonic(6), ultrasonic(3));
		if (bieyeObstacle() > Bieye_THREAD)
		{
			if (ultrasonic(3) > Ultra_THREAD3)//前方有障碍，左前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn left\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(35);
				return 0.0;
			}
			else if(ultrasonic(6) > Ultra_THREAD6)//前方有障碍，右前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn right\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(-35);
				return 0.0;
			}
			else     //前方、左前、右前有障碍
			{
				if(block==0)
				{
					block++;
					cannotAvoidObstacles();//发送不能规避障碍
					printf("Cannot avoid obstacle! gas0 brake500 delay1s\n");
					gas(0.0);
					pGas = 0;
					brake(brakeV);
					delay_1s(1);
					/*gas(0.0);
					pGas = 0;
					brake(brakeV);
					delay_1s(brakeDealyTime);
					brake(0.0);*/
				}
				//报告不能规避障碍
				//printf("Cannot avoid obstacle!block=%d\n", block);
				return 0.0;
			}
		}
		else//前方无障碍
		{
			if (ultrasonic(6) < Ultra_THREAD6 && ultrasonic(3) > Ultra_THREAD3)//右前有障碍，左前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn left\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(35);
				return 0.0;
			}
			else if(ultrasonic(3) < Ultra_THREAD3 && ultrasonic(6) > Ultra_THREAD6)//左前有障碍，右前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn right\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(-35);
				return 0.0;
			}
			else if (ultrasonic(3) < Ultra_THREAD3 && ultrasonic(6) < Ultra_THREAD6)//左前、右前有障碍
			{
				if (block == 0)
				{
					block++;
					//报告不能规避障碍
					cannotAvoidObstacles();//发送不能规避障碍
					printf("Cannot avoid obstacle! gas0 brake500 delay1s\n");
					gas(0.0);
					pGas = 0;
					brake(brakeV);
					delay_1s(1);
				}
				//报告不能规避障碍
				//printf("Cannot avoid obstacle!block=%d\n", block);
				return 0.0;
			}
			else
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, move towards target\n");
					block = 0;
				}
				if (distance > angleDis) {
					steer(coorSteerAngle);
				}
				else {
					steer(coorSteerAngle*distance / angleDis);
				}
			}
		}
	}
	else
	{
		FirstWrongCnt = 0;//清零误判值
	}
}

double moveTargetWithTurnInMeter(car &mCar, float minDistance, float gasV, \
	float brakeV, int brakeDealyTime, float & pGas, float angleDis)
{
	unsigned int mCarCounter = getCarCounter();
	mCar.sita = -G_steer;
	// 轮子转1圈对应编码器计数4096点
	mCar.move((mCarCounter - G_carCounter)*1.09955741 / 4096.0);
	G_carCounter = mCarCounter;

	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngleInMeter(mCar);
	distance = getDistanceInMeter(mCar);
	printf("coorSteerAngle:%f, distance:%f\n", coorSteerAngle, distance);
	
	if (G_distance > 0.0 && distance > G_distance)//开始绕远了
	{
		if (FirstWrongCnt == 0) {
			FirstWrongdistance = G_distance;//记录下第一次变远前 离目标距离
		}
		//printf("手动 当前离目标点距离:%f   上次离目标距离：%f  FirstWrongCnt++\n", distance, G_distance);
		FirstWrongCnt++;
	}
	//else if (FirstWrongCnt < 3 && distance <= G_distance) {//距离增加次数<3后又离目标越来越近认为误判，清零错误计算值
	//	FirstWrongCnt = 0;
	//}
	printf("FirstWrongCnt:%d\n", FirstWrongCnt);
	//if (distance - FirstWrongdistance >= 2) //偏离大于2m
	if (FirstWrongCnt >= WrongCntTHREAD)
	{
		printf("distance:%f   FirstWrongdistance:%f\n", distance, FirstWrongdistance);
		CannotFindTarget = 1;
		G_distance = 0;
		FirstWrongCnt = 0;
		//continue;
		return 0.0;
	}

	G_distance = distance;//记录上一个循环离目标点距离
	
	if (distance > minDistance) {
		if (block == 0) {
			brake(0.0);
			gas(gasV);
		}
        printf("bieye:%f  u6:%8.4f  u3:%8.4f\n",bieyeObstacle(),ultrasonic(6),ultrasonic(3));
		if (bieyeObstacle() > Bieye_THREAD)
		{
            
			if (ultrasonic(3) > Ultra_THREAD3)//前方有障碍，左前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn left\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(35);
				return 0.0;
			}
			else if (ultrasonic(6) > Ultra_THREAD6)//前方有障碍，右前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn right\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(-35);
				return 0.0;
			}
			else     //前方、左前、右前有障碍
			{
				if (block == 0)
				{
					block++;
					//报告不能规避障碍
					cannotAvoidObstacles();//发送不能规避障碍
					printf("Cannot avoid obstacle! gas0 brake500 delay1s\n");
					gas(0.0);
					pGas = 0;
					brake(brakeV);
					delay_1s(1);
					
				}
				//报告不能规避障碍
				//printf("Cannot avoid obstacle!block=%d\n", block);
				return 0.0;
			}
		}
		else//前方无障碍
		{
			if (ultrasonic(6) < Ultra_THREAD6 && ultrasonic(3) > Ultra_THREAD3)//右前有障碍，左前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn left\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(35);
				return 0.0;
			}
			else if (ultrasonic(3) < Ultra_THREAD3 && ultrasonic(6) > Ultra_THREAD6)//左前有障碍，右前无障碍
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, turn right\n");
					block = 0;
				}
				pGas = gasV / 2;
				steer(-35);
				return 0.0;
			}
			else if (ultrasonic(3) < Ultra_THREAD3 && ultrasonic(6) < Ultra_THREAD6)//左前、右前有障碍
			{
				if (block == 0)
				{
					block++;
					//报告不能规避障碍
					cannotAvoidObstacles();//发送不能规避障碍
					printf("Cannot avoid obstacle! gas0 brake500 delay1s\n");
					gas(0.0);
					pGas = 0;
					brake(brakeV);
					delay_1s(1);
				}
				//报告不能规避障碍
				//printf("Cannot avoid obstacle!block=%d\n",block);
				return 0.0;
			}
			else
			{
				if (block == 1) {
					brake(0.0);
					printf("obstacle disapear, move towards target\n");
					block = 0;
				}
				if (distance > angleDis) {
					steer(coorSteerAngle);
				}
				else {
					steer(coorSteerAngle*distance / angleDis);
				}
			}
		}
	}
	else
	{
		FirstWrongCnt = 0;//清零误判值
		gas(0.0);//手动前进到达目标不刹车，只松油门
	}
}
/*******************************************************************************************************************/

double moveTargetWithStopInMeter(car &mCar, float minDistance, float gasV, \
                                float brakeV, int brakeDealyTime,float & pGas,float angleDis)
{
    int mCarCounter = getCarCounter();
    mCar.sita = -G_steer;
    // 轮子转1圈对应编码器计数4096点
    if(G_carCounter - mCarCounter > 5000) {
        mCar.move((65536 + mCarCounter-G_carCounter)*1.09955741/4096.0);
    } else {
        mCar.move((mCarCounter-G_carCounter)*1.09955741/4096.0);
    }
    printf("move dis:%f\n", (mCarCounter-G_carCounter)*1.09955741/4096.0);
	printf("mCarCounter:%d, G_carCounter:%d\n", mCarCounter, G_carCounter);
    G_carCounter = mCarCounter;

	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngleInMeter(mCar);
	distance = getDistanceInMeter(mCar);
    printf("coorAngle:%f, distance:%f, x:%lf, y:%lf\n", coorSteerAngle, distance, mCar.x, mCar.y);
    if(distance > minDistance) {
        printf("bieyeObs:%f\n",bieyeObstacle());
        if(bieyeObstacle()> Bieye_THREAD)
        {
            gas(0.0);
			pGas=0;
    		brake(brakeV);
    		/*delay_1s(brakeDealyTime);
    		brake(0.0);*/
        }
        else
        {
            brake(0.0);
    		gas(gasV);
            printf("gasV:%f\n",gasV);
			if(distance>angleDis){
                steer(coorSteerAngle);				
			}
			else{
                steer(coorSteerAngle*distance/angleDis);				
			}
        }
	} else {
		gas(0.0);//手动前进到达目标不刹车，只松油门
	}
}


void moveBack(float steerV, float gasV, float moveDistance, int startCnt)
{
    static bool isOutCnt = false;//?
	double coorSteerAngle, distance;
    int mCarCounter = getCarCounter();
    // 轮子转1圈对应编码器计数4096点
    if(abs(G_carCounter - mCarCounter) > 5000) {
        isOutCnt = true;
    } 
    if(isOutCnt) {
        distance = (startCnt - mCarCounter + 65536)*1.09955741/4096.0;
    } else {
        distance = (startCnt - mCarCounter)*1.09955741/4096.0;
    }
    printf("startCnt:%d G_carCounter:%d  mCarCounter:%d\n", startCnt, G_carCounter, mCarCounter);
    printf("moveDistance:%f\n", distance);
    G_carCounter = mCarCounter;
    if(abs(distance) < moveDistance) {
        gas(gasV);
        steer(steerV);				
        printf("gasV:%f steer:%f \n",gasV, steerV);
	} else {
        isOutCnt = false;
		backfinish = 1;
	}
	printf("backfinish:%d\n", backfinish);
}

void moveForward(float steerV, float gasV, float moveDistance, int startCntF)
{
	static bool isOutCntF = false;//?
	double coorSteerAngle, distance;
	int mCarCounter = getCarCounter();
	// 轮子转1圈对应编码器计数4096点
	if (abs(G_carCounter - mCarCounter) > 5000) {
		isOutCntF = true;
	}
	if (isOutCntF) {
		distance = (startCntF - mCarCounter + 65536)*1.09955741 / 4096.0;
	}
	else {
		distance = (startCntF - mCarCounter)*1.09955741 / 4096.0;
	}
	printf("startCntF:%d G_carCounter:%d  mCarCounter:%d\n", startCntF, G_carCounter, mCarCounter);
	printf("moveDistance:%f\n", distance);
	G_carCounter = mCarCounter;
	if (abs(distance) < moveDistance) {
		gas(gasV);
		steer(steerV);
		printf("gasV:%f steer:%f \n", gasV, steerV);
	}
	else {
		isOutCntF = false;
		frontfinish = 1;
	}
	printf("frontfinish:%d\n", frontfinish);
}

// double moveTargetWithTurnInMeter(car &mCar, float minDistance, float gasV, \
                                // float brakeV, int brakeDealyTime,float & pGas,float angleDis)
// {
    // unsigned int mCarCounter = getCarCounter();
    // mCar.sita = -G_steer;
    // // 轮子转1圈对应编码器计数4096点
    // mCar.move((mCarCounter-G_carCounter)*1.09955741/4096.0);
    // G_carCounter = mCarCounter;

	// double coorSteerAngle, distance;
	// coorSteerAngle = getSteerAngleInMeter(mCar);
	// distance = getDistanceInMeter(mCar);
    // printf("coorSteerAngle:%f, distance:%f\n", coorSteerAngle, distance);
	// if(distance > minDistance) {
		// brake(0.0);
		// gas(gasV);
		// if (bieyeObstacle() > 4)  //if(bieyeObstacle()>4 || tofObstacle() < 1200)
		// {
			// // if 当超声波检测左侧也有障碍时停止 else 左拐
			// if(ultrasonic(2) < Ultra_THREAD3){
				// gas(0.0);
				// pGas = 0;
				// brake(brakeV);
				// delay_1s(brakeDealyTime);
				// brake(0.0);
			// }
			// else{
			// pGas = gasV / 2;
			// steer(35);
			// }
		// }
        // else
		// {
			// if(distance>angleDis){
            // steer(coorSteerAngle);				
			// }
			// else{
            // steer(coorSteerAngle*distance/angleDis);				
			// }
        // }
	// } else {
		// //gas(0.0);
		// //brake(brakeV);
		// //delay_1s(brakeDealyTime);
		// //brake(0.0);
	// }
// }

double setTrapezoidalSize(double latOrigin, double lonOrigin, double latTarget, double lonTarget)
{
   return 0.00;
}
/*
bool logInCloud(int ID)
{
    cloudDataMsgTest.ID = ID;
    cloudPubFlag = 1;
    delay_1s(3);
    return cloudDataMsgTest.logIn;
}

//3157.261721 11205.270968
//3157.262664 11205.266910
vector<double> getPath(int x = 0)
{   
    // 该变量在云平台更新完路径后置为1，读取完路径后置为0
    cloudPathValid = 0;

    // ^ 这段代码主要用于测试本地下发路径
    //   真实测试需要注释
    vector<double> simPath;
    //   设置惯导模拟GPS经纬度
    simPath.push_back(gpsLatLon2WGS84(3157.261721));
    simPath.push_back(gpsLatLon2WGS84(11205.270968));
    simPath.push_back(gpsLatLon2WGS84(3157.262664));
    simPath.push_back(gpsLatLon2WGS84(11205.266910));
    if(x==0) {
        return simPath;
    } else {
        simPath.clear();
        return simPath;
    }
    // v

    return cloudPath;
}

bool getStart(void)
{
    return cloudDataMsgTest.isAutoDrive;
}

bool getStop(void)
{
    return !cloudDataMsgTest.isAutoDrive;
}

bool getTurn(void)
{
    return cloudDataMsgTest.isAvoidObstacles;
}

bool getManualModel()
{
    return cloudDataMsgTest.isManualPath;
}

void reachTarget(void)
{
    cloudPubFlag = 1;
    cloudDataMsgTest.isArrive = true;
    cloudDataMsgTest.isCanAvoidObstacles = true;
    cloudDataMsgTest.currentLat = 0.0;
    cloudDataMsgTest.currentLon = 0.0;
}

void cannotAvoidObstacles(void) 
{
    cloudPubFlag = 1;
    cloudDataMsgTest.isArrive = false;
    cloudDataMsgTest.isCanAvoidObstacles = false;
    cloudDataMsgTest.currentLat = 0.0;
    cloudDataMsgTest.currentLon = 0.0;
}

void reportCurrentLatLon(double lat, double lon)
{
    cloudPubFlag = 1;
    cloudDataMsgTest.isArrive = false;
    cloudDataMsgTest.isCanAvoidObstacles = true;
    cloudDataMsgTest.currentLat = lat;
    cloudDataMsgTest.currentLon = lon;
}
*/

void runTof()
{
    char cmd[100];
    sprintf(cmd,"ps -ef | grep MultipleStreamRead | awk '{print $2}' | xargs kill -9");
    system(cmd);	
    sprintf(cmd,"./MultipleStreamRead &");
    system(cmd);	
}

void killTof()
{
    char cmd[100];

    sprintf(cmd,"ps -ef | grep MultipleStreamRead | awk '{print $2}' | xargs kill -9");
    system(cmd);	
}

void shutdown()
{
	char cmd[100];
	newTX2Shutdown();
	delay_1s(1);
	sprintf(cmd, "sudo shutdown now");
	system(cmd);
}

#endif