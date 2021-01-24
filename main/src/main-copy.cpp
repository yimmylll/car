#include <ros/ros.h>
// #include <stdlib.h>
// #include <stdio.h>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include "CSMiddleWare/SPData.h"
#include "CSMiddleWare/TCPData.h"
#include "gpsreader/GPSData.h"
#include "std_msgs/Int8.h"

using namespace std;

int printControl = 0;

CSMiddleWare::SPData carDataMsg;
double lat = 0.0;
double lon = 0.0;
double gpsAngle = 0.0;
double backCompassAngle = 0.0;
double steerAngle = 0.0;
double frontCompassAngle = 0.0;
double carspeed = 0.0;

double obsNum = 0.0;

// 接收到订阅的消息后，会进入消息回调函数，类似中断服务函数，里面执行的内容不能太多
void gpsCallback(const gpsreader::GPSData::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    lat = msg->lat;
    lon = msg->lon;
    gpsAngle = msg->heading;

    // ROS_INFO("lat:%f  lon:%f  gpsAngle:%f", msg->lat, msg->lon, msg->heading);
    /*
    if(printControl%2 == 1)
    {
        cout.precision( numeric_limits<double>::digits10 + 1);
        cout << lat << " " << lon << " " << gpsAngle << endl;
    }
    */
}

// 接收到订阅的消息后，会进入消息回调函数，类似中断服务函数，里面执行的内容不能太多
void carInfoCallback(const CSMiddleWare::SPData::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    carspeed = (double)msg->SpeedVal;
    steerAngle = msg->AngleVal;
    // ROS_INFO("carspeed:%f  steerAngle:%f", carspeed, steerAngle);
}

// 接收到订阅的消息后，会进入消息回调函数，类似中断服务函数，里面执行的内容不能太多
void frontphoneInfoCallback(const CSMiddleWare::TCPData::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    frontCompassAngle = msg->CompassAngleVal;
    // ROS_INFO("frontCompassAngle:%f ", frontCompassAngle);
}

float obsNums[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
void obsInfoCallback(const std_msgs::Int8::ConstPtr& msg)
{
    obsNums[9] = obsNums[8];
    obsNums[8] = obsNums[7];
    obsNums[7] = obsNums[6];
    obsNums[6] = obsNums[5];
    obsNums[5] = obsNums[4];
    obsNums[4] = obsNums[3];
    obsNums[3] = obsNums[2];
    obsNums[2] = obsNums[1];
    obsNums[1] = obsNums[0];
    obsNums[0] = float(msg->data);
    obsNum = obsNums[0]*0.15 + obsNums[1]*0.15 + obsNums[2]*0.15 + obsNums[3]*0.1 + obsNums[4]*0.1 + obsNums[5]*0.1 + obsNums[6]*0.1 + obsNums[7]*0.05 + obsNums[8]*0.05 + obsNums[9]*0.05;
    // ROS_INFO("obs num :%d", obsNum);
}

void *callbackThread(void *ptr)
{
    ros::NodeHandle *pn = (ros::NodeHandle *)ptr;
    ros::Publisher carPub = (*pn).advertise<CSMiddleWare::SPData>("/SPWrite", 1000);
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
    }
    out.close();
}

double cfGpsLat()
{
    return lat;
}

double cfGpsLon()
{
    return lon;
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

double speed()
{
    return carspeed;
}

void steer(float v)
{
    carDataMsg.AngleVal = v;
}

void gas(float v)
{
    carDataMsg.ThrottleVal = v;
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

double latlonToMeter(double origin)
{
    double r = 6371 * 1000 /100 ;
    double pi = 3.14159265359;
    double rest;
    int cut;

    cut = origin / 100;
    cut = cut * 100;
    rest = origin - cut;
    rest=rest * 10.0 / 6.0;
    origin = cut + rest;

    return (r*origin * 2 * pi / 360.0);  //in m
}

double latToMeter(double origin, double target)
{
    return latlonToMeter(target) - latlonToMeter(origin);
}

double lonToMeter(double origin, double target)
{
    return -(latlonToMeter(target) - latlonToMeter(origin));
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
}

double bieyeObstacle(void)
{
    return obsNum;
}

void delay_1s(int v)
{
    ros::Rate gas_rate(1);
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

double moveTargetWithStop(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV, int brakeDealyTime,float & pGas,float angleDis)
{
	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	if(distance > minDistance) {
        if(bieyeObstacle()>4)
        {
            gas(0.0);
			pGas=0;
    		brake(brakeV);
    		delay_1s(brakeDealyTime);
    		brake(0.0);
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

int tofObstacle(){

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

double moveTargetWithTurn(double latOrigin, double lonOrigin, double latTarget, double lonTarget, double gpsCurrentAngle, float minDistance, float gasV, float brakeV, int brakeDealyTime,float& pGas,float angleDis)
{
	double coorSteerAngle, distance;
	coorSteerAngle = getSteerAngle(latOrigin, lonOrigin, latTarget, lonTarget, gpsCurrentAngle);
	distance = getDistance(latOrigin, lonOrigin, latTarget, lonTarget);
	if(distance > minDistance) {
		brake(0.0);
		gas(gasV);
        if(bieyeObstacle()>4||tofObstacle()<1200)
        {
			pGas=gasV/2;
            steer(35);
        }
        else
		{
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

double setTrapezoidalSize(double latOrigin, double lonOrigin, double latTarget, double lonTarget)
{
   return 0.00;
}



vector<double> getPath(int x)
{
    vector<double> a;
    a.push_back(3156.978007);
    a.push_back(11204.839157);
    a.push_back(3156.987938);
    a.push_back(11204.836187);
    if(x==0)
    {return a;}
    else
    {
        a.clear();
        return a;
    }
}

bool getStart(void)
{
    return true;
}

bool getStop(void)
{
    return false;
}

bool getTurn(void)
{
    return true;
}

void runTof(){

char cmd[100];
sprintf(cmd,"ps -ef | grep MultipleStreamRead | awk '{print $2}' | xargs kill -9");
system(cmd);	
sprintf(cmd,"./MultipleStreamRead &");
system(cmd);	
	
}

void killTof(){

char cmd[100];

sprintf(cmd,"ps -ef | grep MultipleStreamRead | awk '{print $2}' | xargs kill -9");
system(cmd);	


	
}


int main(int argc, char **argv)
{
	// parameters^
	float minDistance=1;	
	float brakeV=500;	
	float gasV=2.5;
	float gasRatio=0.1;  //speed changing rate
	float angleDis=3;  //distance for damping angle
    int brakeDealyTime =3;		
	// parametersV
	
    pthread_t id;
    // 初始化ROS节点
    ros::init(argc, argv, "main");

    // 创建节点句柄
    ros::NodeHandle n;
    ros::NodeHandle *pn = &n;

    // 创建一个Subscriber，订阅名为/gpsread的topic，订阅话题的队列长度10，注册回调函数gpsCallback
    ros::Subscriber gps_sub = n.subscribe("/gpsread", 10, gpsCallback);
    ros::Subscriber carInfo_sub = n.subscribe("/SPRead", 1000, carInfoCallback);
    ros::Subscriber frontphoneInfo_sub = n.subscribe("/TCPRead", 1000, frontphoneInfoCallback);
    ros::Subscriber obs_sub = n.subscribe("/obsNum", 10, obsInfoCallback);

    ros::Rate loop_rate(1);
    
    int ret = pthread_create(&id, NULL, callbackThread, (void*)pn);
    if(ret) {
        ROS_INFO("Create pthread error!");
        return 1;
    }

  vector<double> path;
  int pathSize=0;
  int start;
  int pstart=0;
  int stop;
  int pstop=0;
  int tu;
  int go=0;
        double ilat ;
        double ilon ;
        double tlat ;
        double tlon ;
        double igpsangle;
        float pGas=0;
  int targetIndex=0;	
  int patho = 0;
//     gas(0);
//   brake(0);
//   delay_1s(3);
//   return 0;

runTof();

//printf("run tof OK\n");

    while(ros::ok())
    {
		
		//printf("enter while\n");
	
         //printf("tofObstacle: %d\n",tofObstacle());
        // printf("bieyeobstacle: %lf\n",bieyeObstacle());
        //continue;
         ilat = cfGpsLat();
         ilon = cfGpsLon();
         igpsangle = cfGpsAngle();
        //double idistance getDistance(ilat, ilon,)
        // printf("lat:%f   lon:%f\n", ilat, ilon);
        // continue;
        //moveTarget(ilat, ilon, 3156.913886, 11204.859963, igpsangle, 0.5, 2.5, 500, 3);
		
		//检测云平台的路径信息
		
		if(targetIndex>=pathSize){
		path=getPath(patho);
        patho++;
		pathSize=path.size()/2;	
		targetIndex=0;
        printf("pathSize = %d\n",pathSize);	
		}
		//检测云平台的开始信号

		start=getStart();
		
		//检测云平台的停止信号
		stop=getStop();
		
		//检测云平台是否绕障碍
		tu=getTurn();
		
		//停止小车
		if(pstop==0&&stop==1){
		gas(0.0);
		pGas=0;
		brake(brakeV);	
		go=0;	
        printf("car stop\n");	
	
			continue;
		}		
		//启动小车
		if(pstart==0&&start==1){

			go=1;
            printf("car start\n");		
		}
		//已走完所有目标点
        // printf("targetIndex= %d\n",targetIndex);
        // printf("pathSize= %d\n",pathSize);
		if(targetIndex>=pathSize){
			
		gas(0.0);
		pGas=0;
		brake(brakeV);
        delay_1s(3);
         printf("car finish\n");
         brake(0);
        delay_1s(3);
        //return 0;
        break;
		continue;
		}
		
		if(go==0){
			
			printf("car go = %d\n",go);	
			continue;
		}
		
		//到达第targetIndex个目标点
		double dis=getDistance(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1]);
	    if(dis<minDistance){
			targetIndex++;	
            printf("reach target%d\n",targetIndex);	
            gas(0.0);
            pGas = 0;	
		}else{
		//走第targetIndex个目标点
		if(tu){
		moveTargetWithTurn(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1], igpsangle, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);			
			
		}else{
		moveTargetWithStop(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1], igpsangle, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);				
			
		}
        pGas=gasV*gasRatio+pGas*(1-gasRatio);
        // printf("dis = %f\n", dis);
        // printf("pGas = %f\n", pGas);
		}
	//	printf("dis = %f\n", dis);
		pstart=start;
		pstop=stop;
    }
killTof();
    pthread_exit(NULL);
    return 0;
}
