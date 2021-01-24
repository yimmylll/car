#ifndef __CLOUD_H__
#define __CLOUD_H__

#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h> 
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fcntl.h> 
#include <math.h>
#include "std_msgs/String.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define TCPBUFFERLENGTH 10000000

class tcpBuffer
{
public:
    char *data;
    int start;
    int end;
    // bool push(char *buffer, int length);    // 满了返回0， 不满返回1
    // bool pop(char *buffer, int length);     // 成功出栈返回1， 不成功返回0
    // bool check(char *buffer, int length);   // 检查收到的数据，不移动任何指针

    tcpBuffer()
    {
        data = new char[TCPBUFFERLENGTH];
        start = 0;
        end = 0;
    }

    ~tcpBuffer()
    {
        delete[]data;
    }

    bool push(char *buffer, int length)
    {
        int x;
        for(int i = 0; i < length; i++) {
            x = end + i;

            data[x%TCPBUFFERLENGTH] = buffer[i];
        
            /*
            if((x+1)%TCPBUFFERLENGTH == start) {  // 写满了
                return 0;
            }
            */
        }
        end += length;
        end = end%TCPBUFFERLENGTH;

        return 1;
    }

    bool pop(char *buffer, int length)
    {
        int x;
        for(int i = 0; i < length; i++) {
            x = start + i;

            buffer[i] = data[x%TCPBUFFERLENGTH];

            /*
            if(x%TCPBUFFERLENGTH == end) {  // 读空了
                return 0;
            }
            */
        }
        start += length;
        start = start % TCPBUFFERLENGTH;

        return 1;
    }

    bool check(char *buffer, int length)
    {
        int x;
        for(int i = 0; i < length; i++) {
            x = start + i;

            buffer[i] = data[x%TCPBUFFERLENGTH];

            /*
            if(x%TCPBUFFERLENGTH == end) {  // 读空了
                return 0;
            }
            */
        }

        return 1;
    }
};

double gpsLatLon2WGS84(double origin)
{
    double rest;
    int cut;

    cut = origin / 100;
    rest = origin - cut*100;
    rest=rest / 60.0;
    origin = cut + rest;

    return origin;
}


struct cloudData
{
    int ID;
    bool logIn;
    bool isSoftReboot;
    bool isAutoDrive;
    bool isAvoidObstacles;
    bool isControl;
    bool isRoutecomplete;
    bool isManualPathF;
    bool isManualPathB;
    bool isShutdown;
    float cloudGas;
    float cloudSteerAngle;
    float cloudLat;
    float cloudLon;

    bool isCanAvoidObstacles;
    bool isArrive;
    float currentLat;
    float currentLon;
};

cloudData cloudDataMsg;
char DataToSend[248];
char PrintOut[512];
bool isPubFlag = false;
int isSubdataFlag = 0;
int realID = 0;
extern int cloudPathValid;
int sendFlag = 0;

vector<double> cloudPathBuff;
vector<double> cloudPath;

void Array2Hex(const char *buf, int len, char *out)
{
    #define int2ascii(c) (c)>9?((c)+0x37):((c)+0x30);
    char temp;
    int i = 0;
    for(i = 0; i < len; i++) {
        temp = buf[i]&0xf0;
	out[3*i+0] = int2ascii(temp>>4);
	temp = buf[i]&0x0f;
	out[3*i+1] = int2ascii(temp);
	out[3*i+2] = 0x20;
    }
    out[3*i] = 0;
}

char softRebootFlag[16] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07};
char successAnswerFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
char errorAnswerFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09};
char routeDownFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B};
char routecompleteFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C};
char startAutoDriveFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D};
char stopAutoDriveFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E};
char avoidObstaclesFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
char notAvoidObstaclesFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12};
char openManualPathFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13};
char closeManualPathFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14};
char openManualPathBFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15};
char closeManualPathBFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16};
char shutdownFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17};
void DataAnalysis(const char *RecBuffer)
{
    char softRebootN = 0;
    char successAnswerN = 0;
    char errorAnswerN = 0;
    char routeDownN = 0;
    char routecompleteN = 0;
    char startAutoDriveN = 0;
    char stopAutoDriveN = 0;
    char avoidObstaclesN = 0;
    char notAvoidObstaclesN = 0;
    char openManualPathN = 0;
    char closeManualPathN = 0;
    char openManualPathBN = 0;
    char closeManualPathBN = 0;
    bool isSoftReboot = false;
    bool isSuccessAnswer = false;   //
    bool isErrorAnswer = false;     //
    bool isRouteDown = false;
    bool isRoutecomplete = false;
    bool isStartAutoDrive = false;
    bool isStopAutoDrive = false;
    bool isAvoidObstacles = false;
    bool isnotAvoidObstacles = false;
    bool isManualPathF = false;
    bool isNotManualPath = false;
    bool isManualPathB = false;
    bool isNotManualPathB = false;
    bool isShutdown = false;

    if(*(RecBuffer + 81) == softRebootFlag[15]) {
        isSoftReboot = 1;
    } else if(*(RecBuffer + 81) == successAnswerFlag[15]) {
        isSuccessAnswer = 1;
    } else if(*(RecBuffer + 81) == errorAnswerFlag[15]) {
        isErrorAnswer = 1;
    } else if(*(RecBuffer + 81) == routeDownFlag[15]) {
        isRouteDown = 1;
    } else if(*(RecBuffer + 81) == routecompleteFlag[15]) {
        isRoutecomplete = 1;
    } else if(*(RecBuffer + 81) == startAutoDriveFlag[15]) {
        isStartAutoDrive = 1;
    } else if(*(RecBuffer + 81) == stopAutoDriveFlag[15]) {
        isStopAutoDrive = 1;
    } else if(*(RecBuffer + 81) == avoidObstaclesFlag[15]) {
        isAvoidObstacles = 1;
    } else if(*(RecBuffer + 81) == notAvoidObstaclesFlag[15]) {
        isnotAvoidObstacles = 1;
    } else if(*(RecBuffer + 81) == openManualPathFlag[15]) {
        isManualPathF = 1;
    } else if(*(RecBuffer + 81) == closeManualPathFlag[15]) {
        isNotManualPath = 1;
    } else if(*(RecBuffer + 81) == openManualPathBFlag[15]) {
        isManualPathB = 1;
    } else if(*(RecBuffer + 81) == closeManualPathBFlag[15]) {
        isNotManualPathB = 1;
    } else if(*(RecBuffer + 81) == shutdownFlag[15]) {
        isShutdown = 1;
        cloudDataMsg.isShutdown = true;
    } 
    /*
    for(int i = 66; i < 82; i++) {
        if(*(RecBuffer + i) == softRebootFlag[i-66]) {
            softRebootN++;
        }
        if(*(RecBuffer + i) == successAnswerFlag[i-66]) {
            successAnswerN++;
        }
        if(*(RecBuffer + i) == errorAnswerFlag[i-66]) {
            errorAnswerN++;
        }
        if(*(RecBuffer + i) == routeDownFlag[i-66]) {
            routeDownN++;
        }
        if(*(RecBuffer + i) == routecompleteFlag[i-66]) {
            routecompleteN++;
        }
        if(*(RecBuffer + i) == startAutoDriveFlag[i-66]) {
            startAutoDriveN++;
        }
        if(*(RecBuffer + i) == stopAutoDriveFlag[i-66]) {
            stopAutoDriveN++;
        }
        if(*(RecBuffer + i) == avoidObstaclesFlag[i-66]) {
            avoidObstaclesN++;
        }
        if(*(RecBuffer + i) == notAvoidObstaclesFlag[i-66]) {
            notAvoidObstaclesN++;
        }
        if(*(RecBuffer + i) == openManualPathFlag[i-66]) {
            openManualPathN++;
        }
        if(*(RecBuffer + i) == closeManualPathFlag[i-66]) {
            closeManualPathN++;
        }
        if(*(RecBuffer + i) == openManualPathBFlag[i-66]) {
            openManualPathBN++;
        }
        if(*(RecBuffer + i) == closeManualPathBFlag[i-66]) {
            closeManualPathBN++;
        }

        if(softRebootN == 16) {
            isSoftReboot = 1;
        } else if(successAnswerN == 16) {
            isSuccessAnswer = 1;
        } else if(errorAnswerN == 16) {
            isErrorAnswer = 1;
        } else if(routeDownN == 16) {
            isRouteDown = 1;
        } else if(routecompleteN == 16) {
            isRoutecomplete = 1;
        } else if(startAutoDriveN == 16) {
            isStartAutoDrive = 1;
        } else if(stopAutoDriveN == 16) {
            isStopAutoDrive = 1;
        } else if(avoidObstaclesN == 16) {
            isAvoidObstacles = 1;
        } else if(notAvoidObstaclesN == 16) {
            isnotAvoidObstacles = 1;
        } else if(openManualPathN == 16) {
            isManualPathF = 1;
        } else if(closeManualPathN == 16) {
            isNotManualPath = 1;
        } else if(openManualPathBN == 16) {
            isManualPathB = 1;
        } else if(closeManualPathBN == 16) {
            isNotManualPathB = 1;
        } 
    }
    */
    if(isSoftReboot) {
        cloudDataMsg.isSoftReboot = true;
    } else {
        cloudDataMsg.isSoftReboot = false;
    }
    if(isRouteDown) {   // 解析经纬度 
        int pointPosition = 0;
        double lat=0.0, lon=0.0;
        // 维度解析
        for(int i=146; i < 162; i++) {
            if(*(RecBuffer + i) == 0x2e) {
                pointPosition = i;
            }
        }
        for(int i=(pointPosition+1); i < 162; i++) {
            lon += pow(0.1,i-pointPosition)*(*(RecBuffer + i) - 0x30);
        }
        for(int i=(pointPosition-1); i > 145; i--) {
            lon += pow(10,pointPosition-i-1)*(*(RecBuffer + i) - 0x30);
        }
        // 经度解析
        for(int i=162; i < 178; i++) {
            if(*(RecBuffer + i) == 0x2e) {
                pointPosition = i;
            }
        }
        for(int i=(pointPosition+1); i < 178; i++) {
            lat += pow(0.1,i-pointPosition)*(*(RecBuffer + i) - 0x30);
        }
        for(int i=(pointPosition-1); i > 161; i--) {
            lat += pow(10,pointPosition-i-1)*(*(RecBuffer + i) - 0x30);
        }
        // printf("接收：经纬度路径: lat:%f lon:%f\n", lat, lon);
        cloudPathBuff.push_back(lat);
        cloudPathBuff.push_back(lon);

        return;
    }
    if(isRoutecomplete) {
        printf("接收：%d 路径规划完毕\n", int(cloudPathBuff.size()/2));
        cloudDataMsg.isRoutecomplete = true;
        cloudPath.swap(cloudPathBuff);
        vector<double>().swap(cloudPathBuff);
        cloudPathValid = 1;
    } else {
        cloudDataMsg.isRoutecomplete = false;
    }
    if(isStartAutoDrive) {
        printf("接收：开始自动驾驶\n");
        cloudDataMsg.isAutoDrive = true;
    }
    if(isStopAutoDrive) {
        printf("接收：停止自动驾驶\n");
        cloudDataMsg.isAutoDrive = false;
        // cloudDataMsg.cloudGas = (float)((*(RecBuffer+86)<<24)|(*(RecBuffer+87)<<16)|(*(RecBuffer+88)<<8)|(*(RecBuffer+89)));
        // printf("cloudGas = %f\n", cloudDataMsg.cloudGas);
    }
    if(isAvoidObstacles) {
        printf("接收：绕开障碍物\n");
        cloudDataMsg.isAvoidObstacles = true;
    }
    if(isnotAvoidObstacles) {
        printf("接收：不绕开障碍物\n");
        cloudDataMsg.isAvoidObstacles = false;
    }
    if(isManualPathF) {
        printf("接收：进入手动指定路径前进模式\n");
        cloudDataMsg.isManualPathF = true;
    }
    if(isNotManualPath) {
        printf("接收：退出手动指定路径前进模式\n");
        cloudDataMsg.isManualPathF = false;
    }
    if(isManualPathB) {
        printf("接收：进入手动指定路径后退模式\n");
        cloudDataMsg.isManualPathB = true;
    }
    if(isNotManualPathB) {
        printf("接收：退出手动指定路径后退模式\n");
        cloudDataMsg.isManualPathB = false;
    }
    
    isPubFlag = true;
}

// 对数据格式验证
void DataReceivePrepar(const char * RecData)
{
    static int times = 0;
    char checkout;
    if(*RecData == 0x23 && *(RecData + 1) == 0x23 && *(RecData + 246) == 0x24 && *(RecData + 247) == 0x24){
        checkout = *RecData ^ *(RecData + 1);
        for(int i = 2; i < 242; i++) {
            checkout = checkout^*(RecData + i);
            times++;
        }
        // printf("checkout = %x\r\n", checkout);
        // printf("checkout = %x\r\n", *(RecData+245));
        if(*(RecData+245) == checkout) {
            DataAnalysis(RecData);
        }
    }
}

void PackSendData(int devType, 
                  int carId, 
                  int devId, 
                  int devLocation, 
                  int cMD,
                  // float gas, float angle, 
                  int clientGetDevType, 
                  int targetDevType,
                  int targetId, 
                  double lat, 
                  double lon,
                  int None5 = 0, 
                  int None6 = 0, 
                  int None7 = 0,
                  int None8 = 0)
{
    int i = 0;
    char str[32];
    int chectout[4];
    for(i = 0; i < 256; i++) {
        DataToSend[i] = 0x00;
    }
    DataToSend[0] = 0x23;
    DataToSend[1] = 0x23;

    // 发起设备类型 2-17
    // 1 -> 云平台; 2 -> 手机; 3 -> TX2; 4 -> 客户端
    
    DataToSend[14] = BYTE3(devType);
    DataToSend[15] = BYTE2(devType);
    DataToSend[16] = BYTE1(devType);
    DataToSend[17] = BYTE0(devType);
    
    //sprintf((DataToSend + 2), "%016d", devType);
    // 整车标识码 18-33
    // 未知
    /*
    DataToSend[30] = BYTE3(carId);
    DataToSend[31] = BYTE2(carId);
    DataToSend[32] = BYTE1(carId);
    DataToSend[33] = BYTE0(carId);
    */
    sprintf((DataToSend + 18), "%016d", carId);
    // 发起设备标识码 34-49
    // 未知
    /*
    DataToSend[46] = BYTE3(devId);
    DataToSend[47] = BYTE2(devId);
    DataToSend[48] = BYTE1(devId);
    DataToSend[49] = BYTE0(devId);
    */
    sprintf((DataToSend + 34), "%016d", devId);
    // 发起设备位置 50-65
    DataToSend[62] = BYTE3(devLocation);
    DataToSend[63] = BYTE2(devLocation);
    DataToSend[64] = BYTE1(devLocation);
    DataToSend[65] = BYTE0(devLocation);
    // 命令单元 66-81
    // 1 -> 登入(普通命令)
    // 2 -> 登出(普通命令)
    // 3 -> 心跳命令(一级命令)
    // 4 -> 应答标志预留-接受成功(普通命令)
    // 5 -> 应答标志预留-接受错误(普通命令)
    // 6 -> 上报小车当前位置(普通命令)
    // 7 -> 到达目的地(普通命令)
    // 8 -> 无法绕开障碍物(普通命令)
    if(cMD == 1) {
        DataToSend[81] = 0x01;
    } else if(cMD == 2) {
        DataToSend[81] = 0x02;
    } else if(cMD == 3) {
        DataToSend[66] = 0x01;
        DataToSend[81] = 0x03;
    } else if(cMD == 4) {
        DataToSend[81] = 0x08;
    } else if(cMD == 5) {
        DataToSend[81] = 0x09;
    } else if(cMD == 6) {
        DataToSend[81] = 0x0A;
    } else if(cMD == 7) {
        DataToSend[81] = 0x0F;
    } else if(cMD == 8) {
        DataToSend[81] = 0x11;
    } else if(cMD == 9) {   // 下发经纬度
        DataToSend[81] = 0x0B;
    } else if(cMD == 10) {  // 停止自动驾驶
        DataToSend[81] = 0x0E;
    } else if(cMD == 11) {  // 停止自动驾驶
        DataToSend[81] = 0x0C;
    }
    // 远程控制字段 82-97 可不用
    // V A
    // $DV=xxxx,A=xxxx#
    /*
    DataToSend[82] = '$';
    DataToSend[83] = 'D';
    DataToSend[84] = 'V';
    DataToSend[85] = '=';
    DataToSend[86] = BYTE3(gas);
    DataToSend[87] = BYTE2(gas);
    DataToSend[88] = BYTE1(gas);
    DataToSend[89] = BYTE0(gas);
    DataToSend[90] = ',';
    DataToSend[91] = 'A';
    DataToSend[92] = '=';
    DataToSend[93] = BYTE3(angle);
    DataToSend[94] = BYTE2(angle);
    DataToSend[95] = BYTE1(angle);
    DataToSend[96] = BYTE0(angle);
    DataToSend[97] = '#';
    */
    // 客户端获取设备类型 98-113
    // 1 -> 云平台; 2 -> 手机; 3 -> TX2; 4 -> 客户端
    DataToSend[110] = BYTE3(clientGetDevType);
    DataToSend[111] = BYTE2(clientGetDevType);
    DataToSend[112] = BYTE1(clientGetDevType);
    DataToSend[113] = BYTE0(clientGetDevType);
    // 目的设备类型 114-129
    // 1 -> 云平台; 2 -> 手机; 3 -> TX2; 4 -> 客户端
    DataToSend[126] = BYTE3(targetDevType);
    DataToSend[127] = BYTE2(targetDevType);
    DataToSend[128] = BYTE1(targetDevType);
    DataToSend[129] = BYTE0(targetDevType);
    // 目的设备标识码 130-145
    DataToSend[142] = BYTE3(targetId);
    DataToSend[143] = BYTE2(targetId);
    DataToSend[144] = BYTE1(targetId);
    DataToSend[145] = BYTE0(targetId);
    // 路径信息经度 146-161
    sprintf((DataToSend + 146), "%016.7lf", lon);
    // 路径信息纬度 162-177
    sprintf((DataToSend + 162), "%016.7lf", lat);

    DataToSend[245] = DataToSend[0]^DataToSend[1];
    for(i = 2; i < 242; i++) {
        DataToSend[245] = DataToSend[245]^DataToSend[i];
    }

    DataToSend[246] = 0x24;
    DataToSend[247] = 0x24;
}


void msgInit(void) 
{
    cloudPathValid = 0;
    cloudDataMsg.ID = realID;
    cloudDataMsg.logIn = false;
    cloudDataMsg.isSoftReboot = false;
    cloudDataMsg.isAutoDrive = false;
    cloudDataMsg.isAvoidObstacles = false;
    cloudDataMsg.isControl = false;
    cloudDataMsg.isRoutecomplete = false;
    cloudDataMsg.isManualPathF = false;
    cloudDataMsg.isManualPathB = false;
    cloudDataMsg.isShutdown = false;
    cloudDataMsg.cloudGas = 0.0;
    cloudDataMsg.cloudSteerAngle = 0.0;
    cloudDataMsg.cloudLat = 0.0;
    cloudDataMsg.cloudLon = 0.0;


    cloudDataMsg.isCanAvoidObstacles = true;
    cloudDataMsg.isArrive = false;
    cloudDataMsg.currentLat = 0.0;
    cloudDataMsg.currentLon = 0.0;
}


void *cloudCallbackThread(void *ptr)
{
    ros::Rate loop_rate(1000);
    std_msgs::String ClientRecData; 
    tcpBuffer tBuffer;
    char buffer[1000];
    char checkBuffer[1000];
    int isAgain = 0;
    int vailBit = 0;
    int isFoundStart = 0;
    int isFoundStop = 0;
    int RecLen = 0;

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8801);//云平台
    addr.sin_addr.s_addr = inet_addr("47.103.47.192");
    //  addr.sin_port = htons(8801);//模拟串口
    // addr.sin_addr.s_addr = inet_addr("192.168.8.103");
	// addr.sin_port = htons(8801);//模拟串口lym
	//  addr.sin_addr.s_addr = inet_addr("192.168.43.150");
    int ClientSocket = socket(AF_INET, SOCK_STREAM,0);
    if(ClientSocket == -1) {
        ROS_INFO("Socket creat error");
        return 0;
    }
    int Res = connect(ClientSocket,(struct sockaddr*)&addr,sizeof(addr));
    if(Res == -1)
    {
        ROS_INFO("Server connect fail");
        return 0;
    }
    ROS_INFO("Server connect success");
    int options = fcntl(ClientSocket, F_GETFL, 0); 
    fcntl(ClientSocket, F_SETFL, options | O_NONBLOCK); 
    msgInit();
    while(ros::ok())
    {
        bzero(buffer,1000);
        bzero(checkBuffer,1000);
        RecLen = recv(ClientSocket,buffer,sizeof(buffer),0);
        if(RecLen > 0) {
            tBuffer.push(buffer, RecLen);
            while(1) {
                tBuffer.check(checkBuffer, 1000);
                for(int i = 0; i < 1000; i++) {
                    if(checkBuffer[i] == 0x23 && checkBuffer[i+1] == 0x23 && isFoundStart == 0) {
                        isFoundStart = 1;
                        vailBit = i;
                        // printf("vailBit = %d\n", vailBit);
                    }
                    if(isFoundStart == 1 && checkBuffer[i] == 0x24 && checkBuffer[i+1] == 0x24) {
                        isFoundStop = 1;
                        if((checkBuffer[i+2] == 0x23 || checkBuffer[i+3] == 0x23 || checkBuffer[i+4] == 0x23) \
                        && (checkBuffer[i+83] == 0x0B || checkBuffer[i+84] == 0x0B || checkBuffer[i+85] == 0x0B)) {
                            isAgain = 1;
                        }
                        bzero(buffer,1000);
                        tBuffer.pop(buffer, vailBit);
                        // Array2Hex(checkBuffer, 249, PrintOut);
                        // ROS_INFO_STREAM("pop Date: " << PrintOut);
                        break;
                    }
                }
                if(isFoundStop == 1) {
                    bzero(buffer,1000);
                    tBuffer.pop(buffer, 248);
                    DataReceivePrepar(buffer);
                    // Array2Hex(buffer, 249, PrintOut);
                    // ROS_INFO_STREAM("Rec Date: " << PrintOut);
                }
                isFoundStop = 0;
                isFoundStart = 0;
                vailBit = 0;

                if(isAgain == 0) {
                    break;
                }

                isAgain = 0;
            }
            // ClientRecData.data = buffer;
            if(cloudDataMsg.logIn == false && buffer[0] == 0x30) {
                msgInit();
                cloudDataMsg.logIn = true;
                isPubFlag = true;
                printf("登入成功\n");
                printf("当前设备ID：%016d\n", cloudDataMsg.ID);
            }
        }
        if(sendFlag == 1) {
            sendFlag = 0;
            PackSendData(1,cloudDataMsg.ID,cloudDataMsg.ID,9,1,0,1,1,0,0);
            send(ClientSocket, DataToSend, 248, 0);
            cloudDataMsg.logIn = false;
        } else if(sendFlag == 2){
            sendFlag = 0;
            PackSendData(1,cloudDataMsg.ID,cloudDataMsg.ID,9,7,0,1,1,0.0,0.0);
            send(ClientSocket, DataToSend, 248, 0);
        } else if(sendFlag == 3){
            sendFlag = 0;
            PackSendData(1,cloudDataMsg.ID,cloudDataMsg.ID,9,8,0,1,1,0.0,0.0);
            send(ClientSocket, DataToSend, 248, 0);
        } else if(sendFlag == 4){
            sendFlag = 0;
            PackSendData(1,cloudDataMsg.ID,cloudDataMsg.ID,9,6,0,1,1,cloudDataMsg.cloudLat,cloudDataMsg.cloudLon);
            send(ClientSocket, DataToSend, 248, 0);
        }
        // loop_rate.sleep();
    }
}

void delay_s(int v)
{
    ros::Rate gas_rate(1);
    while(v--)
        gas_rate.sleep();
}


bool logInCloud(int ID)
{
    realID = ID;
    cloudDataMsg.ID = ID;
    sendFlag = 1;
    printf("发送：正在登入\n");
    delay_s(3);
    return cloudDataMsg.logIn;
}

int getManualModel(void);

vector<double> getPath(int x = 0)
{   
    // 该变量在云平台更新完路径后置为1，读取完路径后置为0
    cloudPathValid = 0;
    
    // ^ 这段代码主要用于测试本地下发路径
    //   真实测试需要注释
// 	if(getManualModel()==0)//自动驾驶模式
// 	{
// 		vector<double> simPath;
// 		//   设置惯导模拟GPS经纬度
// 		simPath.push_back(31.9543877);

// simPath.push_back(112.087944);
// simPath.push_back(31.9543877);

// simPath.push_back(112.0879059);
// simPath.push_back(31.9543877);

// simPath.push_back(112.0878372);
// simPath.push_back(31.9543877);

// simPath.push_back(112.0877609);
// simPath.push_back(31.9543858);

// simPath.push_back(112.0876541);
// simPath.push_back(31.9543858);

// simPath.push_back(112.0875473);
// simPath.push_back(31.9543858);

// simPath.push_back(112.0874176);
// simPath.push_back(31.9543858);

// simPath.push_back(112.0872726);
// simPath.push_back(31.9543533);

// simPath.push_back(112.0872116);
// simPath.push_back(31.9542904);

// simPath.push_back(112.0872192);
// simPath.push_back(31.9541893);

// simPath.push_back(112.0872269);
// simPath.push_back(31.9540844);

// simPath.push_back(112.0872269);
// simPath.push_back(31.9540081);

// simPath.push_back(112.0872345);
// simPath.push_back(31.9539032);

// simPath.push_back(112.0872345);
// simPath.push_back(31.9538555);

// simPath.push_back(112.0872421);
// simPath.push_back(31.9538555);

// simPath.push_back(112.0872955);
// simPath.push_back(31.9538574);

// simPath.push_back(112.0873718);
// simPath.push_back(31.9538555);

// simPath.push_back(112.0874939);
// simPath.push_back(31.9538555);

// simPath.push_back(112.0875854);
// simPath.push_back(31.9538612);

// simPath.push_back(112.0876846);
// simPath.push_back(31.9539413);

// simPath.push_back(112.0877304);
// simPath.push_back(31.9540062);

// simPath.push_back(112.0877686);
// simPath.push_back(31.9540882);

// simPath.push_back(112.0878067);
// simPath.push_back(31.9542122);

// simPath.push_back(112.087883);
// simPath.push_back(31.954298);

// simPath.push_back(112.0879288);
// simPath.push_back(31.9543877);

// simPath.push_back(112.0879669);
// simPath.push_back(31.9543896);

// simPath.push_back(112.0879364);


//		printf("return fake simpath\n");

//			return simPath;
		
//	}
//	else {

		// v

		printf("return real cloudpath\n");
		return cloudPath;
//	}
}

void compelStop1(void)
{
    cloudDataMsg.isAutoDrive = false;
}

void compelStart1(void)
{
    cloudDataMsg.isAutoDrive = true;
}

void compelManualModelfalse(void)
{
    cloudDataMsg.isManualPathF=0;
    cloudDataMsg.isManualPathB=0;
}

bool getStart(void)
{
    return cloudDataMsg.isAutoDrive;
    // return true;
}

bool getStop(void)
{
    return !cloudDataMsg.isAutoDrive;
    // return false;
}

bool getTurn(void)
{
    return cloudDataMsg.isAvoidObstacles;
    // return true;
}

int getManualModel()
{
    
    if(cloudDataMsg.isManualPathF) {
        return 1;
    } else if(cloudDataMsg.isManualPathB) {
        return 2;
    } else {
        return 0;
    }
     //return true;
}

bool getShutdown(void)
{
    return cloudDataMsg.isShutdown;
}

void reachTarget(void)
{
    sendFlag = 2;
    printf("发送：到达目的地\n");
}

void cannotAvoidObstacles(void) 
{
    sendFlag = 3;
    printf("发送：不能绕开障碍物\n");
}

void reportCurrentLatLon(double lat, double lon)
{
    sendFlag = 4;
    cloudDataMsg.cloudLat = lat;
    cloudDataMsg.cloudLon = lon;
    printf("发送：上报经纬度\n");
}

#endif