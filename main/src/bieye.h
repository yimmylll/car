#ifndef __BIEYE_H__
#define __BIEYE_H__

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

int isNewTX2Shutdown = 0;
double obsNum = 0.0;
double obsNums[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void newTX2Shutdown(void)
{
    isNewTX2Shutdown = 1;
}

void bieyeArray2Hex(const char *buf, int len, char *out)
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

void bieyeDataAnalysis(const char * RecData)
{
    if(*RecData == '#' && *(RecData + 3) == '$'){
        int obsNumInt = (*(RecData + 1) - 48)*10 + (*(RecData + 2) - 48);
        obsNums[9] = obsNums[8];
        obsNums[8] = obsNums[7];
        obsNums[7] = obsNums[6];
        obsNums[6] = obsNums[5];
        obsNums[5] = obsNums[4];
        obsNums[4] = obsNums[3];
        obsNums[3] = obsNums[2];
        obsNums[2] = obsNums[1];
        obsNums[1] = obsNums[0];
        obsNums[0] = double(obsNumInt);
        obsNum = obsNums[0]*0.15 + obsNums[1]*0.15 + obsNums[2]*0.15 + obsNums[3]*0.1 + obsNums[4]*0.1 + obsNums[5]*0.1 + obsNums[6]*0.1 + obsNums[7]*0.05 + obsNums[8]*0.05 + obsNums[9]*0.05;
        // ROS_INFO("obs num :%d", obsNum);
    }
}

void *bieyeCallbackThread(void *ptr)
{
    ros::Rate loop_rate(100);
    std_msgs::String ClientRecData; 
    char buffer[256];
    char PrintOut[5];
    int RecLen = 0;
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8889);
    addr.sin_addr.s_addr = inet_addr("192.168.8.104");//wifiip
    int ClientSocket = socket(AF_INET, SOCK_STREAM,0);
    if(ClientSocket == -1) {
        ROS_INFO("Socket creat error");
        return 0;
    }
    int Res = connect(ClientSocket,(struct sockaddr*)&addr,sizeof(addr));
    if(Res == -1)
    {
        ROS_INFO("New TX2 server connect fail");
        return 0;
    }
    ROS_INFO("New TX2 server connect success");
    int options = fcntl(ClientSocket, F_GETFL, 0); 
    fcntl(ClientSocket, F_SETFL, options | O_NONBLOCK); 

    while(ros::ok())
    {
        bzero(buffer,256);
        RecLen = recv(ClientSocket,buffer,sizeof(buffer),0);
        if(RecLen > 0) {
            ClientRecData.data = buffer;
            bieyeDataAnalysis(buffer);
            // bieyeArray2Hex(ClientRecData.data.c_str(), 4, PrintOut);
            // ROS_INFO_STREAM("Rec Date: " << PrintOut);
        }
        if(isNewTX2Shutdown == 1) {
            send(ClientSocket, "shutdown", 8, 0);
            isNewTX2Shutdown = 0;
        }
    }
}

double bieyeObstacle(void)
{
     //printf("obsNum\n");
     return obsNum;
     //return 0.0;
}


#endif