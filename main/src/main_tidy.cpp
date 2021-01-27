#include "main.h"

int printControl = 0;

CSMiddleWare::SPData carDataMsg;
cloudProtocol::cloudData cloudDataMsgTest;
double lat = 0.0;
double lon = 0.0;
double gpsAngle = 0.0;
int gpsQF = 0;
double backCompassAngle = 0.0;
double steerAngle = 0.0;
double frontCompassAngle = 0.0;
unsigned int carCounter = 0;
unsigned int preCarCounter = 0;
int block = 0;

int cloudPathValid = 0;

double G_dx;    // 丢失GPS之前小车到下一个目标x方向的距离，对应经度
double G_dy;    // 丢失GPS之前小车到下一个目标y方向的距离，对应纬度
double G_angle; // 丢失GPS之前小车到下一个目标的角度
bool G_lostGps; // 是否丢失GPS
car G_car;      // 当前车辆
int G_carCounter;// 上一次速度编码器计数	
float G_steer;  // 当前转向角

bool indoorStartUp = true;
int backfinish = 0;//倒车到达目的地标志
int frontfinish = 0;//手动前几到达目的地标志
int CannotFindTarget = 0;
int FirstWrongCnt = 0;//误点计数值
//bieye0


int main(int argc, char **argv)
{
	// parameters^
	float minDistance=0.5;	
	float brakeV=700;	
	float gasV=1.0;
	float FrontgasV = 0.8;
	float BackgasV = -1.5;
	float BackmoveDistance = 0.2;
	float FrontmoveDistance = 0.5;
	float gasRatio=0.1;  //speed changing rate
	float angleDis=3;  //distance for damping angle
    int brakeDealyTime =3;		
	// parametersV
	G_car.width = 0.5;
    G_car.length = 1.0;
    pthread_t idSerial, idCloud, idBieye;
    // 初始化ROS节点
    ros::init(argc, argv, "main");
 
    // 创建节点句柄
    ros::NodeHandle n;
    ros::NodeHandle *pn = &n;

    // 创建一个Subscriber，订阅名为/gpsread的topic，订阅话题的队列长度10，注册回调函数gpsCallback
    ros::Subscriber gps_sub = n.subscribe("/gpsread", 10, gpsCallback);
    ros::Subscriber carInfo_sub = n.subscribe("/SPRead", 1000, carInfoCallback);
    ros::Subscriber frontphoneInfo_sub = n.subscribe("/TCPRead", 1000, frontphoneInfoCallback);
    ros::Subscriber backphoneInfo_sub = n.subscribe("/TCPReadBack", 1000, backphoneInfoCallback);

    ros::Rate loop_rate(10);

    int ret = pthread_create(&idSerial, NULL, serialCallbackThread, (void*)pn);
    if(ret) {
        ROS_INFO("Create serialPort pthread error!");
        return 1;
    }
    ROS_INFO("Create serialPort pthread success!");

    ret = pthread_create(&idCloud, NULL, cloudCallbackThread, NULL);
    if(ret) {
        ROS_INFO("Create cloud pthread error!");
        return 1;
    }
    ROS_INFO("Create cloud pthread success!");

    ret = pthread_create(&idBieye, NULL, bieyeCallbackThread, NULL);
    if(ret) {
        ROS_INFO("Create bieye pthread error!");
        return 1;
    }
    ROS_INFO("Create bieye pthread success!");

    vector<double> path;
    int pathSize=0;

    int start;
    int pstart=0;
	int tu;
    int singlepoint=0;
    int go=0;
    double dis;
    double ilat ;
    double ilon ;
    double tlat ;
    double tlon ;
    double igpsangle;
    float pGas = 0;
    int targetIndex=0;	
    int manualMode = 0;
    //bool ismanualBack = 0;
    double angleBack = 0.0;
	double angleFront = 0.0;
    int startCnt = 0;
	int startCntF = 0;
    // 用于本地发送路径，只有当patho为0的时候发送一次
    int patho = 0;//

//	int stop_DelayTimes = 31;//初始不刹车释放
/**********************************************/
    gas(0.0);
    pGas = 0;
    brake(0.0);
    // 为了避免大角度的转向，在开始之前将角度设为0
    G_steer = 0;
	int delayTimes0 = 0;
	while (ros::ok())//转向
	{
	    loop_rate.sleep();
	    delayTimes0++;
	    steer(0.0);//每次1°
	    if(delayTimes0==50)//5s
	    {break;}
	} 

	//return 0;//释放刹车，回正方向 后结束


	 /********************************刹车测试*********************************/
	// brake(0.0);
    // printf("brake0\n");
	// delay_1s(2);
	// return 0;

	/********************************compelStop1 compelStart1测试*********************************/
	// delay_1s(1);
	// printf("初始getStart:%d  getStop:%d  getTurn():%d\n", getStart(), getStop(), getTurn());
	// compelStop1();
	// printf("compelStop1()后 getStart:%d  getStop:%d  getTurn():%d\n", getStart(), getStop(), getTurn());
	// compelStart1();
	// printf("compelStop1()后 getStart:%d  getStop:%d  getTurn():%d\n", getStart(), getStop(), getTurn());
	// return 0;

    /********************************倒车测试*********************************/
    // delay_1s(1);
    // printf("enable\n");
    // gas(1.0);
    // delay_1s(3);
    // gas(0.0);
    // delay_1s(2);
    // gas(-1.0);
    // delay_1s(5);
    // gas(0.0);
    // delay_1s(2);
    // gas(1.0);
    // delay_1s(3);
    // gas(0.0);
    // delay_1s(1);
    // return 0;

   /***************速度测试 new steer*********************************/
    // while(ros::ok())//转向
    //  {
    //     loop_rate.sleep();
    //     ros::spinOnce(); 
    //     gas(0.0);
    //  }
     // gas(0.0);
    //    delay_1s(2);
    //   return 0;

    /***************小车控制转向测试 new steer*********************************/
    // int delayTimes1 = 0;
    // printf("start steer-15\n");
    // while(ros::ok())//转向
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce(); 
    //     gas(0.0);
    //     delayTimes1++;
    //     steer(-15.0);//每次1°
    //     if(delayTimes1==20)//2s
    //     {break;}
    // }
    // printf("start go 2s\n");
    // gas(1.5);
    // delay_1s(2);   
    // printf("stop\n");
    // gas(0.0);
    // printf("steer0\n");
    // delayTimes1 = 0;
    // while(ros::ok())//回正
    // {
    //    loop_rate.sleep();
    //    delayTimes1++;
    //    steer(0.0);
    //    if(delayTimes1==20)//2s
    //    {break;}
    // }
    // printf("car back!\n");
    // gas(-1.0);
    // delay_1s(1);
    // printf("car stop\n");
    // gas(0.0);
    // brake(brakeV);
    // delay_1s(2);
    // brake(0.0);
    // delay_1s(2);
    // return 0;
    /********************************************双目测试********************************************/
    // while(ros::ok())
    //  {
    //      loop_rate.sleep();
    //      ros::spinOnce(); 
    //      //printf("should 1\n");
    //      printf("real bieyeObstacle:%8.4f\n", bieyeObstacle());
    //  }
    //  return 0;
    /*********************************************end***********************************************/

    /*******************************************超声波测试*******************************************/
    // while(ros::ok())
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce(); 
    //     printf("u1:%8.4f ", ultrasonic(1));//前右
    //     printf("u2:%8.4f ", ultrasonic(2));//前左
    //     printf("u3:%8.4f ", ultrasonic(3));
    //     printf("u4:%8.4f ", ultrasonic(4));
    //     printf("u5:%8.4f ", ultrasonic(5));
    //     printf("u6:%8.4f\n", ultrasonic(6));
    // }
    // return 0;
    /*********************************************end***********************************************/

    /*****************************************验证GPS航向角******************************************/
    /*
    double cLat, cLon, cAngle;
    double latMeter, lonMeter, vAngle;
    cLat = cfGpsLat();
    cLon = cfGpsLon();
    cAngle = cfGpsAngle();
    latMeter = latToMeter(cLat, gpsLatLon2WGS84(3157.262664));
    lonMeter = -lonToMeter(cLon, gpsLatLon2WGS84(11205.266910));
    vAngle = vectorAngle(latMeter, lonMeter);
    printf("cAngle:%f, vAngle:%f\n",cAngle, vAngle);
    return 0;
    */
    /*********************************************end***********************************************/

    /********************************标定GPS航向角与手机电子罗盘**************************************/
    
   // gas(1.0);
    //steer(15);
    // double gpsAngleTest, phoneAngleTest;
    // double eAngle, mAngle = 0.0, rms = 0.0;
    // int index = 0;
    // while(ros::ok())
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce();  
       //// printf("carcounter:%d\n",getCarCounter());//打印编码器
        // gpsAngleTest = cfGpsAngle();
        // phoneAngleTest = backCompass();
        // printf("gpsAngle:%f, phoneAngle:%f\n", gpsAngleTest, phoneAngleTest);
        // eAngle = gpsAngleTest - phoneAngleTest;
        // mAngle += eAngle;
        // rms += eAngle*eAngle;
        // index++;
        // if(index > 600) {
        //     gas(0.0);
        //     mAngle = mAngle / index;
        //     rms = rms / index;
        //     rms = sqrt(rms - mAngle*mAngle);
        //     printf("mAngle:%f, rms:%f\n", mAngle, rms);
        //     break;
        // }
    // }
    // return 0;
    
    /**********************************************end*********************************************/

    /***************************************手动模式测试代码*****************************************/
    /*
    vector<double> s = manualPathToMeter(0.0, 1.0, -0.0);
    printf("x = %f, y = %f\n", s[0], s[1]);
    printf("dx = %f, dy = %f\n", latToMeter(0.0, s[0]), lonToMeter(0.0, s[1]));
    return 0;
    */
    /**********************************************end*********************************************/

    /***********************************编码器与GPS标定代码*****************************************/
    /*
    delay_1s(1);
    unsigned int carCounterF = getCarCounter();
    unsigned int carCounterB;
    double startLat = cfGpsLat();
    double startLon = cfGpsLon();
    double endLat, endLon;
    double dx, dy;
    double dis;

    printf("start: lat:%lf, lon:%lf, counter: %d\n", startLat, startLon, carCounterF);
    gas(2.0);
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();    
        if(getCarCounter() - carCounterF > 12000) {
            gas(0.0);
            brake(600.0);
            delay_1s(3);
            carCounterB = getCarCounter();
            endLat = cfGpsLat();
            endLon = cfGpsLon();
            dx = latToMeter(startLat, endLat); 
            dy = lonToMeter(startLon, endLon);
            dis = sqrt(pow(abs(dx), 2)+pow(abs(dy), 2));
            printf("end: lat:%lf, lon:%lf, counter: %d\n", endLat, endLon, carCounterB);
            printf("end: dx:%lf, dy:%lf, dis: %lf\n", dx, dy, dis);
            printf("end: encode dis:%d\n", carCounterB-carCounterF);
            delay_1s(1);
            brake(0.0);
            delay_1s(1);
            return 0;
        }
    }
    return 0;
    */
    /**********************************************end*********************************************/

    // runTof();
    // printf("run tof OK\n");

    /****************************************cloud登录代码******************************************/
    int ID = 1; //设置小车ID，每个车必须唯一
    int delayTimes = 0; // 延时计数
    while(!logInCloud(ID)) {    // 登入云平台，若没登上则阻塞，每3s重连一次，直到连上
        loop_rate.sleep();
    }
    printf("ID:%016d 云接入成功\n", ID);
    delay_1s(1);

    /****************************************cloud测试代码******************************************/
	//reachTarget() 到达目的地命令    cannotAvoidObstacles() 不能规避障碍命令  reportCurrentLatLon(ilat,ilon) 报告当前位置
	//  bool lastStart = 0;
	//  bool lastTurn = 0;
	//  int lastManualModel = 0;
	//  bool lastShutdown = 0;
    //  bool lastSinglePoint = 0;
	//  while(ros::ok())
	//  {
	//  	//打印路径数据
	//     if(cloudPathValid == 1) 
    //     {   
	//         path = getPath();      // 获取路径数据
	//         int pathSize=path.size()/2;
	//         printf("获取经纬度路径:%d条\n", pathSize);
	//         for (int i = 0; i < path.size();i++) 
    //         {
	//             cout << path[i] << " ";
	//         }
	//         cout << " " << endl;
	//     }
    //     //打印命令
	//  	if(getStart() == true && getStart() != lastStart) 
    //     {   printf("开始自动驾驶\n");} 
    //     else if(getStart() == false && getStart() != lastStop) 
    //     {   printf("停止自动驾驶\n");} 
        
    //     if(getTurn() == true && getTurn() != lastTurn) 
    //     {   printf("开启绕障模式\n");}
    //     else if(getTurn() == false && getTurn() != lastTurn)
    //     {   printf("关闭绕障模式\n");}
        
    //     if(getManualModel()==1 && getManualModel() != lastManualModel)
    //     {   printf("开启手动front模式\n");}
    //     else if(getManualModel()==2 && getManualModel() != lastManualModel)
    //     {	printf("开启手动back模式\n");}
    //     else if(getManualModel()==0 && getManualModel() != lastManualModel)//compelManualModelfalse
    //     {   printf("自动驾驶模式\n");}
    //     else
    //     {   printf("ERROR: manualmode=%d\n",getManualModel());}
		
    //     if (getShutdown() == true && getShutdown() != lastShutdown) 
    //     {  printf("关闭小车\n");}

    //     if (getSinglePoint() == 1 && getSinglePoint() != lastSinglePoint)
    //     {   printf("开启单点模式\n");}
    //     else if (getSinglePoint() == 0 && getSinglePoint() != lastSinglePoint)
    //     {   printf("关闭单点模式\n");}
	//     lastStart = getStart();
	//     lastStop = getStop();
	//     lastTurn = getTurn();
	//  	lastManualModel=getManualModel();
	// 	lastShutdown = getShutdown();
    //     lastSinglePoint = getSinglePoint();
	 	
	//     loop_rate.sleep();
	//  }
	//  return 0;
	/********************************************end**************************************************/

    /*******************GPS打印：每5s发送一次当前位置************************************/
    // while (ros::ok())
    // {
    //     delayTimes++;
    //     if (delayTimes == 50) {
    //         //记录当前GPS坐标和方向角
    //         ilat = cfGpsLat();
	// 		ilon = cfGpsLon();
    //         reportCurrentLatLon(ilat, ilon);
    //         printf("报告当前位置  ilat:%f  ilon:%f\n",ilat,ilon);
    //         delayTimes = 0;
    //     }
    //     loop_rate.sleep();
    // }
    // return 0;
    /***************************************************/
    
    delay_1s(1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

         //检测云平台是否关机
		if (getShutdown()) {
            printf("MES:Enter getShutdown!\n");
			gas(0.0);
			brake(brakeV);
			delay_1s(3);
			brake(0.0);
			delay_1s(1);
			shutdown();
		}

		//检测云平台是否手动前进/倒退信号
        manualMode = getManualModel();

		//检测云平台的开始信号
		start = getStart();

		//检测云平台是否绕障碍
		tu = getTurn();

        //检测云平台是否单点模式
        singlepoint = getSinglePoint();//to do....................

        // 检测GPS, getGpsStatus输入1模拟真实GPS，输入0模拟丢失GPS
		getGpsStatus();
        // printf("manualmode:%d    start:%d    tu:%d\n",\
        //  manualMode, start, tu);
		// 当patho为1时，获取真实GPS，之后再也不会获取真实GPS，以此来模拟惯导
		//patho++;

        
        if (G_lostGps == false)//自动模式下有GPS差分信号
        {
            //记录当前GPS坐标和方向角
            ilat = cfGpsLat();
			ilon = cfGpsLon();
			igpsangle = cfGpsAngle();

            //每5s发送一次当前位置
            delayTimes++;
            if (delayTimes == 50) 
            {
                reportCurrentLatLon(ilat, ilon);
                printf("MES:报告当前位置 ilat:%f  ilon:%f\n",ilat,ilon);
                delayTimes = 0;
            }

            //存储所有全局变量,惯导用
            G_dx = latToMeter(ilat, path[2*targetIndex]);   // 小车坐标系
            G_dy = lonToMeter(ilon, path[2*targetIndex+1]); // 小车坐标系
            G_angle = gpsAngleToCoordinate(igpsangle);      // 小车坐标系
            G_car.x = 0.0;                                  // carH坐标系
            G_car.y = 0.0;                                  // carH坐标系
            G_car.ang = coorToCarHAngle(G_angle);           // carH坐标系
            G_carCounter = getCarCounter();
        }

        //接收路径
        if (cloudPathValid == 1)
        {
            printf("MES:enter cloudPathValid == 1\n");
            vector<double>().swap(path);
            path=getPath();//cloudPathValid会清0
            pathSize=path.size()/2;
			targetIndex = 0;//清零
            if (pathSize > 0)
            {
                if (manualMode == 1)
                {
                    frontfinish = 0;
                    startCntF = getCarCounter();//获取当前计数值
                    G_carCounter = startCntF; //G_carCounter？
                    angleFront = (0.5 - path[1])*60.0;//反过来!
                    compelStart1();
                    //printf("    手动前进pathsize:%d  path[0]:%8.4f  path[1]:%8.4f \n", pathSize, path[0], path[1]);//y,x 经、纬度
                    printf("    angleFront:%f\n", angleFront);
                }
                else if (manualMode == 2)
                {
                    backfinish = 0; //倒车到达目的地 标志
                    startCnt = getCarCounter();//获取当前计数值
                    G_carCounter = startCnt; //G_carCounter？
                    angleBack = (path[1] - 0.5)*60.0;
                    compelStart1();
                    //printf("    手动倒退pathsize:%d  path[0]:%8.4f  path[1]:%8.4f \n", pathSize, path[0], path[1]);//y,x 经、纬度
                    printf("    angleBack:%f\n", angleBack);
                }
                else
                {
                    printf("    AutoDrive--pathSize:%d\n", pathSize);
                    compelStop1();
                }
                
            }
            else
            {
                printf("ERROR: pathSize:%d  pathSize<=0\n", pathSize);
            }
        }
        
		
		//停止小车
		if (start == 0)
        {
			if (pstart == 1) 
            {
				gas(0.0);
                pGas = 0;
                if(singlepoint == 1)//单点模式
                {   brake(brakeV);}
                else
                {   brake(0.0);}
    			printf("MES:car stop!---\n");
			}
			pstart = start;
			continue;
		}

        //启动小车
        else
        {
            pstart = start;
            brake(0.0);

            if (manualMode==1)//手动前进
            {
                moveForward(angleFront, FrontgasV, FrontmoveDistance, startCntF);
                if (frontfinish == 1) {//前进到达目的地 标志
                    frontfinish = 0;
                    targetIndex = 1;
                    compelStop1();//开启强迫停止
                    printf("MES:manual_front finish\n");
                }
                continue;
            }
            else if (manualMode==2)//手动后退
            {
                moveBack(angleBack, BackgasV, BackmoveDistance, startCnt);
                if(backfinish==1){//倒车到达目的地 标志
                    backfinish = 0;
                    targetIndex=1;
                    compelStop1();//开启强迫停止
                    printf("MES:manual_back finish\n"); 
                }
                continue;
            }
            else//自动驾驶
            {
                //计算离当前目标距离
                if (G_lostGps == true)///////惯导
                {
                    dis = getDistanceInMeter(G_car);
                    printf("Warning:Lost GPS");
                    // printf("    G_car.x:%f  G_car.y:%f  \
                    // G_dx:%f  G_dy:%f\n", G_car.x, G_car.y,G_dx, G_dy);
                }
                else//GPS
                {
                    dis = getDistance(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1]);
		        	//printf("    ilat:%f  ilon:%f  path[2*targetIndex]:%f  path[2*targetIndex+1]:%f   targetIndex:%d\n", ilat, ilon, path[2 * targetIndex], path[2 * targetIndex + 1],targetIndex);
                }
                printf("MES:离目标距离 %f m\n", dis);

                /***************************/
                if (dis < minDistance)//到达当前目标点
                {
                    printf("MES:reach target%d\n", targetIndex);
                    targetIndex++;
                    FirstWrongCnt = 0;
                }
                else//未到达当前目标点
                {
                    if (tu)//绕障
                    {
                        if(G_lostGps == true)
                        {
                            moveTargetWithTurnInMeter(G_car, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);
                        }
                        else
                        {
                            moveTargetWithTurn(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1], igpsangle, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);
                        }
                    }
                    else//避障
                    {
                        if(G_lostGps == true)
                        {
                            moveTargetWithStopInMeter(G_car, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);
                        } 
                        else 
                        {
                            moveTargetWithStop(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1], igpsangle, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);			
                        }
                    }
                    if(CannotFindTarget == 1)
                    {
                        CannotFindTarget = 0;
                        printf("Warning:放弃目标%d\n", targetIndex);
                        targetIndex++;//放弃当前目标
                    }
                }

                //判断是否走完所有目标点
                if (targetIndex >= pathSize )
                { 
                    brake(brakeV);
                    delay_1s(3);
                    brake(0.0);
                    printf("MES:car finish autodrive! Wait for next path\n");
                    reachTarget();//发送到达目的地指令
                    compelStop1();
                }
                //未走完目标点，且丢失GPS，存储当前位置离下一目标距离
                else if(G_lostGps == true)  // && targetIndex < pathSize
                {
                    G_dx = latToMeter(ilat, path[2*targetIndex]);
                    G_dy = lonToMeter(ilon, path[2*targetIndex+1]);
                }

                //改变pGas
                pGas=gasV*gasRatio+pGas*(1-gasRatio);
            }
        }
    }
    //^ros::ok循环
    // killTof();
    // pthread_exit(NULL);

    brake(0.0);
    delay_1s(1);
    printf("brake0.0 delay1s\n");
    return 0;
}
