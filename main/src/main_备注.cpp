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

    int stop;
    int pstop=1;//初始不刹车

	int tu;
    int go=0;
    double ilat ;
    double ilon ;
    double tlat ;
    double tlon ;
    double igpsangle;
    float pGas=0;
    int targetIndex=0;	
    int manualMode = 0;
    //bool ismanualBack = 0;
    double angleBack = 0.0;
	double angleFront = 0.0;
    int startCnt = 0;
	int startCntF = 0;
    // 用于本地发送路径，只有当patho为0的时候发送一次
    int patho = 0;//
    int manualBack_go = 0;//手动倒车开始标志
	int manualFront_go = 0;//手动前进开始标志

//	int stop_DelayTimes = 31;//初始不刹车释放
/**********************************************/
    gas(0.0);
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

	//  while(ros::ok())//转向
	//   {
	//      loop_rate.sleep();
	//      ros::spinOnce(); 
	// 	// printf("brake0\n");
	//      brake(0.0);
	//   }
	//  printf("brake0\n");
	//   brake(0.0);
	//     delay_1s(2);
	//    return 0;


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
    // printf("start\n");
    // gas(1.5);
    // delay_1s(3);
    //printf("steer-15\n");
    //  while(ros::ok())//转向
    //  {
    //     loop_rate.sleep();
    //     ros::spinOnce(); 
    //     gas(0.0);
    //    delayTimes1++;
    //    steer(-15.0);//每次1°
    //    if(delayTimes1==30)//3s
    //    {break;}
     // }   
    //printf("gas0\n");
   // gas(0.0);
    //printf("steer0\n");
    //delayTimes1 = 0;
    //while(ros::ok())//回正
    //{
    //    loop_rate.sleep();
    //    delayTimes1++;
    //    steer(0.0);
    //    if(delayTimes1==20)//0.02s
    //    {break;}
    //}
    // printf("car back!\n");
    // gas(-1.0);
     //delay_1s(1);
    // printf("car stop\n");
    //  int delaytime000=0;
    // while(ros::ok())
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce(); 
    //     gas(0.0);
    //     delaytime000++;
    //     if(delaytime000==10){
    //         break;
    //     }
    // }
    // gas(0.0);
    // brake(brakeV);
    // delay_1s(2);
    // brake(0.0);
    //    delay_1s(2);
    //    return 0;
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
    //1111

    /****************************************cloud测试代码******************************************/
	////reachTarget() 到达目的地命令    cannotAvoidObstacles() 不能规避障碍命令  reportCurrentLatLon(ilat,ilon) 报告当前位置
	//  bool lastStart = 0;
	//  bool lastStop = 1;
	//  bool lastTurn = 0;
	//  int lastManualModel = 0;
	//  bool lastShutdown = 0;
	//  while(ros::ok())
	//  {
	//  		//printf("****************************************\n");
	//  		if(cloudPathValid==1){printf("cloudPathValid:%d\n",cloudPathValid);}
    //          printf("getManualModel:%d\n",getManualModel());

	//      if(cloudPathValid == 1) {   // cloudPathValid为1时表面路径数据已经准备好
	//          path = getPath(0);      // 获取路径数据
	//          int pathSize=path.size()/2;
	//          printf("获取经纬度路径:%d条\n", pathSize);
	//          for (int i = 0; i < path.size();i++) {  // 打印路径数据
	//              cout << path[i] << " ";
	//          }
	//          cout << " " << endl;
	// 			printf("getStart:%d  getStop:%d\n",getStart(),getStop());
	//      }
	//  	   if(getStart() == true && getStart() != lastStart) {
	//          printf("开始自动驾驶\n");
	//      } else if(getStop() == true && getStop() != lastStop) {
	//          printf("停止自动驾驶\n");
	//      } else if(getTurn() == true && getTurn() != lastTurn) {
	//          printf("躲避障碍物\n");
	//      } else if(getManualModel()==1 && getManualModel() != lastManualModel){
	//  			printf("开启手动front模式\n");
	//  	   }
    //        else if(getManualModel()==2 && getManualModel() != lastManualModel){
	//  			printf("开启手动back模式\n");
	// 	  }
    //       else if(getManualModel()==0 && getManualModel() != lastManualModel)//compelManualModelfalse
    //       {
    //           printf("自动驾驶模式\n");
    //       }
	// 		else if (getShutdown() == true && getShutdown() != lastShutdown) {
	// 		   printf("关闭小车\n");
	// 	  }
	//      lastStart = getStart();
	//      lastStop = getStop();
	//      lastTurn = getTurn();
	//  	   lastManualModel=getManualModel();
	// 	  lastShutdown = getShutdown();
	//  	/**********************向云平台发送路径测试代码********************************/
	//     //  delayTimes++;//
	//     //  if(delayTimes == 30) {     // 每3s报告当前位置
	//  	// 	   ilat = cfGpsLat();
	//  	// 	   ilon = cfGpsLon();
	//     //      reportCurrentLatLon(ilat, ilon);
	//     //      printf("lat:%f   lon:%f\n", ilat, ilon);
	//  	// 	   delayTimes = 0;
	//     //  }
	//      loop_rate.sleep();
	//  }
	//  return 0;
	/********************************************end**************************************************/

    delay_1s(1);
	//printf("初始getTurn():%d\n", getTurn());
    while(ros::ok())
    {
		//printf("enter while\n");
	
         //printf("tofObstacle: %d\n",tofObstacle());
        // printf("bieyeobstacle: %lf\n",bieyeObstacle());
        //continue;
        
        //double idistance getDistance(ilat, ilon,)
        // printf("lat:%f   lon:%f\n", ilat, ilon);
        // continue;
        //moveTarget(ilat, ilon, 3156.913886, 11204.859963, igpsangle, 0.5, 2.5, 500, 3);
		// printf("cloudPathValid:%d\n", cloudPathValid);
        // printf("carCounter: %d\n", getCarCounter());
        // 获取路径
        loop_rate.sleep();
         //printf("manualMode:%d\n", manualMode);
		if(manualMode == 1 && cloudPathValid == 1) {//手动前进模式，收到点
			block = 0;//下发坐标默认初始无障碍
			compelStart1();//开启强制开始
			pstop = 1;//暂停标志位的上一个循环状态
	
			//stop_DelayTimes = 31;
            vector<double>().swap(path);
            path=getPath();
            pathSize=path.size()/2;
			targetIndex = 0;//清零目标标志位
			printf("手动前进pathsize:%d  path[0]:%8.4f  path[1]:%8.4f \n", pathSize, path[0], path[1]);//y,x 经、纬度
           
            if(pathSize > 0) {
				/****************************************/
				manualFront_go = 1; //开始手动前进
				frontfinish = 0; //倒车到达目的地 标志
				startCntF = getCarCounter();//获取当前计数值
				G_carCounter = startCntF; //更新 G_carCounter
				angleFront = (0.5 - path[1])*60.0;//前进车角度
				printf("angleFront:%f\n", angleFront);
				/****************************************/

    //            printf("G_steer:%f\n",G_steer);
    //            path = manualPathToMeter(path[0], path[1], G_steer);//实际path[0]收到云平台发送的y值!!
    //            ilat = 0.0;
    //            ilon = 0.0;
    //            igpsangle = 270.0;
    //           // printf("ilat:%f   ilon:%f\n",ilat,ilon);
    //            printf("manualpathtometer path[0]:%8.4f  path[1]:%8.4f\n", path[0], path[1]);//y,x 经、纬
    //            G_dx = latToMeter(ilat, path[0]);               // 小车坐标系
    //            G_dy = lonToMeter(ilon, path[1]);               // 小车坐标系
    //            G_angle = gpsAngleToCoordinate(igpsangle);      // 小车坐标系
    //            G_car.x = 0.0;                                  // carH坐标系
    //            G_car.y = 0.0;                                  // carH坐标系
    //            G_car.ang = coorToCarHAngle(G_angle);           // carH坐标系
    //            G_carCounter = getCarCounter();

    //           
				///*gas(0.0);
				//pGas = 0;*/

    //            printf("G_dx:%f, G_dy:%f, G_angle:%f, GCar_angle:%f, G_Counter:%d\n", \
    //                    G_dx, G_dy, G_angle, G_car.ang, G_carCounter);
            }
        } else if(manualMode == 2 && cloudPathValid == 1) {//手动倒车模式，收点
			compelStart1();//开启强制开始
			//stop_DelayTimes = 31;
            vector<double>().swap(path);
            path=getPath();
            pathSize=path.size()/2;
			targetIndex = 0;//走完清零
			printf("手动  倒车pathsize:%d  path[0]:%8.4f  path[1]:%8.4f\n", pathSize, path[0], path[1]);//y,x 经、纬度
            if(pathSize > 0) {
				manualBack_go = 1; //开始手动倒车
				backfinish = 0; //倒车到达目的地 标志
                startCnt = getCarCounter();//获取当前计数值
				G_carCounter = startCnt; //更新G_carCounter
                angleBack = (path[1] - 0.5)*60.0;//计算前进角度
                printf("angleBack:%f\n", angleBack);
            }
        } else if(targetIndex>=pathSize && cloudPathValid == 1) { //自动驾驶模式，收点
			//if (patho == 0) {
				block = 0;//下发坐标默认初始无障碍
				compelStop1();//收点后等待 开始标志下发 才开始
				pstop = 1;//初始不刹车
                vector<double>().swap(path);
				path = getPath(0);
				pathSize = path.size() / 2;
				targetIndex = 0;//清零目标索引
			//}
			printf("pathSize = %d\n", pathSize);    
		}

		// 检测GPS, getGpsStatus输入1模拟真实GPS，输入0模拟丢失GPS
		getGpsStatus(patho);
		//// 当patho为1时，获取真实GPS，之后再也不会获取真实GPS，以此来模拟惯导
		//patho++;
		// 如果找到GPS，则记录当前经纬度和航向角数据
        //printf("G_lostGps=%d\n",G_lostGps);
		if (G_lostGps == false) {//gps固定解时 记录当前坐标点
			ilat = cfGpsLat();          // GPS坐标系
			ilon = cfGpsLon();          // GPS坐标系
			igpsangle = cfGpsAngle();   // GPS坐标系

			//每5s发送一次当前位置
			delayTimes++;
			if (delayTimes == 50) {
				reportCurrentLatLon(ilat, ilon);
				printf("报告当前位置  ilat:%f  ilon:%f\n",ilat,ilon);
				delayTimes = 0;
			}
            // igpsangle = backCompass();
            // 如果路径没有走完，存储所有全局变量
            if(targetIndex < pathSize) {
                G_dx = latToMeter(ilat, path[2*targetIndex]);   // 小车坐标系
                G_dy = lonToMeter(ilon, path[2*targetIndex+1]); // 小车坐标系
                G_angle = gpsAngleToCoordinate(igpsangle);      // 小车坐标系
                G_car.x = 0.0;                                  // carH坐标系
                G_car.y = 0.0;                                  // carH坐标系
                G_car.ang = coorToCarHAngle(G_angle);           // carH坐标系
                G_carCounter = getCarCounter();
            }
            printf("G_dx:%f, G_dy:%f, G_angle:%f, GCar_angle:%f, G_Counter:%d\n", \
                    G_dx, G_dy, G_angle, G_car.ang, G_carCounter);
        }

        /*
        if(patho == 3) {
            break;
        } else {
            continue;
        }
        */
		//检测云平台是否关机
		if (getShutdown()) {
            printf("Enter getShutdown!\n");
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

		//检测云平台的停止信号
		stop = getStop();

		//检测云平台是否绕障碍
		tu=getTurn();
		// printf("manualmode:%d\n", manualMode);
		 //printf("start:%d   stop:%d   tu:%d\n", start, stop, tu);
		 //printf("pstop:%d\n", pstop);
		
		//停止小车
		if (stop == 1 ) {
			if (pstop == 0) {//判断stop标志位上升沿
				gas(0.0);//多点模式
				pGas = 0;
                if(manualMode==0){//手动模式不刹车，自动模式下发停车指令后，刹车
                    brake(brakeV);
                }				
				printf("car stop!---\n");
			}
			pstop = stop;//在下次循环中对比新循环的stop标志位状态，用于判断上升下降沿
			continue;
		}
		brake(0.0);
		pstop = 0;//启动小车后，当前循环的stop标志位状态就是0

        if(manualBack_go==1){//开始手动倒车
            printf("moveback abs(distance)\n");
            moveBack(angleBack, BackgasV, BackmoveDistance, startCnt);//手动倒车函数
            if(backfinish==1){//倒车到达目的地 标志
                printf("wait for next manual_Back point!\n");
                backfinish = 0;//清零手动倒车完成标志位
                targetIndex=1;
                manualBack_go = 0;//清零手动倒车标志位
				compelStop1();//开启强迫停止
				pstop = 0;
				printf("manual_back置stop=1，下一次应打印car stop刹死\n");
            }
            continue;
        }

		//**************//
		if (manualFront_go == 1) {//开始手动前进
			printf("moveforward abs(distance)\n");
			moveForward(angleFront, FrontgasV, FrontmoveDistance, startCntF);//手动前进函数
			if (frontfinish == 1) {//倒车到达目的地 标志
				printf("wait for next manual_Front point!\n");
				frontfinish = 0;//清零手动前进结束标志位
				targetIndex = 1;
				manualFront_go = 0;//清零手动前进标志位
				compelStop1();//开启强迫停止 置stop = 1
				pstop = 0;//使判断stop 上升沿
				printf("manual_front置stop=1，下一次应打印car stop刹死\n");
			}
			continue;
		}
		printf("**********************************\n");//每次小车启动且进入非倒车循环

		//到达第targetIndex个目标点
		double dis;
        if(G_lostGps == true) {//无gps固定解下距离计算
            dis = getDistanceInMeter(G_car);
             printf("G_car.x:%f  G_car.y:%f  G_dx:%f  G_dy:%f\n",G_car.x, G_car.y,G_dx, G_dy);
        } else {//有gps固定解下距离计算
            dis = getDistance(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1]);
			printf("ilat:%f  ilon:%f  path[2*targetIndex]:%f  path[2*targetIndex+1]:%f   targetIndex:%d\n", ilat, ilon, path[2 * targetIndex], path[2 * targetIndex + 1],targetIndex);
        }
        printf("离目标 dis:%f\n",dis);
	    if(dis<minDistance){//到达当前目标点
			targetIndex++;//下一目标
            FirstWrongCnt = 0;
            //走下一目标前，清零误点计数值，在movetarget turn函数里面，当离目标越来越远时计数值自增，当离远一定距离后，直接去下一目标点
			printf("reach target%d!\n", targetIndex);
			if (targetIndex >= pathSize ) {  //已走完所有目标点
				if (manualMode == 0) {//非手动模式下走完退出
					gas(0.0);
					pGas = 0;
					brake(brakeV);//自动驾驶结束，单点模式
					delay_1s(3);
					printf("car finish autodrive! Wait for next path\n");
					reachTarget();//发送到达目的地指令
					compelStop1();//更新停车标志位
					pstop = 0;
					continue;
				}
				printf("wait for next manual_Front path\n");
				compelStop1();//开启强迫停止
				pstop = 0;
				printf("manual_front置stop=1，下一次应打印car stop刹死\n");
				continue;
			}
            else if(G_lostGps == true) { //没有gps固定解时，更新G_car
                G_dx = latToMeter(ilat, path[2*targetIndex]);
                G_dy = lonToMeter(ilon, path[2*targetIndex+1]);
            }	
            gas(0.0);//这个会令车每次走到一个目标点就顿
            pGas = 0;	//油门从0慢慢增加到给定速度
		} else {
            //走第targetIndex个目标点
            if(tu){//绕障模式，遇到障碍绕开
                if(G_lostGps == true) {//无gps固定解
                    moveTargetWithTurnInMeter(G_car, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);
                } else {
                    moveTargetWithTurn(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1], igpsangle, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);			
                }
				if (CannotFindTarget == 1) {//如果无法到达当前目标点，在movetargetwithturn函数里面把此标志位置1
					CannotFindTarget = 0;
					targetIndex++;//放弃当前目标，走下一个点
				}
				if (targetIndex >= pathSize ) {//已走完所有目标点
                    if (manualMode == 0) {//自动模式下走完
                        gas(0.0);
                        pGas = 0;
                        brake(brakeV);//刹车
                        delay_1s(3);
                        printf("car finish autodrive! Wait for next path\n");
                        reachTarget();//发送到达目的地指令
                        compelStop1();//置stop标志位1
                        pstop = 0;//使检测stop上升沿
                        continue;
                    }
                    printf("wait for next manual_Front path\n");
                    compelStop1();//开启强迫停止
                    pstop = 0;//使下次判断stop上升延
                    printf("manual_front置stop=1，下一次应打印car stop刹死\n");
                    continue;
                }
            } else {//避障模式，遇到障碍停车
                if(G_lostGps == true) {//gps无固定解
                    moveTargetWithStopInMeter(G_car, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);
                } else {//gps有固定解
                    moveTargetWithStop(ilat, ilon, path[2*targetIndex], path[2*targetIndex+1], igpsangle, minDistance, gasV*gasRatio+pGas*(1-gasRatio), brakeV, brakeDealyTime,pGas,angleDis);			
                }
            }
            pGas=gasV*gasRatio+pGas*(1-gasRatio);//调整速度
		}
        ros::spinOnce();
    }
    // killTof();
    // pthread_exit(NULL);

    printf("brake0.0 delay1s\n");
    brake(0.0);
    delay_1s(1);
    return 0;
}
