#include <iostream>
#include <fstream>
using namespace std;
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "myguide/KFApp.h"
#include "myguide/EXGUIDE.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

ros::Publisher gpsInsPathPub;
ros::Publisher gpsInsAttPub;
static int gpsOutputFlag = 0;

// extern ros::Time nowtime;

// 接收到gps数据的时候，上传到参数服务器，
void gpsGet(sensor_msgs::NavSatFix gpsmsg){
    std::vector<double> gpspos;
    int gpsflag;
    gpspos.push_back(gpsmsg.latitude);
    gpspos.push_back(gpsmsg.longitude);
    gpspos.push_back(gpsmsg.altitude);
    gpsflag = 1;
    ros::param::set("std_gpspos",gpspos);
    ros::param::set("gpsGetFlag",gpsflag);
}

void gpsVGet(geometry_msgs::Vector3Stamped gpsVmsg){
    std::vector<double> gpsvel;
    // hector的gps速度是北西天
    gpsvel.push_back(-gpsVmsg.vector.y);
    gpsvel.push_back(gpsVmsg.vector.x);
    gpsvel.push_back(gpsVmsg.vector.z);
    ros::param::set("std_gpsvel",gpsvel);
}

void imuGet(sensor_msgs::Imu msg){
    static CKFApp kf(TS);
    static int count = 1; //imu求解次数
    static int gpsCount = 0;
    static PointCG pointcg(22.799674,113.959796);

    double wx = msg.angular_velocity.x;
    double wy = msg.angular_velocity.y;
    double wz = msg.angular_velocity.z;
    // ROS_INFO("imu的三个方向角速度为：%f,%f,%f",wx,wy,wz);

    double ax = msg.linear_acceleration.x;
    double ay = msg.linear_acceleration.y;
    double az = msg.linear_acceleration.z;
    // ROS_INFO("imu的三个方向加速度为：%f,%f,%f",ax,ay,az);

    double imuX;
    double imuY;
    // imu输出的经纬度被转换成xy。

    double accs[3];
    double gyro[3];
    accs[0] = ax; accs[1] = ay; accs[2] = az;
    gyro[0] = wx; gyro[1] = wy; gyro[2] = wz; 
	
    std::vector<double> std_gpspos;
    std::vector<double> std_gpsvn;
    std::vector<double> std_uwbpos;
    std::vector<double> std_uwbvn;

    double gpspos_middle[3];
    double gpsvn_middle[3];
    double uwbpos_middle[3];
    double uwbvn_middle[3];
    // gps和imu数据定义

    static CVect3 eb;
    static CVect3 db;

    static CVect3 gpspos=LLH(22.799674,113.959796,0);
    static CVect3 gpsvn=O31;
    static CVect3 uwbpos = LLH(22.799674,113.959796,0);
    static CVect3 uwbvn=O31;

    static int isIndoors = 0;

    // 从参数服务器那里拿真实位置
    double carrealx = 0;
    double carrealy = 0;
    if(gpsCount >= 2){
        // 求解两次再判断，避免那边还没上传数据到参数服务器
        ros::param::get("carrealx",carrealx);
        ros::param::get("carrealy",carrealy);
    }
    // 如果真实位置在房间内的范围，则没有gps数据，变为uwb数据
    double startX = 100;
    double endX = 200;
    double startY = 100;
    double endY = 200;
    if(carrealx >= startX && carrealx <= endX && carrealy >= startX && carrealy <= startY){
        isIndoors = 1;
    }else{
        isIndoors = 0;
    }

    if(count == 1){
        double yaw0 = C360CC180(90.0*glv.deg);  // 北偏东为正
        kf.Init(CSINS(a2qua(CVect3(0,0,yaw0)), O31, gpspos));  // 请正确初始化方位和位置
        eb = CVect3(0.0,0.0,0.0)*glv.dps;  // 陀螺零偏 deg/s
        //eb = CVect3(ax,ay,az);
        // db = 031;
    }
	CVect3 wm = (*(CVect3*)gyro-eb)*TS;
    // CVect3 vm = (*(CVect3*)accs*glv.g0-db)*TS;
	CVect3 vm = (*(CVect3*)accs-db)*TS;

    // 组合导航
    // 如果进入室内，那么将没有gps信号，使用uwb和ins组合导航
    if(!isIndoors){
        // 不在室内，则有gps/imu组合导航
        int gpsGetFlag;
        ros::param::get("gpsGetFlag",gpsGetFlag);
        if(gpsGetFlag == 1)
        {
            gpsCount += 1;
            gpsGetFlag = 0;
            gpsOutputFlag = 1; // 接收到了gps信号 
            ros::param::set("gpsGetFlag",gpsGetFlag);
            ros::param::get("std_gpspos",std_gpspos);
            ros::param::get("std_gpsvel",std_gpsvn);
            for (int i = 0; i < 3; i++)
            {
                gpspos_middle[i] = std_gpspos[i];
                gpsvn_middle[i] = std_gpsvn[i];
                // gpsvn_middle[i] = 0;
            }
            gpsvn = *(CVect3*)gpsvn_middle; 
            gpspos = *(CVect3*)gpspos_middle;
            gpspos.k = 0;
            kf.SetMeasGNSS(gpspos / 180 * PI, gpsvn);
            ROS_INFO("gps速度为：Ve=%f,Vn=%f",gpsvn.i,gpsvn.j);
        }
    }else{
        // 在室内则有uwb/imu组合导航
        int uwbGetFlag;
        ros::param::get("uwbGetFlag",uwbGetFlag);
        if (uwbGetFlag == 1)
        {
            ros::param::set("uwbGetFlag",0);
            ros::param::get("std_uwbpos",std_uwbpos);
            ros::param::get("std_uwbvn",std_uwbvn);
            for (int i = 0; i < 3; i++)
            {
                uwbpos_middle[i] = std_uwbpos[i];
                uwbvn_middle[i] = std_uwbvn[i];
            }
            uwbvn = *(CVect3*)uwbvn_middle; 
            uwbpos = *(CVect3*)uwbpos_middle;
            kf.SetMeasGNSS(uwbpos, uwbvn);
        }
        
    }
    
    kf.Update(&wm, &vm, 1, TS);
    pointcg.set(kf.sins.pos.i * 180.0 / PI,kf.sins.pos.j * 180.0 / PI); //输入是角度制

    // 写出文件
    ofstream dataFile;
    dataFile.open("imugpsData.txt", ofstream::app);
    if (gpsOutputFlag == 1)
    {
        // 朝TXT文档中写入数据，有gps数据，一并输出
        dataFile << wx << " " << wy << " " << wz
        << " " << ax << " " << ay << " " << az 
        << " " << count*0.01 << " " << gpsOutputFlag 
        << " " << gpspos.i << " " << gpspos.j << " " << gpspos.k 
        << " " << gpsvn.i << " " << gpsvn.j << " " << gpsvn.k << endl;
        // 关闭文档
        dataFile.close();
        gpsOutputFlag = 0;
    }else{
         // 朝TXT文档中写入数据，无gps数据，只输出imu数据
        dataFile << wx << " " << wy << " " << wz
        << " " << ax << " " << ay << " " << az 
        << " " << count*0.01 << " " << gpsOutputFlag 
        << " " << 0 << " " << 0 << " " << 0 
        << " " << 0 << " " << 0 << " " << 0 << endl;
        // 关闭文档
        dataFile.close();
        gpsOutputFlag = 0;
    }
    

    // if (!isIndoors)
    // {
    //     // pointcg.set(kf.avpi.pos.i * 180.0 / PI,kf.avpi.pos.j * 180.0 / PI); //输入是角度制
    //     pointcg.set(kf.sins.pos.i * 180.0 / PI,kf.sins.pos.j * 180.0 / PI); //输入是角度制
    // }
    // else{
    //     pointcg.set(kf.sins.pos.i * 180.0 / PI,kf.sins.pos.j * 180.0 / PI); //输入是角度制
    // }
    
    // 发布组合导航结果的路径消息
    static nav_msgs::Path gpsInspathmsg;
    ros::Time nowtime = ros::Time::now();
    gpsInspathmsg.header.frame_id = "map";
    gpsInspathmsg.header.stamp = nowtime;

    geometry_msgs::PoseStamped mypose;
    mypose.header.frame_id = "map";
    mypose.header.stamp = nowtime;
    mypose.pose.position.x = pointcg.getX();
    mypose.pose.position.y = pointcg.getY();
    mypose.pose.position.z = 0;
    mypose.pose.orientation.w = 1;
    gpsInspathmsg.poses.push_back(mypose);

    gpsInsPathPub.publish(gpsInspathmsg);

    // 把组合导航结果(小车的avp放到参数服务器，姿态用弧度表示，位置用平面xy)
    std::vector<double> gnssInsResult;
    gnssInsResult.push_back(kf.sins.att.i);gnssInsResult.push_back(kf.sins.att.j);gnssInsResult.push_back(kf.sins.att.k);//姿态加个负号，可能代码有问题
    gnssInsResult.push_back(kf.sins.vn.i);gnssInsResult.push_back(kf.sins.vn.j);gnssInsResult.push_back(kf.sins.vn.k);
    gnssInsResult.push_back(mypose.pose.position.x);gnssInsResult.push_back(mypose.pose.position.y);
    ros::param::set("gnssinsresult",gnssInsResult);
    geometry_msgs::Vector3 gnssInsYawMsg;
    gnssInsYawMsg.z = -kf.sins.att.k / PI * 180.0;
    gpsInsAttPub.publish(gnssInsYawMsg);

    // 把小车的位置和姿态输出到话题，以便在rviz中查看
    // geometry_msgs::Pose gpsInsAttMsg;
    // geometry_msgs::Quaternion q1;
    
    // gpsInsAttMsg.position.x = mypose.pose.position.x;
    // gpsInsAttMsg.position.y = mypose.pose.position.y;
    
    // ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,p1=%f,p2=%f,p3=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    // kf.sins.pos.i / PI * 180.0,kf.sins.pos.j / PI * 1s80.0,kf.sins.pos.k);

    // ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,p1=%f,p2=%f,p3=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    // kf.sins.pos.i / PI * 180.0,kf.sins.pos.j / PI * 180.0,kf.sins.pos.k);

    // ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,x=%f,y=%f,h=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    // pointcg.getX(),pointcg.getY(),kf.avpi.pos.k);
    count += 1;

    
    // 写出文件
    // ofstream dataFile;
    // dataFile.open("dataFile.txt", ofstream::app);
    // // 朝TXT文档中写入数据
    // dataFile << count << " " << kf.sins.pos.i / PI * 180.0 << " " << kf.sins.pos.j / PI * 180.0 << " " << kf.sins.pos.k << endl;
    // // 关闭文档
    // dataFile.close();
    
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"solveimu");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu",10000,imuGet);
    ros::Subscriber gpsSub = nh.subscribe("/sensor_msgs/NavSatFix",10000,gpsGet);
    ros::Subscriber gpsVSub = nh.subscribe("/sensor_msgs/NavSatFix_Velocity",10000,gpsVGet);
    gpsInsPathPub = nh.advertise<nav_msgs::Path>("/mycar/gpsinspath",20);
    gpsInsAttPub = nh.advertise<geometry_msgs::Vector3>("/mycar/gpsinsatt",5);
    ros::spin();
    return 0;
}