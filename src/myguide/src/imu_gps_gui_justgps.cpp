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
#include "myguide/EXGUIDE.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

//静态发布器
static ros::Publisher justGpsPathPub;
static ros::Publisher justInsPathPub;
static ros::Publisher justInsAttPub;
static ros::Publisher justInsVelPub;

void imuGet(sensor_msgs::Imu msg){
    static CKFApp kf(TS);
    static int count = 1;
    static PointCG pointcg(22.799674,113.959796);
    ros::Time nowTime = msg.header.stamp;
    double timeDouble = double(nowTime.nsec) / pow(10,9);
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
	

    static CVect3 eb;
    static CVect3 db;
    if(count == 1){
        double yaw0 = C360CC180(90.0*glv.deg);  // 北偏东为正
        CVect3 gpspos=LLH(22.799674,113.959796,0), gpsvn=O31;
        kf.Init(CSINS(a2qua(CVect3(0,0,yaw0)), O31, gpspos));  // 请正确初始化方位和位置
        eb = CVect3(0.0,0.0,0.0)*glv.dps;  // 陀螺零偏 deg/s
        // eb = CVect3(ax,ay,az);
        db=O31;
    }
	CVect3 wm = (*(CVect3*)gyro-eb)*TS;
    // CVect3 vm = (*(CVect3*)accs*glv.g0-db)*TS;
	CVect3 vm = (*(CVect3*)accs-db)*TS;

    kf.Update(&wm, &vm, 1, TS);
    pointcg.set(kf.sins.pos.i * 180.0 / PI,kf.sins.pos.j * 180.0 / PI); //输入是角度制

    // 发布纯imu求解的信息话题
    geometry_msgs::Vector3 attmsg;
    geometry_msgs::Vector3 velmsg;
    attmsg.x = kf.sins.att.i / PI * 180;
    attmsg.y = kf.sins.att.j / PI * 180;
    attmsg.z = kf.sins.att.k / PI * 180;
    justInsAttPub.publish(attmsg);
    

    velmsg.x = kf.sins.vn.i;
    velmsg.y = kf.sins.vn.j;
    velmsg.z = kf.sins.vn.k;
    justInsVelPub.publish(velmsg);
    

    // 发布路径话题
    static nav_msgs::Path inspathmsg;
    geometry_msgs::PoseStamped inspose;
    ros::Time nowtime = ros::Time::now();
    
    inspose.header.frame_id = "map";
    inspose.header.stamp = nowtime;
    inspose.pose.position.x = pointcg.getX();
    inspose.pose.position.y = pointcg.getY();
    inspose.pose.orientation.w = 1;

    inspathmsg.header.frame_id = "map";
    inspathmsg.header.stamp = nowtime;
    inspathmsg.poses.push_back(inspose);

    // ROS_INFO("INS输出的姿态为:x=%f,y=%f,z=%f,速度为：x=%f,y=%f,z=%f,位置为:x=%fm,y=%fm时间为：%fs",attmsg.x,attmsg.y,attmsg.z,velmsg.x,velmsg.y,velmsg.z,inspose.pose.position.x,inspose.pose.position.y,count * 0.01);

    justInsPathPub.publish(inspathmsg);
    // ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,p1=%f,p2=%f,p3=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    // kf.sins.pos.i / PI * 180.0,kf.sins.pos.j / PI * 180.0,kf.sins.pos.k);
    // ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,x=%f,y=%f,h=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    // pointcg.getX(),pointcg.getY(),kf.sins.pos.k);
    
    // // 写出文件
    // ofstream dataFile;
    // dataFile.open("dataFile.txt", ofstream::app);
    // // 朝TXT文档中写入数据
    // ROS_INFO("输出的偏航角：%f",attmsg.z);
    // ROS_INFO("INS输出的姿态为:x=%f,y=%f,z=%f,速度为：x=%f,y=%f,z=%f,位置为:x=%fm,y=%fm时间为：%fs",attmsg.x,attmsg.y,attmsg.z,velmsg.x,velmsg.y,velmsg.z,inspose.pose.position.x,inspose.pose.position.y,count * 0.01);
    // dataFile << wx << " " << wy << " " << wz
    // << " " << ax << " " << ay << " " << az 
    // << " " << count*0.01 
    // << " " << attmsg.x << " " << attmsg.y << " " << attmsg.z 
    // << " " << velmsg.x << " " << velmsg.y << " " << velmsg.z 
    // << " " << inspose.pose.position.x << " " << inspose.pose.position.y 
    // << " " << kf.sins.pos.i << " "<< kf.sins.pos.j << endl;
    // // 关闭文档
    // dataFile.close();

    // 输出纯imu数据
    ofstream dataFile;
    dataFile.open("imuDatas.txt", ofstream::app);
    dataFile << std::fixed << std::setprecision(10); // 设置精度为8位小数
    // 朝TXT文档中写入数据
    dataFile << wx << " " << wy << " " << wz
    << " " << ax << " " << ay << " " << az 
    << " " << count*0.01 << endl;
    // 关闭文档
    dataFile.close();

    count += 1;
}

void gpsGet(sensor_msgs::NavSatFix gpsmsg){
    static PointCG pointcg(22.799674,113.959796);
    pointcg.set(gpsmsg.latitude,gpsmsg.longitude); //输入是角度制

    //ROS_INFO("gps输出的x位置为:%f,Y位置为%f",pointcg.getX(),pointcg.getY());
    static int gpsCount = 1;
    static int isIndoors = 0;
    

    // 发布ros路径话题
    ros::Time nowtime = ros::Time::now();
    static nav_msgs::Path gpspathmsg;
    gpspathmsg.header.frame_id = "map";
    gpspathmsg.header.stamp = nowtime;

    geometry_msgs::PoseStamped gpspose;
    gpspose.header.frame_id = "map";
    gpspose.header.stamp = nowtime;
    gpspose.pose.position.x = pointcg.getX();
    gpspose.pose.position.y = pointcg.getY();
    //gpspose.pose.orientation.w = 1;
    
    
    // 从参数服务器那里拿真实位置
    double carrealx = 0;
    double carrealy = 0;

    if(gpsCount >= 3){
        // 求解两次再判断，避免那边还没上传数据到参数服务器
        ros::param::get("carrealx",carrealx);
        ros::param::get("carrealy",carrealy);
    }
    if(carrealx >= 200 && carrealx <= 800 && carrealy >= -4 && carrealy <= 4){
        isIndoors = 1;
    }else{
        isIndoors = 0;
    }
    // isIndoors = 0;
    if(!isIndoors){
        gpspathmsg.poses.push_back(gpspose);
    }
    justGpsPathPub.publish(gpspathmsg);
    gpsCount += 1;

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"solveimu");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu",10000,imuGet);
    // 测试平面坐标转化代码
    ros::Subscriber sub2 = nh.subscribe("/sensor_msgs/NavSatFix",10000,gpsGet);
    justGpsPathPub = nh.advertise<nav_msgs::Path>("/mycar/gpspath",20);
    justInsPathPub = nh.advertise<nav_msgs::Path>("/mycar/inspath",20);
    justInsAttPub = nh.advertise<geometry_msgs::Vector3>("/mycar/insatt",20);
    justInsVelPub = nh.advertise<geometry_msgs::Vector3>("/mycar/insvel",20);
    //justInsAttPub = nh.advertise<>("",)
    ros::spin();
    return 0;
}