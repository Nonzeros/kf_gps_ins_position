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
    
    static int count = 1;
    static PointCG imuPointcg(22.799674,113.959796);

    double wx = msg.angular_velocity.x;
    double wy = msg.angular_velocity.y;
    double wz = msg.angular_velocity.z;
    // ROS_INFO("imu的三个方向角速度为：%f,%f,%f",wx,wy,wz);

    double ax = msg.linear_acceleration.x;
    double ay = msg.linear_acceleration.y;
    double az = msg.linear_acceleration.z;
    // ROS_INFO("imu的三个方向加速度为：%f,%f,%f",ax,ay,az);

    // 接收惯导数据求解，每两次求解一次
    static Matrix62d wvm;
    static double imuT = 0.01;
    static ImuSolver imusolver3;
    Vector10d avp00;
    avp00 << 0,0,89.95/180.0*PI,0,0,0,22.799674/180.0*PI,113.959796/180*PI,0,0;

    if (count % 2 != 0 )
    {
        // 奇数次获得imu输出不求解
        wvm(0,0) = wx * imuT; wvm(1,0) = wy * imuT; wvm(2,0) = wz * imuT;
        wvm(3,0) = ax * imuT; wvm(4,0) = ay * imuT; wvm(5,0) = az * imuT;
    }else{
        wvm(0,1) = wx * imuT; wvm(1,1) = wy * imuT; wvm(2,1) = wz * imuT;
        wvm(3,1) = ax * imuT; wvm(4,1) = ay * imuT; wvm(5,1) = az * imuT;
        imusolver3.sinsSolve(wvm,avp00,2*imuT);
        double Lat = imusolver3.avpOut(6);
        double Lon = imusolver3.avpOut(7);
        imuPointcg.set(Lat * 180.0 / PI,Lon * 180.0 / PI); //输入是角度制

        // 发布路径话题
        static nav_msgs::Path inspathmsg;
        geometry_msgs::PoseStamped inspose;
        ros::Time nowtime = ros::Time::now();
        
        inspose.header.frame_id = "map";
        inspose.header.stamp = nowtime;
        inspose.pose.position.x = imuPointcg.getX();
        inspose.pose.position.y = imuPointcg.getY();
        inspose.pose.orientation.w = 1;

        inspathmsg.header.frame_id = "map";
        inspathmsg.header.stamp = nowtime;
        inspathmsg.poses.push_back(inspose);

        ROS_INFO("INS输出的位置为:x=%f,y=%f，时间为：%fs",inspose.pose.position.x,inspose.pose.position.y,count * 0.01);

        justInsPathPub.publish(inspathmsg);
    }
    
    count += 1;
    // double imuX;
    // double imuY;
    // imu输出的经纬度被转换成xy。
       // 注意姿态角这里小于0度的话要用360加姿态角
    double yaw360;
    if (imusolver3.avpOut(2)/PI*180.0 < 0 )
    {
        yaw360 = 360 + imusolver3.avpOut(2)/PI*180.0;
    }else
    {
        yaw360 = imusolver3.avpOut(2)/PI*180.0;
    }
    
    if(yaw360 > 180){
        yaw360 = yaw360 - 360 ; 
    }


    ofstream dataFile;
    dataFile.open("indoor3_imu.txt", ofstream::app);
    dataFile << std::fixed << std::setprecision(8); // 设置精度为9位小数
    ros::Time current_time = ros::Time::now();
    double current_time_sec = current_time.toSec();
    dataFile << imusolver3.avpOut(0)/PI*180.0 << " " << imusolver3.avpOut(1)/PI*180.0 << " " << yaw360 << " "
    << imusolver3.avpOut(3) << " " << imusolver3.avpOut(4) << " " << imusolver3.avpOut(5) << " "
    << imuPointcg.getX() << " " << imuPointcg.getY() << " " << 0 << " "
    << current_time_sec << endl;

    // 关闭文档
    dataFile.close();
    

    // 发布纯imu求解的信息话题
    // geometry_msgs::Vector3 attmsg;
    // geometry_msgs::Vector3 velmsg;
    // attmsg.x = kf.sins.att.i / PI * 180;
    // attmsg.y = kf.sins.att.j / PI * 180;
    // attmsg.z = kf.sins.att.k / PI * 180;
    // justInsAttPub.publish(attmsg);
    

    // velmsg.x = kf.sins.vn.i;
    // velmsg.y = kf.sins.vn.j;
    // velmsg.z = kf.sins.vn.k;
    // justInsVelPub.publish(velmsg);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"solveimu");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu",100,imuGet);
    // 测试平面坐标转化代码
    justInsPathPub = nh.advertise<nav_msgs::Path>("/mycar/inspath",100);
    // justInsAttPub = nh.advertise<geometry_msgs::Vector3>("/mycar/insatt",20);
    // justInsVelPub = nh.advertise<geometry_msgs::Vector3>("/mycar/insvel",20);
    //justInsAttPub = nh.advertise<>("",)
    ros::spin();
    return 0;
}