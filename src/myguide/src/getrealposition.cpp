#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include <iostream>
#include <string>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <fstream>
using namespace std;

static ros::Publisher velPub;
static ros::Publisher realPosPathPub;
static ros::Publisher euler_pub;
//ros::Time nowtime;

void doModel(gazebo_msgs::ModelStates state){
    // nowtime = ros::Time::now();
    static string str = "mycar";
    static int index;
    static int realPathCount = 1;
    geometry_msgs::Pose realPose;
    geometry_msgs::Twist realVel;
    geometry_msgs::Vector3 realVelLin;
    geometry_msgs::Vector3 euler_angles;

    // 每20次获得位置发布一次
    if (realPathCount % 10 == 0)
    {
            for(int i = 0; i < 6 ;i++){
            if(str.compare(state.name[i]) == 0){
                index = i;
                break;
            }
        }
    realPose = state.pose[index];
    realVel = state.twist[index];
    double realX = realPose.position.x;
    double realY = realPose.position.y;
    double realZ = realPose.position.z;
    double realVX = realVel.linear.x;
    double realVY = realVel.linear.y;
    double realVZ = realVel.linear.z;

    static nav_msgs::Path realpathmsg;
    realpathmsg.header.frame_id = "map";
    ros::Time nowtime = ros::Time::now();
    realpathmsg.header.stamp = nowtime;

    geometry_msgs::PoseStamped realpathpoint;

    realpathpoint.pose.position.x = realX;
    realpathpoint.pose.position.y = realY;
    realpathpoint.pose.position.z = realZ;
    realpathpoint.pose.orientation.x = realPose.orientation.x;
    realpathpoint.pose.orientation.y = realPose.orientation.y;
    realpathpoint.pose.orientation.z = realPose.orientation.z;
    realpathpoint.pose.orientation.w = realPose.orientation.w;
    
    // 姿态四元数转欧拉角
    tf2::Quaternion quat;
    quat.setX(realPose.orientation.x);
    quat.setY(realPose.orientation.y);
    quat.setZ(realPose.orientation.z);
    quat.setW(realPose.orientation.w);

    // 发布真实欧拉角话题
    tf2::Matrix3x3(quat).getRPY(euler_angles.x, euler_angles.y, euler_angles.z);
    euler_angles.z = -(euler_angles.z * 180 / 3.1415926535) + 90; //调整到北偏东为正的地理坐标系下
    if (euler_angles.z > 180)
    {
        euler_angles.z = euler_angles.z - 360;
    }
    
    
    euler_pub.publish(euler_angles);

    realpathpoint.header.stamp = nowtime;
    realpathpoint.header.frame_id = "map";
    realpathmsg.poses.push_back(realpathpoint);

    // 发布小车真实位置话题
    realPosPathPub.publish(realpathmsg);
    // 改成0 测试用
    // ros::param::set("carrealx",0);
    // ros::param::set("carrealy",0);

    ros::param::set("carrealx",realX);
    ros::param::set("carrealy",realY);

    realVelLin.x = realVX;
    realVelLin.y = realVY;
    realVelLin.z = realVZ;
    // ROS_INFO("模型名字：%s",state.name[index].c_str());
    // ROS_INFO("小车的真正xyz位置为：%f,%f,%f",realX,realY,realZ);
    // 发布小车真实速度话题
    velPub.publish(realVelLin);
    
    // 将真实avp写入txt中
    ofstream dataFile;
    dataFile.open("indoor_outdoor_real1.txt", ofstream::app);
    ros::Time current_time = ros::Time::now();
    double current_time_sec = current_time.toSec();

    dataFile << std::fixed << std::setprecision(8); // 设置精度为9位小数
    // 朝TXT文档中写入数据（输出的姿态都是角度制，z是之前就已经换成角度了）
    dataFile << euler_angles.x/3.1415926 * 180 << " " << euler_angles.y/3.1415926 * 180 << " " << euler_angles.z << " "
    << realVX << " " << realVY << " "  << realVZ << " "
    << realX << " " << realY << " " << realZ << " "
    << current_time_sec << endl;
    ROS_INFO("真实x为%f",realX);
    ROS_INFO("真实y为%f",realY);
    // 关闭文档
    dataFile.close();

    }
    realPathCount += 1;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"getposition");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",100,doModel);
    velPub = nh.advertise<geometry_msgs::Vector3>("/mycar/realvellin",100);
    realPosPathPub = nh.advertise<nav_msgs::Path>("/mycar/realPath",100);
    euler_pub = nh.advertise<geometry_msgs::Vector3>("/mycar/realEuler", 100);
    ros::spin();

    return 0;
}
