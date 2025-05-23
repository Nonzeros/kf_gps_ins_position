#include "ros/ros.h"
#include "myguide/EXGUIDE.h"
#include "pozyx_simulation/uwb_data.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

UWBanchor a1(1,22.066000,5.463770,0.0);
UWBanchor a2(2,22.072575,-5.359310,0.0);
UWBanchor a3(3,10.189300,5.484553,0.0);
UWBanchor a4(4,10.085800,-5.214020,0.0);
ros::Publisher uwbpub;
nav_msgs::Path uwbPathmsg;
std::vector<double> indoorRange;

void uwbGet(pozyx_simulation::uwb_data uwb_datas){
    static PointCG uwbPointCG;
    EasyUWB uwbsolver;
    std::vector<UWBanchor> anchors;
    std::vector<double> modelPos;
 
    double carrealx;
    double carrealy;
    ros::param::get("carrealx",carrealx);
    ros::param::get("carrealy",carrealy);

    ROS_INFO("运行到uwb");

    // 判断是否在室内，在室内uwb工作
    if(carrealx >= indoorRange[0] && carrealx <= indoorRange[1] && carrealy >= indoorRange[2] && carrealy <= indoorRange[3]){
        a1.distanceWithTag = uwb_datas.distance[0];
        a2.distanceWithTag = uwb_datas.distance[1];
        a3.distanceWithTag = uwb_datas.distance[2];
        a4.distanceWithTag = uwb_datas.distance[3];
        anchors.push_back(a1);anchors.push_back(a2);anchors.push_back(a3);anchors.push_back(a4);
        
        modelPos = uwbsolver.solveModelPos(anchors);
        // uwb路径话题发布
        uwbPathmsg.header.frame_id = "map";
        uwbPathmsg.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped uwbpose;
        uwbpose.header.frame_id = "map";
        uwbpose.header.stamp = ros::Time::now();

        uwbpose.pose.position.x = modelPos[0];
        uwbpose.pose.position.y = modelPos[1];
        uwbpose.pose.orientation.w = 1;

        uwbPathmsg.poses.push_back(uwbpose);
        uwbpub.publish(uwbPathmsg);
        
        // 需要把输出的平面xy转化成经纬度才能放到参数服务器里 单位是deg
        uwbPointCG.setX(modelPos[0]);
        uwbPointCG.setY(modelPos[1]);   
        uwbPointCG.calLLH();     

        std::vector<double> uwbpos;
        std::vector<double> uwbvn;
        uwbpos.push_back(uwbPointCG.getLat());uwbpos.push_back(uwbPointCG.getLon());uwbpos.push_back(0.0);
        uwbvn.push_back(0);uwbvn.push_back(0.0);uwbvn.push_back(0.0);
        
        ros::param::set("std_uwbpos",uwbpos);
        ros::param::set("std_uwbvn",uwbvn);
        // 通过参数服务器发布uwb定位信息，如果uwbflag=1，那就说明信号到达。
        ros::param::set("uwbGetFlag",1);
    }
    
    // ROS_INFO("模型的x=%f,y=%f,z=%f",modelPos[0],modelPos[1],0);
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"uwbposition");
    ros::NodeHandle nh;
    // 读取室内范围
    ros::param::get("indoorrange",indoorRange);

    ros::Subscriber sub = nh.subscribe("/uwb_data_topic",100,uwbGet);
    uwbpub = nh.advertise<nav_msgs::Path>("/uwbpath",100);
    ros::spin();
    return 0;
}
