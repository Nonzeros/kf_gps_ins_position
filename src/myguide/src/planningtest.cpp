#include "ros/ros.h"
#include "myguide/EXGUIDE.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"


ros::Publisher pathpub;
ros::Publisher pointpub;
ros::Publisher goalpointpub;
ros::Publisher startpointpub;
ros::Subscriber mapsub;
ros::Publisher cmdvelpub;

nav_msgs::OccupancyGrid mymap;

void getGoalPoint(geometry_msgs::PoseStamped goalPoint){

    // 规划出全局路径
    clock_t start_time=clock();
    static int count = 1;
    RRT rrt;
    RRTMap rrtmap;
    RRTNode beginNode(0.0,0.0);
    RRTNode goalNode(goalPoint.pose.position.x,goalPoint.pose.position.y);
    std::vector<RRTNode> mypathnodes;
    int solvemaxtimes = 500000;
    rrtmap.datas = mymap.data;
    rrtmap.height = mymap.info.height;
    rrtmap.width = mymap.info.width;
    
    rrt.init(rrtmap,beginNode,goalNode,1.5,pointpub,goalpointpub,startpointpub);
    mypathnodes = rrt.solve(solvemaxtimes);
    clock_t end_time=clock();
    std::cout << "The run time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

    static nav_msgs::Path rrtpathmsg;
    ros::Time nowtime = ros::Time::now();
    rrtpathmsg.header.frame_id = "map";
    rrtpathmsg.header.stamp = nowtime;

    geometry_msgs::PoseStamped rrtpose;
    rrtpose.header.frame_id = "map";
    for (int i = mypathnodes.size() -1 ; i>=0 ; i--)
    {
        rrtpose.header.stamp = ros::Time::now();
        rrtpose.pose.position.x = mypathnodes[i].x;
        rrtpose.pose.position.y = mypathnodes[i].y;
        rrtpose.pose.orientation.w = 1;
        rrtpathmsg.poses.push_back(rrtpose);
    } 
    pathpub.publish(rrtpathmsg);

    // 简单的路径规划跟踪方法

    /*
        1、拿到路径点数组和小车当前位置
        2、因为走的是平面直线，只需要跟踪当前姿态角和小车目前位置到目标点的角度
        3、如果姿态角不满足条件，则控制cmd_vel旋转；如果满足条件，则控制cmd_vel匀速行驶，到达目标点则控制cmd_vel为0，重复动作
    */
    int pointNums = mypathnodes.size();
    geometry_msgs::Quaternion q;
    double realYaw;
    double hopeYaw;

    for (int i = pointNums-2; i > -1; i--)
    {
        //1、拿到路径点数组和小车当前位置
        std::vector<double> gnssInsResult;
        ros::param::get("gnssinsresult",gnssInsResult);
        geometry_msgs::Twist cmdvelMsg;

        //2、因为走的是平面直线，只需要跟踪当前姿态角和小车目前位置到目标点的角度
        //2.1 计算希望的姿态角
        realYaw = gnssInsResult[2];
        double a1 = atan2(-gnssInsResult[7] + mypathnodes[i].y, -gnssInsResult[6] + mypathnodes[i].x);
        if (a1 >= -PI && a1 <= -PI/2)
        {
            hopeYaw = -3*PI/2 - a1; 
        }else{
            hopeYaw = PI/2 - atan2(-gnssInsResult[7] + mypathnodes[i].y, -gnssInsResult[6] + mypathnodes[i].x);
        }
        
        // 首先控制其转到指定角度
        while (abs(hopeYaw - realYaw) >= 0.001)
        {   
            cmdvelMsg.angular.z = 0.35;
            cmdvelpub.publish(cmdvelMsg);
            ros::param::get("gnssinsresult",gnssInsResult);
            realYaw = gnssInsResult[2];
            ros::param::set("/mycar/realyaw",realYaw);
            ros::param::set("/mycar/hopeyaw",hopeYaw);
        }
        cmdvelMsg.angular.z = 0;
        cmdvelpub.publish(cmdvelMsg);


        // 然后控制线速度
        ros::param::get("gnssinsresult",gnssInsResult);
        while ( sqrt(pow(gnssInsResult[7]-mypathnodes[i].y,2) + pow(gnssInsResult[6]-mypathnodes[i].x,2)) >= 0.05)
        {
            cmdvelMsg.linear.x = 0.1;
            cmdvelpub.publish(cmdvelMsg);
            ros::param::get("gnssinsresult",gnssInsResult);
            ROS_INFO("test");
        }
        cmdvelMsg.linear.x = 0;
        cmdvelpub.publish(cmdvelMsg);
    
}
    
    



}

void doMap(nav_msgs::OccupancyGrid mymap1){
    mymap = mymap1;
    // clock_t start_time=clock();
    // static int count = 1;
    //     RRT rrt;
    //     RRTMap rrtmap;
    //     RRTNode beginNode(0.0,0.0);
    //     RRTNode goalNode(2.0,3.8);
    //     std::vector<RRTNode> mypathnodes;
    //     int solvemaxtimes = 99999;
    //     rrtmap.datas = mymap.data;
    //     rrtmap.height = mymap.info.height;
    //     rrtmap.width = mymap.info.width;
        
    //     rrt.init(rrtmap,beginNode,goalNode,0.3,pointpub,goalpointpub,startpointpub);
    //     mypathnodes = rrt.solve(solvemaxtimes);
    //     clock_t end_time=clock();
    //     std::cout << "The run time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

    //     static nav_msgs::Path rrtpathmsg;
    //     ros::Time nowtime = ros::Time::now();
    //     rrtpathmsg.header.frame_id = "map";
    //     rrtpathmsg.header.stamp = nowtime;

    //     geometry_msgs::PoseStamped rrtpose;
    //     rrtpose.header.frame_id = "map";
    //     for (int i = mypathnodes.size() -1 ; i>=0 ; i--)
    //     {
    //         rrtpose.header.stamp = ros::Time::now();
    //         rrtpose.pose.position.x = mypathnodes[i].x;
    //         rrtpose.pose.position.y = mypathnodes[i].y;
    //         rrtpose.pose.orientation.w = 1;
    //         rrtpathmsg.poses.push_back(rrtpose);
    //     } 
    //     pathpub.publish(rrtpathmsg);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"getposition");
    ros::NodeHandle nh;
    pointpub = nh.advertise<geometry_msgs::PointStamped>("/addpoint",10000);
    goalpointpub = nh.advertise<geometry_msgs::PointStamped>("/goalpoint",1);
    startpointpub = nh.advertise<geometry_msgs::PointStamped>("/startpoint",1);

    mapsub = nh.subscribe("/map",2,doMap);
    ros::Subscriber goalpointsub = nh.subscribe("/move_base_simple/goal",10,getGoalPoint);
    pathpub = nh.advertise<nav_msgs::Path>("/rrtpath",1);
    cmdvelpub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",2);
    ros::spin();

    return 0;
}







 // count++;
    // 获取地图的宽度和高度
    // int width = mymap->info.width;
    // int height = mymap->info.height;

    // // 获取地图的分辨率
    // float resolution = mymap->info.resolution;

    // // 获取地图原点的坐标
    // float origin_x = mymap->info.origin.position.x;
    // float origin_y = mymap->info.origin.position.y;

    // // 遍历地图数据
    // for (int y = 0; y < height; ++y) {
    //     for (int x = 0; x < width; ++x) {
    //         // 获取当前单元格的值（-1表示未知，0表示空闲，100表示占据）
    //         int value = mymap->data[x + y * width];

    //         // 如果当前单元格是占据状态，将其坐标发布出去
    //         if (value == 100) {
    //             // 计算当前单元格的坐标
    //             float x_coord = origin_x + (x + 0.5) * resolution;
    //             float y_coord = origin_y + (y + 0.5) * resolution;

    //             // TODO: 发布坐标信息到新话题
    //             ROS_INFO("Occupied pixel at (%f, %f)", x_coord, y_coord);
    //         }
    //     }
    // }