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
ros::Publisher justGpsPathPub;
static int gpsOutputFlag = 0;
std::vector<double> indoorRange; // 设置室内环境范围第1到第4个元素分别为-x,x,-y,y

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

    // 发布gps路径话题
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
    
    
    // 小车的真实位置
    // double carrealx = 0;
    // double carrealy = 0;

    // if(gpsCount >= 3){
    //     // 求解两次再判断，避免那边还没上传数据到参数服务器
    //     ros::param::get("carrealx",carrealx);
    //     ros::param::get("carrealy",carrealy);
    // }
    if(pointcg.getX() >= indoorRange[0] && pointcg.getX() <= indoorRange[1] && pointcg.getY() >= indoorRange[2] && pointcg.getY() <= indoorRange[3]){
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

void gpsVGet(geometry_msgs::Vector3Stamped gpsVmsg){
    std::vector<double> gpsvel;
    // hector的gps速度是北西天
    gpsvel.push_back(-gpsVmsg.vector.y);
    gpsvel.push_back(gpsVmsg.vector.x);
    gpsvel.push_back(gpsVmsg.vector.z);
    ros::param::set("std_gpsvel",gpsvel);
}

void imuGet(sensor_msgs::Imu msg){
    static int count = 1; //imu求解次数
    static int gpsCount = 0;
    static int uwbCount = 0;
    static PointCG pointcg(22.799674,113.959796);
    static PointCG pointcg2(22.799674,113.959796); // 创建一个新的pointcg2对象，将平面xy转成经纬度
    static double imuBias[6] = {0.0}; // 零偏数组

    double wx = msg.angular_velocity.x;
    double wy = msg.angular_velocity.y;
    double wz = msg.angular_velocity.z;
    // ROS_INFO("imu的三个方向角速度为：%f,%f,%f",wx,wy,wz);

    double ax = msg.linear_acceleration.x;
    double ay = msg.linear_acceleration.y;
    double az = msg.linear_acceleration.z;
    // ROS_INFO("imu的三个方向加速度为：%f,%f,%f",ax,ay,az);
    
    // 减去零偏
    wx = wx - imuBias[0];wy = wy - imuBias[1];wy = wy - imuBias[2];
    ax = ax - imuBias[3];ay = ay - imuBias[4];az = az - imuBias[5];
    // ROS_INFO("陀螺零偏x:%f,y:%f,z:%f，加速度计零偏:x:%f,y:%f,z:%f",imuBias[0],imuBias[1],imuBias[2],imuBias[3],imuBias[4],imuBias[5]);

    // 角速度，加速度数组
    double accs[3];
    double gyro[3];
    accs[0] = ax; accs[1] = ay; accs[2] = az;
    gyro[0] = wx; gyro[1] = wy; gyro[2] = wz; 
	
    // 用于放到参数服务器的vector变量
    std::vector<double> std_gpspos;
    std::vector<double> std_gpsvn;
    std::vector<double> std_uwbpos;
    std::vector<double> std_uwbvn;
    
    double gpspos_middle[3];
    double gpsvn_middle[3];
    double uwbpos_middle[3];
    double uwbvn_middle[3];
    
    // 判断是否在室内，在室内为1，否则为0
    static int isIndoors = 0;

    // 如果真实位置在房间内的范围，则没有gps数据，变为uwb数据
    // if(carrealx >= indoorRange[0] && carrealx <= indoorRange[1] && carrealy >= indoorRange[2] && carrealy <= indoorRange[3]){
    //     isIndoors = 1;
    // }else{
    //     isIndoors = 0;
    // }

    

    // imu求解器和eskf求解器定义
    static Matrix62d wvm;
    static double imuT = 0.01;
    static ImuSolver imusolver;
    static ESKFSolver eskfSolver; // imu/gps组合导航滤波器
    static ESKFSolver eskfSolver2;// imu/uwb组合导航滤波器
    static MatrixXd uwbdatas,gpsdatas;
    uwbdatas.resize(6,1);
    gpsdatas.resize(6,1);
    Vector10d avp00;

    // 组合导航结果在房间范围内，则没有gps数据，变为uwb数据
    if(pointcg.getX() >= indoorRange[0] && pointcg.getX() <= indoorRange[1] && pointcg.getY() >= indoorRange[2] && pointcg.getY() <= indoorRange[3]){
        isIndoors = 1;
    }else{
        isIndoors = 0;
    }

    // 滤波器和位置初始化
    if(count == 1){
        double yaw0 = 89.95/180.0*PI;
        // eskfSolver.init(1000,49000,0.000005,3,0.02); // 参数4：1为6状态观测，2是只有速度观测，3是只有位置观测
        // eskfSolver.init(10,50000,0.00001,3,0.02); 0.1v适合参数 3位置 
        Vector6d q1,r1,q2,r2;
        q1 << 5,5,5,5,5,5;
        r1 << 5,5,5,5,5,5;
        // q2 << 5000,5000,5000,5000,5000,5000;
        // r2 << 0.00005,0.1,0,pow(10,-8) * 5,pow(10,-8) * 5,pow(10,-8) * 5;
        // r2 << 0,0,0,0,0,0;
        q2 << 5000,5000,5000,5000,5000,5000;
        r2 << 0.00005,0.1,0,pow(10,-7) * 5,pow(10,-7) * 5,pow(10,-7) * 5;

        // q2 << 55000,55000,55000,55000,55000,0;
        // r2 << 0.00005,0.1,0,0.000001,0.000001,0;

        eskfSolver.init(0.005,q1,r1,1,0.02);  //0.2位置噪声 0.001速度gps噪声  0.001 imu噪声适合的参数  6观测

        // eskfSolver2.init(1000,q2,r2,3,0.02);
        eskfSolver2.init(0.001,q2,r2,3,0.02);
        // eskfSolver.init(0.1,50,5,2,0.02);  0.2位置噪声 0.001速度gps噪声  0.001 imu噪声适合的参数  3速度
        avp00 << 0,0,yaw0,0,0,0,22.799674/180.0*PI,113.959796/180*PI,0,0;
    } 
	
    // 先是sins求解(如果counts是2的倍数，那就sins求解)
      // 接收惯导数据求解，每两次求解一次
    if (count % 2 != 0 )
    {
        // 奇数次获得imu输出不求解
        wvm(0,0) = wx * imuT; wvm(1,0) = wy * imuT; wvm(2,0) = wz * imuT;
        wvm(3,0) = ax * imuT; wvm(4,0) = ay * imuT; wvm(5,0) = az * imuT;
    }else{
        wvm(0,1) = wx * imuT; wvm(1,1) = wy * imuT; wvm(2,1) = wz * imuT;
        wvm(3,1) = ax * imuT; wvm(4,1) = ay * imuT; wvm(5,1) = az * imuT;
        imusolver.sinsSolve(wvm,avp00,2*imuT);

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
                
                // uwb/gps数据赋值
                gpsdatas(0,0) = std_gpspos[0] / 180 * PI; gpsdatas(1,0) = std_gpspos[1]  / 180 * PI; gpsdatas(2,0) = std_gpspos[2];
                // uwbdatas(2,0) = std_gpspos[2];
                gpsdatas(3,0) = std_gpsvn[0]; gpsdatas(4,0) = std_gpsvn[1]; gpsdatas(5,0) = std_gpsvn[2];
               
                // 滤波求解
                Vector10d avp2_bar;
                avp2_bar =  eskfSolver.solveAvpBar(gpsdatas,wvm,imusolver.avp1,imusolver.avp2,imuBias);
                // 更新imu求解器里的avp2
                imusolver.avp1 = avp2_bar;
                // imusolver.avp1(2,0) = -imusolver.avp2(2,0);
                imusolver.avpOut = avp2_bar;
                imusolver.avpOut(2,0) = -imusolver.avpOut(2,0);
            }
        }else{
            // 在室内，有imu/uwb组合导航
            int uwbGetFlag;
            ros::param::get("uwbGetFlag",uwbGetFlag);
            if (uwbGetFlag == 1)
            {
                uwbGetFlag = 0;
                // if (uwbCount == 0)
                // {
                //     /* code */
                // }
                
                ros::param::get("std_uwbpos",std_uwbpos);
                ros::param::get("std_uwbvn",std_uwbvn);

                // uwb_gps数据赋值
                uwbdatas(0,0) = std_uwbpos[0] / 180 * PI; uwbdatas(1,0) = std_uwbpos[1] / 180 * PI; uwbdatas(2,0) = 0;
                // uwbdatas(2,0) = std_gpspos[2];
                uwbdatas(3,0) = std_uwbvn[0]; uwbdatas(4,0) = std_uwbvn[1]; uwbdatas(5,0) = std_uwbvn[2];
                
                if (pointcg.getX() > 10){
                    ROS_INFO("test");
                }

                // 滤波求解
                Vector10d avp2_bar;
                avp2_bar =  eskfSolver2.solveAvpBar(uwbdatas,wvm,imusolver.avp1,imusolver.avp2,imuBias);
                // 更新imu求解器里的avp2
                imusolver.avp1 = avp2_bar;
                // imusolver.avp1(2,0) = -imusolver.avp2(2,0);
                imusolver.avpOut = avp2_bar;
                imusolver.avpOut(2,0) = -imusolver.avpOut(2,0);
            }
    }


    // 求解以后发布话题
    pointcg.set(imusolver.avpOut(6)/PI*180.0,imusolver.avpOut(7)/PI*180.0);

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
    // ROS_INFO("路径的x为%f",mypose.pose.position.x);
    // ROS_INFO("路径的y为%f",mypose.pose.position.y);
    mypose.pose.position.z = 0;
    mypose.pose.orientation.w = 0;
    gpsInspathmsg.poses.push_back(mypose);

    gpsInsPathPub.publish(gpsInspathmsg);

    // if(pointcg.getX() > 12){
    //     ROS_INFO("11");
    // }
    // 将组合导航结果输出为txt
    // 注意姿态角这里小于0度的话要用360加姿态角
    double yaw360;
    if (imusolver.avpOut(2)/PI*180.0 < 0 )
    {
        yaw360 = 360 + imusolver.avpOut(2)/PI*180.0;
    }else
    {
        yaw360 = imusolver.avpOut(2)/PI*180.0;
    }
    
    if(yaw360 > 180){
        yaw360 = yaw360 - 360 ; 
    }

    ofstream dataFile;
    dataFile.open("indoor_outdoor_eskf1.txt", ofstream::app);
    dataFile << std::fixed << std::setprecision(8); // 设置精度为9位小数
    ros::Time current_time = ros::Time::now();
    double current_time_sec = current_time.toSec();
    dataFile << imusolver.avpOut(0)/PI*180.0 << " " << imusolver.avpOut(1)/PI*180.0 << " " << yaw360 << " "
    << imusolver.avpOut(3) << " " << imusolver.avpOut(4) << " " << imusolver.avpOut(5) << " "
    << mypose.pose.position.x << " " << mypose.pose.position.y << " " << imusolver.avpOut(8) << " "
    << current_time_sec << " " << isIndoors << endl;
    ROS_INFO("文件的x为%f,是否在室内%d,单独启动版本",pointcg.getX(),isIndoors);

    // ROS_INFO("文件的y为%f",pointcg.getY());
    // 关闭文档
    dataFile.close();

    // 将imu输出
    ofstream dataFile2;
    dataFile2.open("imuOutput_indoor.txt", ofstream::app);
    dataFile2 << std::fixed << std::setprecision(8); // 设置精度为8位小数

    dataFile2 << wx << " " << wy << " " << wz << " " 
    << ax << " " << ay << " " << az << " " 
    << current_time_sec << endl;

    // 关闭文档
    dataFile2.close();


    // 把组合导航结果(小车的avp放到参数服务器，姿态用弧度表示，位置用平面xy)
    std::vector<double> gnssInsResult;
    gnssInsResult.push_back(imusolver.avpOut(0)/PI*180.0);gnssInsResult.push_back(imusolver.avpOut(1)/PI*180.0);gnssInsResult.push_back(imusolver.avpOut(2)/PI*180.0);//姿态加个负号，可能代码有问题
    gnssInsResult.push_back(imusolver.avpOut(3));gnssInsResult.push_back(imusolver.avpOut(4));gnssInsResult.push_back(imusolver.avpOut(5));
    gnssInsResult.push_back(pointcg.getX());gnssInsResult.push_back(pointcg.getY());gnssInsResult.push_back(imusolver.avpOut(8));
    gnssInsResult.push_back(imusolver.avpOut(9));
    ros::param::set("gnssinsresult",gnssInsResult);
        
    }
    
    count += 1;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"eskf");
    ros::NodeHandle nh;
    // 设置室内范围
    indoorRange.push_back(6.5);indoorRange.push_back(23.1342);indoorRange.push_back(-20.55);indoorRange.push_back(10.55); // -x,x,-y,y
    ros::param::set("indoorrange",indoorRange);

    ros::Subscriber sub = nh.subscribe("/imu",100,imuGet);
    ros::Subscriber gpsSub = nh.subscribe("/sensor_msgs/NavSatFix",100,gpsGet);
    ros::Subscriber gpsVSub = nh.subscribe("/sensor_msgs/NavSatFix_Velocity",100,gpsVGet);
    gpsInsPathPub = nh.advertise<nav_msgs::Path>("/mycar/gpsinspath",100);
    gpsInsAttPub = nh.advertise<geometry_msgs::Vector3>("/mycar/gpsinsatt",100);
    justGpsPathPub = nh.advertise<nav_msgs::Path>("/mycar/gpspath",100);
    ros::spin();
    return 0;
}