#include <iostream>
#include <fstream>
using namespace std;
#include "ros/ros.h"

#include "tf/tf.h"
#include "myguide/KFApp.h"
#include "myguide/EXGUIDE.h"
#include "geometry_msgs/Quaternion.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"



void callback(const sensor_msgs::ImuConstPtr& imuData,const sensor_msgs::NavSatFixConstPtr& gpsData,const geometry_msgs::Vector3StampedConstPtr& gpsVData){
    static CKFApp kf(TS);
    static int count = 1;
    static PointCG pointcg(22.799674,113.959796);

    double wx = imuData->angular_velocity.x;
    double wy = imuData->angular_velocity.y;
    double wz = imuData->angular_velocity.z;
    // ROS_INFO("imu的三个方向角速度为：%f,%f,%f",wx,wy,wz);

    double ax = imuData->linear_acceleration.x;
    double ay = imuData->linear_acceleration.y;
    double az = imuData->linear_acceleration.z;
    // ROS_INFO("imu的三个方向加速度为：%f,%f,%f",ax,ay,az);

    double gpslat = gpsData->latitude;
    double gpslon = gpsData->longitude;
    double gpsh = gpsData->altitude;
    double gpsVE = -gpsVData->vector.y;
    double gpsVN = gpsVData->vector.x;
    double gpsVU = gpsVData->vector.z;
    // GPS的位置速度输出，速度为东北天xyz

    double imuX;
    double imuY;
    // imu输出的经纬度被转换成xy。

    double accs[3];
    double gyro[3];
    double gpspos[3];
    double gpsvn[3];
    accs[0] = ax; accs[1] = ay; accs[2] = az;
    gyro[0] = wx; gyro[1] = wy; gyro[2] = wz; 
	gpspos[0] = gpslat; gpspos[1] = gpslon; gpspos[2] = gpsh;
    gpsvn[0] = gpsVE; gpsvn[1] = gpsVN; gpsvn[2] = gpsVU;

    static CVect3 eb;
    static CVect3 db;

    if(count == 1){
        double yaw0 = C360CC180(90.0*glv.deg);  // 北偏东为正
        CVect3 gpspos=LLH(22.799674,113.959796,0), gpsvn=O31;
        kf.Init(CSINS(a2qua(CVect3(0,0,yaw0)), O31, gpspos));  // 请正确初始化方位和位置
        eb = CVect3(0.0,0.0,0.0)*glv.dps;  // 陀螺零偏 deg/s
        eb = CVect3(ax,ay,az);
        // db = 031;
    }
    
	CVect3 wm = (*(CVect3*)gyro*glv.dps-eb)*TS;
    // CVect3 vm = (*(CVect3*)accs*glv.g0-db)*TS;
	CVect3 vm = (*(CVect3*)accs-db)*TS;

    //kf.SetMeasGNSS(*(CVect3*)gpspos, *(CVect3*)gpsvn);
    kf.Update(&wm, &vm, 1, TS);
    pointcg.set(kf.sins.pos.i * 180.0 / PI,kf.sins.pos.j * 180.0 / PI); //输入三角度制

    ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,p1=%f,p2=%f,p3=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    kf.sins.pos.i / PI * 180.0,kf.sins.pos.j / PI * 180.0,kf.sins.pos.k);
    ROS_INFO("计数第%d次，v1=%f,v2=%f,v3=%f,x=%f,y=%f,h=%f",count,kf.sins.vn.i,kf.sins.vn.j,kf.sins.vn.k,
    pointcg.getX(),pointcg.getY(),kf.sins.pos.k);
    count += 1;



}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"solveimu");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh,"/imu",20);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh,"/sensor_msgs/NavSatFix",20);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gps_vsub(nh,"/sensor_msgs/NavSatFix_Velocity",20);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), imu_sub, gps_sub,gps_vsub);  
    sync.registerCallback(boost::bind(&callback, _1, _2,_3));
    // message_filters::TimeSynchronizer<sensor_msgs::Imu,sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped> sync(imu_sub,gps_sub,gps_vsub,10);
    // sync.registerCallback(boost::bind(&callback,_1,_2,_3));

    ros::spin();
    return 0;
}