#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// 三更新需要用到的子函数
Matrix3d eur2mat(Vector3d eur);
Matrix3d getAnt(Vector3d phi);
Matrix3d getMRV(Vector3d phi);
Vector3d getOmegaie(MatrixXd avp1);
Vector3d getOmegaen(MatrixXd avp1);
Vector3d getG(MatrixXd avp1);
Vector3d mat2eur(Matrix3d mat);

// sins求解
// imu为 x y z角增量 xyz速度增量
MatrixXd sinsSolve(MatrixXd avp0,MatrixXd avp1,MatrixXd imuData);

// 三更新函数
// 姿态不需要k-2时刻的数据
Vector3d solAttIte(MatrixXd avp1,MatrixXd imuData,double deltaT);
Vector3d solVelIte(MatrixXd avp0,MatrixXd avp1,MatrixXd imuData,double deltaT);
Vector3d solVelFir(MatrixXd avp1,MatrixXd imuData,double deltaT);
Vector3d solPosIte(MatrixXd avp0,MatrixXd avp1,MatrixXd imuData,Vector3d vel2,double deltaT);
Vector3d solPosFir(MatrixXd avp1,MatrixXd imuData,Vector3d vel2,double deltaT);

void imuGet(sensor_msgs::Imu msg){
    static int sampCount = 0;
    static int solCount = 0;
    static MatrixXd imuDatas(2,6);
    static MatrixXd avp0(1,9);
    static MatrixXd avp1(1,9);
    static MatrixXd avp2(1,9);
    static MatrixXd avpInit(1,9);

    avpInit <<0,0,0,0,0,0,30.0 / 180.0 * 3.1415926, 130.0 / 180.0 * 3.1415926,0;
    avp0 = avpInit;
    avp1 = avpInit;

    double deltaTSamp = 0.01; // 一次采样用时
    sampCount += 1;

    double wx = msg.angular_velocity.x;
    double wy = msg.angular_velocity.y;
    double wz = msg.angular_velocity.z;
    // ROS_INFO("imu的三个方向角速度为：%f,%f,%f",wx,wy,wz);

    double ax = msg.linear_acceleration.x;
    double ay = msg.linear_acceleration.y;
    double az = msg.linear_acceleration.z;
    // ROS_INFO("imu的三个方向加速度为：%f,%f,%f",ax,ay,az);

    // imu数据存入
    imuDatas(sampCount%2,0) = wx;
    imuDatas(sampCount%2,1) = wy;
    imuDatas(sampCount%2,2) = wz;
    imuDatas(sampCount%2,3) = ax;
    imuDatas(sampCount%2,4) = ay;
    imuDatas(sampCount%2,5) = az;

    if ( (sampCount % 2) == 1)
    {
        //对2取余数为1，说明两次采样，进行一次解算，imu输出要乘上时间才得增量。
        avp2 = sinsSolve(avp0,avp1,imuDatas * deltaTSamp);
        avp0 = avp1;
        avp1 = avp2;
        solCount += 1;
        ROS_INFO("在时刻%f的imu输出：vx:%f,vy:%f,vz:%f",solCount*2*deltaTSamp,avp2(3),avp2(4),avp2(5));
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //基本步骤
    ros::init(argc,argv,"solveimu");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu",10000,imuGet);
    ros::spin();
    return 0;
}


// sins求解
MatrixXd sinsSolve(MatrixXd avp0,MatrixXd avp1,MatrixXd imuData){
    static int count = 1;
    static double deltaT = 2 * 0.01;
    MatrixXd avp2(1,9);
    Vector3d att2(1,3);
    Vector3d vel2(1,3);
    Vector3d pos2(1,3);

    att2 = solAttIte(avp1,imuData,deltaT);

    if (count == 1){
        //初次解算
        vel2 = solVelFir(avp1,imuData,deltaT);
        pos2 = solPosFir(avp1,imuData,vel2,deltaT);
        //接下来解算
    }else{
        vel2 = solVelIte(avp0,avp1,imuData,deltaT);
        pos2 = solPosIte(avp0,avp1,imuData,vel2,deltaT);
    }   
    for (int j = 0; j < 9; j++)
    {
        if (j<3)
        {
            avp2(0,j) = att2(j);
        }
        else if (j<6)
        {
            avp2(0,j) = vel2(j-3);
        }
        else{
            avp2(0,j) = pos2(j-6);
        }   
    }
    count += 1;
    return avp2;
}

// 三更新子函数实现
Matrix3d eur2mat(Vector3d eur){
    Matrix3d c1(3,3);
    Matrix3d c2(3,3);
    Matrix3d c3(3,3);
    c1 << 1,0,0,
        0,cos(eur(0)),sin(eur(0)),
        0,-sin(eur(0)),cos(eur(0));

    c2 << cos(eur(1)),0,-sin(eur(1)),
        0,1,0,
        sin(eur(1)),0,cos(eur(1));

    c3 << cos(eur(2)),-sin(eur(2)),0,
        sin(eur(2)),cos(eur(2)),0,
        0,0,1;
    Matrix3d Mat(3,3);
    Mat = (c2 * c1 * c3).transpose();
    return Mat;
}
Matrix3d getAnt(Vector3d phi){
    Matrix3d res(3,3);
    res << 0,-phi(2),phi(1),
        phi(2),0,-phi(0),
        -phi(1),phi(0),0;
    return res;
}
Matrix3d getMRV(Vector3d phi){
    Matrix3d eye3(3,3);
    Matrix3d res(3,3);
    Matrix3d phiAnt;
    phiAnt = getAnt(phi);
    eye3.setIdentity();
    double phiNorm = phi.norm();
    res = eye3 + sin(phiNorm) / phiNorm *  phiAnt + ( 1 - cos(phiNorm) ) / (phiNorm * phiNorm) * phiAnt * phiAnt;
    return res;
}
Vector3d getOmegaie(MatrixXd avp1){
    double L = avp1(6);
    double omegaieNorm = 7.292115 * pow(10,-5);
    Vector3d omegaie;
    omegaie << 0, omegaieNorm * cos(L),omegaieNorm * sin(L);
    return omegaie;
}
Vector3d getOmegaen(MatrixXd avp1){
    double L = avp1(6);
    double h = avp1(8);
    Vector3d omegaen;
    double e2 = 2.0 / 298.3 - 1 / 298.3 / 298.3; 
    double re = 6378254.0;
    double rn = re / sqrt(1 - e2 * sin(L) * sin(L));
    double rm = rn * (1 - e2) / (1 - e2 * sin(L) * sin(L));
    double vn = avp1(4);
    double ve = avp1(3);
    omegaen << -vn / (rm + h),ve / (rn + h), ve * tan(L) / (rn + h);

    return omegaen;
}
Vector3d mat2eur(Matrix3d mat){
    // 欧拉角单位为rad
    Vector3d eur;
    double theta = asin(mat(2,1));
    double gamma = atan2(-mat(2,0), mat(2,2));
    double psi = atan2(-mat(0,1),mat(1,1));
    eur << theta,gamma,psi;
    return eur;   
}
Vector3d getG(MatrixXd avp1){
    Vector3d g;
    double g0 = 9.780325333434361;
    double L = avp1(6);
    double h = avp1(8);
    g << 0,0, -(g0 * (1 + 0.00527094 * sin(L) * sin(L) + 0.0000232718 * pow(sin(L),4)) - 0.000003086 * h ) ;
    return g;
}


Vector3d solAttIte(MatrixXd avp1,MatrixXd imuData,double deltaT){
    Vector3d omegaie;
    Vector3d omegaen;
    Vector3d omegain;
    omegaie = getOmegaie(avp1);
    omegaen = getOmegaen(avp1);
    omegain = omegaie + omegaen;

    Matrix3d Cnn(3,3);
    Matrix3d Cbb(3,3);
    MatrixXd Cnb1(3,3);
    Vector3d phiib(3,1);
    Vector3d theta1 = imuData.row(0).head(3);
    Vector3d theta2 = imuData.row(1).head(3);
    Vector3d att1 = avp1.row(0).head(3);
    Matrix3d Cnb2(3,3);
    Vector3d att2;

    Cnn = getMRV(deltaT * omegain).transpose();
    phiib = (theta1 + theta2) + 2.0/3.0 *  theta1.cross(theta2);
    Cbb = getMRV(phiib);
    Cnb1 = eur2mat(att1);

    Cnb2 = Cnn * Cnb1 * Cbb;

    att2 = mat2eur(Cnb2);
    return att2;
}
Vector3d solVelIte(MatrixXd avp0,MatrixXd avp1,MatrixXd imuData,double deltaT){
    Vector3d theta1 = imuData.block(0,0,1,3).transpose();
    Vector3d theta2 = imuData.block(1,0,1,3).transpose();
    Vector3d v1 = imuData.row(0).segment(3,3).transpose();
    Vector3d v2 = imuData.row(1).segment(3,3).transpose();
    Vector3d vbrot = 0.5 * (theta1 + theta2).cross(v1 + v2);
    Vector3d vbscul = 2.0 / 3.0 * ( theta1.cross(v2) + v1.cross(theta2)  );
    
    MatrixXd eye3(3,3);
    eye3.setIdentity();
    Vector3d omegainAver(3,1);
    omegainAver = 1.5 * ( getOmegaie(avp1) + getOmegaen(avp1) ) - 0.5 * ( getOmegaie(avp0) + getOmegaen(avp0) );
    MatrixXd Cnb1(3,3);
    Cnb1 = eur2mat(avp1.row(0).head(3));

    Vector3d vsf(3,1);
    vsf = ( eye3 - 0.5 * deltaT * getAnt(omegainAver) ) * Cnb1 * ( (v1 + v2 + vbrot + vbscul) );
    Vector3d vcorg(3,1);
    Vector3d vaver = ( -0.5 * avp0.block(0,3,1,3) + 1.5 * avp1.block(0,3,1,3) ).transpose(); 
    Vector3d gaver = ( -0.5 * getG(avp0) + 1.5 * getG(avp1) ) ;
    vcorg = (-( 2.0 * (-0.5 * getOmegaie(avp0) + 1.5 * getOmegaie(avp1))  + (-0.5 * getOmegaen(avp0) + 1.5 * getOmegaen(avp1))).cross(vaver) + gaver ) * deltaT;
    Vector3d res =  (vsf + vcorg).transpose() + avp1.block(0,3,1,3); 
    return res;
}
Vector3d solVelFir(MatrixXd avp1,MatrixXd imuData,double deltaT){
    // Vector3d theta1 = imuData.row(0).head(3).transpose();
    // Vector3d theta2 = imuData.row(1).head(3).transpose();
    Vector3d theta1 = imuData.block(0,0,1,3).transpose();
    Vector3d theta2 = imuData.block(1,0,1,3).transpose();
    Vector3d v1 = imuData.row(0).segment(3,3).transpose();
    Vector3d v2 = imuData.row(1).segment(3,3).transpose();
    Vector3d vbrot = 0.5 * (theta1 + theta2).cross(v1 + v2);
    
    Vector3d vbscul = 2.0 / 3.0 * ( theta1.cross(v2) + v1.cross(theta2)  );
    
    MatrixXd eye3(3,3);
    eye3.setIdentity();
    Vector3d omegainAver(3,1);
    omegainAver = ( getOmegaie(avp1) + getOmegaen(avp1) );
    MatrixXd Cnb1(3,3);
    Cnb1 = eur2mat(avp1.row(0).head(3));

    Vector3d vsf(3,1);
    vsf = ( eye3 - 0.5 * deltaT * getAnt(omegainAver) ) * Cnb1 * ( (v1 + v2 + vbrot + vbscul) );
    Vector3d vcorg(3,1);
    Vector3d vaver = avp1.block(0,3,1,3).transpose(); 
    Vector3d gaver = getG(avp1);

    vcorg = (-( 2.0 * getOmegaie(avp1) + getOmegaen(avp1) ).cross(vaver) + gaver)  * deltaT;
    Vector3d res =  (vsf + vcorg).transpose() + avp1.block(0,3,1,3); 
    return res;
}
Vector3d solPosIte(MatrixXd avp0,MatrixXd avp1,MatrixXd imuData,Vector3d vel2,double deltaT){
    // 要用到现在的速度
    double hAver =  (-0.5 * avp0(8) + 1.5 * avp1(8));
    double LAver = (-0.5 * avp0(6) + 1.5 * avp1(6));
    double re = 6378254.0;
    double e2 = 2.0 / 298.3 - 1.0 / 298.3 / 298.3;
    double rnAver = re / sqrt(1 - e2 * sin(LAver) * sin(LAver));
    double rmAver = rnAver * (1 - e2) / (1 - e2 * sin(LAver) * sin(LAver));
    double rnhAver = rnAver + hAver;
    double rmhAver = rmAver + hAver;
    MatrixXd mpv(3,3);
    Vector3d p2;
    if ( abs( cos(LAver / rnhAver) - pow(10,-9) ) <= 0){
        //R0S_ERROR("位置更新处出现了分母为0的情况！");
    }
    else{
        mpv << 0,1/rmhAver,0,
             (1.0 / cos(LAver / rnhAver)),0,0,
             0,0,1;
    }
    p2 = avp1.block(0,6,1,3) +  (mpv * ( vel2 + avp1.block(0,3,1,3) ).transpose() / 2.0 * deltaT).transpose();
    return p2; 
}
Vector3d solPosFir(MatrixXd avp1,MatrixXd imuData,Vector3d vel2,double deltaT){
    double hAver =  avp1(8);
    double LAver = avp1(6);
    double re = 6378254.0;
    double e2 = 2.0 / 298.3 - 1.0 / 298.3 / 298.3;
    double rnAver = re / sqrt(1 - e2 * sin(LAver) * sin(LAver));
    double rmAver = rnAver * (1 - e2) / (1 - e2 * sin(LAver) * sin(LAver));
    double rnhAver = rnAver + hAver;
    double rmhAver = rmAver + hAver;
    MatrixXd mpv(3,3);
    MatrixXd p2(1,3);
    if ( abs( cos(LAver / rnhAver) - pow(10,-9) ) <= 0){
        //R0S_INFO("位置更新处出现了分母为0的情况！");
    }
    else{
        mpv << 0,1/rmhAver,0,
             (1.0 / cos(LAver / rnhAver)),0,0,
             0,0,1;
    }
    p2 = avp1.block(0,6,1,3) +  (mpv * (avp1.block(0,3,1,3).transpose() + vel2) * deltaT * 0.5).transpose();
    return p2; 
}