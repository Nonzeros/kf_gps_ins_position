#include <cmath>
#include <vector>
#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include <eigen3/Eigen/Dense>
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
typedef Eigen::Matrix<double,10,1> Vector10d;
typedef Eigen::Matrix<double,3,2> Matrix32d;
typedef Eigen::Matrix<double,6,2> Matrix62d;
typedef Eigen::Matrix<double,15,15> Matrix15d;
typedef Eigen::Matrix<double,15,6> Matrix156d;
typedef Eigen::Matrix<double,15,1> Vector15d;
typedef Eigen::Matrix<double,6,1> Vector6d;
#ifndef _EXGUIDE_H_
#define _EXGUIDE_H_

// ***********************************gazebo平面xy与经纬高互换*******************************
class PointCG
{
private:
    double latitude;
    double longitude;
    double lat0 = 22.799674;
    double lat1 = 22.799674;
    double lon0 = 113.959796;
    double x = 0.0;
    double y = 0.0;
    double R = 6371000;
public:
    PointCG();
    PointCG(double latitude,double longitude,double R=6371000,double lat0=22.799674,double lon0=113.959796,double lat1 = 22.799674);

    void setLat(double latitude);
    void setLon(double longitude);
    void setX(double x);
    void setY(double y);
    void set(double latitude,double longitude);
    double getX(void);
    double getY(void);
    double getLat(void);
    double getLon(void);
    void calXY(void);
    void calLLH(void);
};
// ***********************************gazebo平面xy与经纬高互换*******************************
// ***********************************RRT路径规划begin*************************************
class RRTNode
{
public:
    RRTNode();
    RRTNode(double x,double y);
    double x;
    double y;
    RRTNode* parNodePtr = NULL;
    int selfindex = 0;
    int parindex = -1;
};

class RRTMap
{
public:
    RRTMap();
    RRTMap(double width,double height,std::vector<int8_t>& datas);
    int getValue(int x,int y);
    int coordX2mapx(double x);
    int coordY2mapy(double y);
    double mapx2coordX(int x);
    double mapy2coordY(int y);

    std::vector<int8_t> datas;
    double occupied_thresh = 0.65;
    double originX = -50;
    double originY = -50;
    double resolution = 0.050000;
    double height;
    double width;
    //~RRTMap();
};

class RRTEdge
{
public:
    RRTNode start;
    RRTNode end;
    double length;
    RRTEdge();   
    RRTEdge(RRTNode start,RRTNode end);   
};


class RRT
{
private:
    std::vector<RRTNode> tree;
    std::vector<RRTNode> treemiddle;
    RRTMap map;
    RRTNode goal;
    RRTNode begin;
    double stepSize;
    ros::Publisher pointpub;
    ros::Publisher goalpointpub;
    ros::Publisher startpointpub;
public:
    RRT(/* args */);
    void init(RRTMap& map,RRTNode& begin,RRTNode& goal,double stepSize,ros::Publisher& pointpub,ros::Publisher& goalpointpub,ros::Publisher& statpointpub);
    RRTNode sample(RRTMap& map);
    RRTNode near(RRTNode& xrand,std::vector<RRTNode>& tree);
    RRTNode steer(RRTNode& xrand,RRTNode& near,double stepSize);
    RRTEdge geneEdge(RRTNode& xnew,RRTNode& near);
    int isCollision(RRTMap& map,RRTEdge& edge);
    void addNode(RRTNode& near,RRTNode& node);
    std::vector<RRTNode> solve(int n);
    std::vector<RRTNode> successful(RRTNode& xnow);
};


// 获取随机整数
int getrand(int min,int max);

// ***********************************RRT路径规划end*************************************

// ***********************************UWB信息简易求解************************************
class UWBanchor
{
public:
    int anchorID;
    double distanceWithTag = 0.0;
    double selfX;
    double selfY;
    double selfZ;
    UWBanchor();
    UWBanchor(int id,double selfX,double selfY,double selfZ);
    void updateDis(double distanceWithTag);
};

class EasyUWB
{
public:
    //std::vector<double> modelPos;
    // std::vector<UWBanchor> anchors;
    EasyUWB();
    std::vector<double> solveModelPos(std::vector<UWBanchor> anchors);
};



// ***********************************UWB信息简易求解************************************

// ***********************************捷联惯导双子样求解**********************************
// 由于这里是一次读取一次数据，所有向量都是列向量
Matrix3d att2mat(Vector3d att);
Vector3d mat2att(Matrix3d mat);
Matrix3d anti(Vector3d omega);
Matrix3d Mrv(Vector3d phi);
Vector3d extrapolation(Vector3d x0,Vector3d x1);
double extrapolation(double x0,double x1);
Vector3d attsolve(Matrix32d wm,Matrix32d vm,Vector10d avp1,double T);
Vector3d velsolve(Matrix32d wm,Matrix32d vm,Vector10d avp0,Vector10d avp1,double T);
Vector3d possolve(Vector10d avp0,Vector10d avp1,double T,Vector3d vel2);

// 计算地球参数的类，执行cal函数后，直接从该类对象中读取数据
class EarthPraSolver
{
public:
    EarthPraSolver();
    Vector3d wnie;
    Vector3d wnen;
    Vector3d g;
    double RM;
    double RN;
    double RMh;
    double RNh;
    void cal(Vector10d avp);
};

class ImuSolver
{
public:
    ImuSolver();
    int solveCount = 1;
    Vector10d avp0,avp1,avp2,avpOut;
    void sinsSolve(Matrix62d wvm,Vector10d avp00,double T);
};

// ***********************************捷联惯导双子样求解**********************************
// ***********************************eskf(误差状态卡尔曼滤波)****************************
class ESKFSolver
{
public:
    ESKFSolver();

    int type;
    double T;
    int solveCounts;
    Matrix15d F;
    Matrix156d G;
    Matrix15d P0;
    MatrixXd Kk;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd H;
    Matrix15d Pk,Pkk_1,Pk_1;
    Vector10d deltaX; // 最后一个值没用
    // Vector10d avp2_bar; // 给出修正后的avp2
    Vector15d Xkk_1;
    Vector15d Xk;
    
    
    void init(double pk0,Vector6d q,Vector6d r,int type,double T);
    void solveFG(Matrix62d wvm,Vector10d avp1,double T);
    Vector10d avpUpdate(Vector10d avp2,Vector15d Xk);
    Vector10d solveAvpBar(MatrixXd uwbData,Matrix62d wvm,Vector10d avp1,Vector10d avp2,double* imuBias);
    

};

// ***********************************eskf(误差状态卡尔曼滤波)****************************

// ***********************************一些补充的数学运算**********************************
double sec(double x);
// ***********************************一些补充的数学运算**********************************

#endif

#ifndef PI
#define PI 3.1415926535
#endif