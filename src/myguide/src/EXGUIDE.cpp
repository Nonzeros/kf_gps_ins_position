#include "myguide/EXGUIDE.h"
#include <ctime>
#include <cstdlib>
#include <random>

// ***********************************gazebo平面xy与经纬高互换*******************************
PointCG::PointCG(double latitude,double longitude,double R,double lat0,double lon0,double lat1){
    this->latitude = latitude;
    this->longitude = longitude;
    this->lon0 = lon0;
    this->lat0 = lat0;
    this->lat1 = lat1;
    this->R = R;
    this->calXY();
}

PointCG::PointCG(){}

void PointCG::setLat(double latitude){
    this->latitude = latitude;
}
void PointCG::setLon(double longitude){
    this->longitude = longitude;
}

void PointCG::setX(double x){
    this->x = x;
}

void PointCG::setY(double y){
    this->y = y;
}

double PointCG::getX(void){
    return this->x;
}
double PointCG::getY(void){
    return this->y;
}

double PointCG::getLat(void){
    return this->latitude;
}

double PointCG::getLon(void){
    return this->longitude;
}

void PointCG::calXY(){
    this->x = this->R * (longitude - lon0) * PI / 180.0 * cos(lat1 * PI / 180.0);
    this->y = this->R * (latitude - lat0) * PI / 180.0;
}

void PointCG::calLLH(){
    this->longitude = this->x / cos(lat1 * PI / 180.0) / this->R * 180 / PI + lon0;
    this->latitude = this->y / this->R *180 / PI + lat0;
}

void PointCG::set(double latitude,double longitude){
    // 输入的经纬度是角度制
    this->latitude = latitude;
    this->longitude = longitude;
    this->calXY();
    
}

// ***********************************gazebo平面xy与经纬高互换*******************************

// ***********************************RRT路径规划begin*************************************
RRTNode::RRTNode(double x,double y){
    this->x = x;
    this->y = y;
}

RRTNode::RRTNode(){
}


RRTMap::RRTMap(double width,double height,std::vector<int8_t>& datas){
    this->height = height;
    this->width = width;
    this->datas = datas;
}
RRTMap::RRTMap(){
}

int RRTMap::coordX2mapx(double coord_x){
    int x = (coord_x - this->originX) / this->resolution - 0.5; 
    return x;
}
int RRTMap::coordY2mapy(double coord_y){
    int y = (coord_y - this->originY) / this->resolution - 0.5; 
    return y;
}

double RRTMap::mapx2coordX(int x){
    double x_coord = this->originX + (x + 0.5) * this->resolution;
    return x_coord;
}
double RRTMap::mapy2coordY(int y){
    double y_coord = this->originY + (y + 0.5) * this->resolution;
    return y_coord;
}

int RRTMap::getValue(int x,int y){
    int value = this->datas[x + y * this->width];
    return value;
}

RRTEdge::RRTEdge(RRTNode start,RRTNode end){
    this->start = start;
    this->end = end;
    this->length = sqrt( pow(start.x - end.x,2) + pow(start.y - end.y,2) ); 
}
RRTEdge::RRTEdge(){
}

void RRT::init(RRTMap& map,RRTNode& begin,RRTNode& goal,double stepSize,ros::Publisher& pointpub,ros::Publisher& goalpointpub,ros::Publisher& startpointpub){
    this->map = map;
    this->tree.push_back(begin);
    this->goal = goal;
    this->begin = begin;
    this->stepSize = stepSize;
    this->pointpub = pointpub;
    this->goalpointpub = goalpointpub;
    this->startpointpub = startpointpub;
    geometry_msgs::PointStamped startpointmsg;
    geometry_msgs::PointStamped goalpointmsg;
    ros::Time nowtime = ros::Time::now();
    ros::Duration d(0.3);
    d.sleep();

    startpointmsg.header.frame_id = "map";
    startpointmsg.header.stamp = nowtime;
    startpointmsg.point.x = this->begin.x;
    startpointmsg.point.y = this->begin.y;
    startpointmsg.point.z = 0;
    this->startpointpub.publish(startpointmsg);

    goalpointmsg.header.frame_id = "map";
    goalpointmsg.header.stamp = nowtime;
    goalpointmsg.point.x = this->goal.x;
    goalpointmsg.point.y = this->goal.y;
    goalpointmsg.point.z = 0;
    this->goalpointpub.publish(goalpointmsg);
}
RRTNode RRT::sample(RRTMap& map){
    // 地图从左下角开始，x方向是length，y是width
    srand(time(0));
    RRTNode xrand;
    double A = map.originX;
    double B = -map.originX;

    double C = map.originY;
    double D = -map.originY;
    // double r3 = A + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(B-A)));
    // double r4 = C + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(D-C)));
    // xrand.x = r3;
    // xrand.y = r4;
    
    std::random_device seed;//硬件生成随机数种子
	std::ranlux48 engine(seed());//利用种子生成随机数引擎
    std::uniform_int_distribution<> distrib(A, B);//设置随机数范围，并为均匀分布
    xrand.x = distrib(engine);//随机数

    std::random_device seed2;//硬件生成随机数种子
	std::ranlux48 engine2(seed2());//利用种子生成随机数引擎
    std::uniform_int_distribution<> distrib2(C, D);//设置随机数范围，并为均匀分布
    xrand.y = distrib(engine2);

    while ((xrand.x - this->begin.x) < 0.00001 && (xrand.y - this->begin.y) < 0.00001)
    {
        xrand.x = distrib(engine);
        xrand.y = distrib(engine2);
    }
    return xrand;
}
RRTNode RRT::near(RRTNode& xrand,std::vector<RRTNode>& tree){
    int treeSize = tree.size();
    double mindis = 99999999;
    double distance2;
    int minindex = -1;
    for (int i = 0; i < treeSize; i++)
    {   
        distance2 = sqrt(pow(xrand.x - tree[i].x,2) + pow(xrand.y - tree[i].y,2));  
        if (distance2 < mindis)
        {
            mindis = distance2;
            minindex = i;
        }
    }
    return tree[minindex];
}
RRTNode RRT::steer(RRTNode& xrand,RRTNode& near,double stepSize){
    RRTNode xnew;
    double distance2 = sqrt(pow(xrand.x - near.x,2) + pow(xrand.y - near.y,2));
    if ( (distance2 - 0.0) < 0.000001)
    {
        xnew.x = 0.0;
        xnew.y = 0.0;
    }else{
        xnew.x = stepSize / distance2 * (xrand.x - near.x) + near.x; 
        xnew.y = stepSize / distance2 * (xrand.y - near.y) + near.y;
    }
    return xnew;
}

RRTEdge RRT::geneEdge(RRTNode& xnew,RRTNode& near){
    RRTEdge edge(near,xnew);
    return edge;
}

int RRT::isCollision(RRTMap& map,RRTEdge& edge){
    // 判断地图上这条线经过的地方有没有障碍物
    // 如果新生成的点有障碍物，那就不用往下算了
    int x1 = map.coordX2mapx(edge.start.x);
    int x2 = map.coordX2mapx(edge.end.x);
    int y1 = map.coordY2mapy(edge.start.y);
    int y2 = map.coordY2mapy(edge.end.y);

    if (map.getValue(map.coordX2mapx(edge.end.x),map.coordY2mapy(edge.end.y)) > 0.6) 
    {
        return 1;        
    }

    // 如果新生成的点没有障碍物，那就算这条线上有没有障碍物
    
    
    for (int i = std::min(x1,x2); i < std::max(x1,x2); i++)
    {
        for (int j = std::min(y1,y2); j < std::max(y1,y2); j++)
        {
            if (map.getValue(i,j) > 0.6)
            {
                return 1;
            } 
        }
    }

    //  if (map.getValue(map.coordX2mapx(edge.end.x),map.coordY2mapy(edge.end.y)) == 100 || map.getValue(map.coordX2mapx(edge.end.x),map.coordY2mapy(edge.end.y)) == -1 || map.getValue(map.coordX2mapx(edge.end.x),map.coordY2mapy(edge.end.y)) == 1)
    // {
    //     return 1;        
    // }

    // // 如果新生成的点没有障碍物，那就算这条线上有没有障碍物
    
    
    // for (int i = std::min(x1,x2); i < std::max(x1,x2); i++)
    // {
    //     for (int j = std::min(y1,y2); j < std::max(y1,y2); j++)
    //     {
    //         if (map.getValue(i,j) == 100 || map.getValue(i,j) == -1 || map.getValue(i,j) == 1)
    //         {
    //             return 1;
    //         } 
    //     }
    // }


    return 0;
    
    
}
void RRT::addNode(RRTNode& near,RRTNode& node){
    static int counts = 1;
    node.selfindex = counts;
    node.parindex = near.selfindex;
    this->tree.push_back(node);
    geometry_msgs::PointStamped pointmsg;
    pointmsg.header.frame_id = "map";
    pointmsg.header.stamp = ros::Time::now();
    pointmsg.point.x = node.x;
    pointmsg.point.y = node.y;
    pointmsg.point.z = 0;
    this->pointpub.publish(pointmsg);
    counts += 1;
}
std::vector<RRTNode> RRT::successful(RRTNode& xnew){
    // 路径查找成功了，从最后一个节点以此寻找节点，放到一个数组里
    std::vector<RRTNode> mypathNode;
    int parindexpoint = xnew.selfindex;
    mypathNode.push_back(this->goal);
    int length = this->tree.size();

    while(parindexpoint != -1)
    {  
        mypathNode.push_back(this->tree[parindexpoint]);
        parindexpoint = tree[parindexpoint].parindex;
    }
    

    return mypathNode;
}

RRT::RRT(){

}
std::vector<RRTNode> RRT::solve(int n){
    std::vector<RRTNode> mypathNodes;
    for (int i = 1; i < n; i++)
    {

        RRTNode xrand = sample(this->map);
        RRTNode xnear = near(xrand,this->tree);
        RRTNode xnew = steer(xrand,xnear,this->stepSize);
        // xnew.selfindex = i;

        RRTEdge edge(xnear,xnew);
        if (!isCollision(this->map,edge))
        {
            this->addNode(xnear,xnew);
            double middles = sqrt(pow(xnew.x - this->goal.x,2) + pow(xnew.y - this->goal.y,2));
            if (middles <= this->stepSize)
            {
            // 如果这个新的点距离目标点小于步长，将认为规划完毕
                mypathNodes = this->successful(xnew);

                return mypathNodes;
            }
        }

        // if (i >= 25)
        // {
        //     // 如果这个新的点距离目标点小于步长，将认为规划完毕
        //     mypathNodes = this->successful(xnew);
        //     return mypathNodes;
        // }

    }
    // mypathNodes = this->successful(xnew);
    return mypathNodes;
}

int getrand(int min,int max){
    return ( rand() % (max - min + 1) ) + min;
}

// ***********************************RRT路径规划end*************************************

// ***********************************UWB简易求解****************************************
UWBanchor::UWBanchor(){
}

UWBanchor::UWBanchor(int id,double selfX,double selfY,double selfZ){
    this->anchorID = id;
    this->selfX = selfX;
    this->selfY = selfY;
    this->selfZ = selfZ;
}
void UWBanchor::updateDis(double distance){
    this->distanceWithTag = distance;
}

EasyUWB::EasyUWB(){
}

std::vector<double> EasyUWB::solveModelPos(std::vector<UWBanchor> anchors){
    int length = anchors.size();
    std::vector<double> modelPos;
    if (length == 4)
    {
        // 进行简易求解
        double x1 = anchors[0].selfX;  double y1 = anchors[0].selfY;  double z1 = anchors[0].selfZ;
        double x2 = anchors[1].selfX;  double y2 = anchors[1].selfY;  double z2 = anchors[1].selfZ;
        double x3 = anchors[2].selfX;  double y3 = anchors[2].selfY;  double z3 = anchors[2].selfZ;
        double x4 = anchors[3].selfX;  double y4 = anchors[3].selfY;  double z4 = anchors[3].selfZ;
        double d1 = anchors[0].distanceWithTag;  double d2 = anchors[1].distanceWithTag;
        double d3 = anchors[2].distanceWithTag;  double d4 = anchors[3].distanceWithTag;

        Eigen::Matrix2d A;
        Eigen::Vector2d B;
        Eigen::Vector2d X;
        Eigen::Matrix2d C;

        A << 2*(x2-x1),2*(y2-y1),
            2*(x3-x1),2*(y3-y1);
        B << d1*d1 - d2*d2 + x2*x2 - x1*x1 + y2*y2 - y1*y1,
             d1*d1 - d3*d3 + x3*x3 - x1*x1 + y3*y3 - y1*y1;
        C = A.inverse();
        X = C * B;

        // 三维的如果都在同一个平面，会出现矩阵奇异，因此是否使用三维求解要分情况！
        // Eigen::Matrix3d A;
        // Eigen::Vector3d X;
        // Eigen::Vector3d B;
        // Eigen::Matrix3d C;

        
        // A << 2*(x2-x1),2*(y2-y1),2*(z2-z1),
        //      2*(x3-x1),2*(y3-y1),2*(z3-z1),
        //      2*(x4-x1),2*(y4-y1),2*(z4-z1);
        // B << d1*d1 - d2*d2 + x2*x2 - x1*x1 + y2*y2 - y1*y1 + z2*z2 - z1*z1,
        //      d1*d1 - d3*d3 + x3*x3 - x1*x1 + y3*y3 - y1*y1 + z3*z3 - z1*z1,
        //      d1*d1 - d4*d4 + x4*x4 - x1*x1 + y4*y4 - y1*y1 + z4*z4 - z1*z1;
        // C = A.inverse();
        // X = C * B;

        modelPos.push_back(X(0));
        modelPos.push_back(X(1));
        // modelPos.push_back(X(2));
    }

    return modelPos;
    
}

// ***********************************UWB简易求解****************************************
// ***********************************捷联惯导双子样求解**********************************
Matrix3d att2mat(Vector3d att){
    double a1 = att(0);
    double a2 = att(1);
    double a3 = att(2);
    Matrix3d C1;
    Matrix3d C2;
    Matrix3d C3;
    Matrix3d mat;
    C1 << cos(a3),-sin(a3),0,
        sin(a3),cos(a3),0,
        0,0,1;

    C2 << 1,0,0,
        0,cos(a1),-sin(a1),
        0,sin(a1),cos(a1);

    C3 << cos(a2),0,sin(a2),
        0,1,0,
        -sin(a2),0,cos(a2);
    mat = C1 * C2 * C3;
    return mat;
}
Vector3d mat2att(Matrix3d mat){
    Vector3d att;
    double theta = asin(mat(2,1));
    double gamma,psi;
    if (abs(theta) <= 0.999999)
    {
        gamma = -atan2(mat(2,0),mat(2,2));
        psi = -atan2(mat(0,1),mat(1,1));
    }else{
        gamma = atan2(mat(0,2),mat(0,0));
        psi = 0;
    }
    
    att << theta,gamma,psi;
    return att;
}
Matrix3d anti(Vector3d omega){
    double x = omega(0);
    double y = omega(1);
    double z = omega(2);
    Matrix3d anti;
    anti << 0,-z,y,
            z,0,-x,
            -y,x,0;
    return anti;
}
Matrix3d Mrv(Vector3d phi){
    double normPhi = phi.norm();
    Matrix3d Mrv;
    if (normPhi <= 0.0000000001)
    {
        Mrv.setIdentity();
    }else{
        Mrv = Matrix3d::Identity() + sin(normPhi) / normPhi * anti(phi) +
        ( 1 - cos(normPhi) ) / normPhi / normPhi * anti(phi) * anti(phi);
    }
    return Mrv;
}
Vector3d extrapolation(Vector3d x0,Vector3d x1){
    Vector3d xhalf = ( 3 * x1 - x0) / 2;
    return xhalf;
}
double extrapolation(double x0,double x1){
    double xhalf = (3 * x1 - x0) / 2;
    return xhalf;
}
void EarthPraSolver::cal(Vector10d avp){
    double Re = 6378254; double g0 = 9.780325;
    double f = 1 / 298.257; double m = 1 /288.0;
    double e2 = 2.0 * f - f*f;
    double wie = 7.2921151467 * pow(10,-5);
    double L = avp(6); double h = avp(8);
    double beta = 2.5 * m - f; double beta1 = 1/8 * (2*beta*f + f*f);
    double gL = g0 * ( 1 + beta * sin(L) * sin(L) - beta1 * sin(2*L) * sin(2*L));
    double ve = avp(3); double vn = avp(4);

    this->g << 0,0,-gL;
    this->RM = Re * (1-e2) / pow( 1-e2*sin(L)*sin(L) , 1.5);
    this->RN = Re / pow( 1-e2*sin(L)*sin(L) , 0.5);
    this->wnie << 0,wie*cos(L),wie*sin(L);
    this->wnen << -vn/(RM + h),ve/(RN + h),ve*tan(L) / (RN + h);
    this->RNh = this->RN + h;
    this->RMh = this->RM + h;
}
EarthPraSolver::EarthPraSolver(){}
ImuSolver::ImuSolver(){}

Vector3d attsolve(Matrix32d wm,Matrix32d vm,Vector10d avp1,double T){
    Vector3d wm1 = wm.col(0);Vector3d wm2 = wm.col(1);
    Vector3d vm1 = vm.col(0);Vector3d vm2 = vm.col(1);
    Vector3d att1;att1 << avp1(0),avp1(1),avp1(2);
    Vector3d phi_ib = wm1 + wm2 + 2.0/3.0 * wm1.cross(wm2);
    Matrix3d Cb_b, Cn_b_, Cnn_, Cnb;
    Cb_b = Mrv(phi_ib);
    Cn_b_ = att2mat(att1);
    EarthPraSolver earPraSolver;
    earPraSolver.cal(avp1);
    Vector3d wnin = earPraSolver.wnie + earPraSolver.wnen;
    Cnn_ = Mrv(T * wnin);
    Cnb = Cnn_ * Cn_b_ * Cb_b;
    Vector3d att2 = mat2att(Cnb);
    return att2;
}
Vector3d velsolve(Matrix32d wm,Matrix32d vm,Vector10d avp0,Vector10d avp1,double T){
    // v0为k-2时刻速度，v1为k-1时刻速度
    Vector3d v1; v1 << avp1(3),avp1(4),avp1(5);
    Vector3d v0; v0 << avp0(3),avp0(4),avp0(5);
    Vector3d att1; att1 << avp1(0),avp1(1),avp1(2);
    Matrix3d Cn_b_,win_anti;

    Vector3d wm1,wm2,vm1,vm2;
    Vector3d wie_middle,wen_middle,v_middle,g_middle,win_middle;
    Vector3d deltaVcor,deltaVrot,deltaVscul,deltaVsf;
    // 角增量和速度增量
    wm1 = wm.col(0);wm2 = wm.col(1);
    vm1 = vm.col(0);vm2 = vm.col(1);
    
    // 地球参数计算
    EarthPraSolver earth0;
    EarthPraSolver earth1;
    earth0.cal(avp0); earth1.cal(avp1);

    // 外推计算(外推具有线性性)
    wie_middle = extrapolation(earth0.wnie,earth1.wnie);
    wen_middle = extrapolation(earth0.wnen,earth1.wnen);
    v_middle = extrapolation(v0,v1);
    g_middle = extrapolation(earth0.g,earth1.g);
    win_middle = wie_middle + wen_middle;

    // 有害加速度增量
    Vector3d aa1 = 2 * wie_middle + wen_middle;
    deltaVcor = ( -aa1.cross(v_middle) + g_middle ) * T;

    // 导航系比力速度增量
    deltaVrot = 0.5 * (wm1+wm2).cross(vm1+vm2);
    deltaVscul = 2.0/3.0 * ( wm1.cross(vm2) + vm1.cross(wm2) );
    Cn_b_ = att2mat(att1);
    win_anti = anti(win_middle);
    deltaVsf = ( Matrix3d::Identity() - T/2.0 * win_anti ) * Cn_b_ * (vm1 + vm2 + deltaVrot + deltaVscul);

    // 计算结果
    Vector3d vel2 = v1 + deltaVsf + deltaVrot;
    return vel2;
}
Vector3d possolve(Vector10d avp0,Vector10d avp1,double T,Vector3d vel2){
    // 一些参数定义和地球参数计算
    Vector3d pos1; pos1 << avp1(6),avp1(7),avp1(8);
    Vector3d vel1; vel1 << avp1(3),avp1(4),avp1(5);
    EarthPraSolver earth0,earth1;
    earth0.cal(avp0);
    earth1.cal(avp1);
    double h0,h1,L0,L1,secL0,secL1;
    double secLRNh0,secLRNh1,RMh_middle,secLRNh_middle;

    h0 = avp0(8);h1 = avp1(8);
    L0 = avp0(6);L1 = avp1(6);
    secL0 = sec(L0);secL1 = sec(L1);
    
    secLRNh0 = secL0 / earth0.RNh;
    secLRNh1 = secL1 / earth1.RNh;
    
    RMh_middle = extrapolation(earth0.RMh,earth1.RMh);
    secLRNh_middle = extrapolation(secLRNh0,secLRNh1);
    
    Matrix3d Mpv_middle;
    Mpv_middle << 0,1.0/RMh_middle,0,
                secLRNh_middle,0,0,
                0,0,1;
    Vector3d pos2;
    pos2 = pos1 + ( Mpv_middle * (vel2 + vel1) * T / 2.0 );
    return pos2;
}

void ImuSolver::sinsSolve(Matrix62d wvm,Vector10d avp00,double T){
    // 输入的偏航角是北偏东为正，但计算时是北偏西为正，输入和输出的时候都换一下
    avp00(2) = -avp00(2);

    // T 是求解周期，即双子样算法，imu频率为100hz，T为2*0.01s；
    Matrix32d wms = wvm.block<3,2>(0,0);
    Matrix32d vms = wvm.block<3,2>(3,0);
    Vector3d att2,vel2,pos2;

    // 如果是第一次求解，那么将赋予初始值
    if (this->solveCount == 1)
    {
        this->avp0 = avp00;
        this->avp1 = avp00;
    }

    // 姿态更新
    att2 = attsolve(wms,vms,this->avp1,T);
    // 速度更新
    vel2 = velsolve(wms,vms,this->avp0,this->avp1,T);
    vel2(2,0) = 0.0;
    // 位置更新
    pos2 = possolve(this->avp0,this->avp1,T,vel2);
    pos2(2,0) = 0.0;

    // 合并求解结果
    this->avp2.block<3,1>(0,0) = att2;
    this->avp2.block<3,1>(3,0) = vel2;
    this->avp2.block<3,1>(6,0) = pos2;
    this->avp2(9) = solveCount * T;
    
    this->avp0 = this->avp1;
    this->avp1 = this->avp2;
    
    // 求解次数更新
    this->solveCount += 1;
    // 输出要对姿态角加个负号
    this->avpOut = this->avp2;
    this->avpOut(2) = -this->avpOut(2);


}
// ***********************************捷联惯导双子样求解**********************************

// ***********************************eskf(误差状态卡尔曼滤波)****************************
ESKFSolver::ESKFSolver(){}
void ESKFSolver::solveFG(Matrix62d wvm,Vector10d avp1,double T){
    // 参数定义相关
    Matrix3d M1,M2,M3;
    Matrix3d Maa,Mav,Map;
    Matrix3d Mva,Mvv,Mvp;
    Matrix3d Mpv,Mpp;
    Matrix3d zeros3 = Matrix3d::Zero();

    Vector3d fnst,fbst,att1,v1;
    att1 << avp1(0),avp1(1),avp1(2);
    v1 << avp1(3),avp1(4),avp1(5);
    Matrix3d Cnb = att2mat(att1);
    fbst = ( wvm.block<3,1>(3,0) + wvm.block<3,1>(3,1) ) / T * 2;
    fnst = Cnb * fbst;

    EarthPraSolver earth1;
    earth1.cal(avp1);
    double L = avp1(6);
    double h = avp1(8);
    double ve = avp1(3);
    double vn = avp1(4);
    double m = 1.0 / 288.0;
    double f = 1.0 / 298.0;
    double beta = 2.5 * m - f;
    double beta1 = 1.0 / 8.0 * (2*beta * f + f*f);
    double beta2 = 3.08 * pow(10,-6);
    double beta3 = 8.08 * pow(10,-9);
    double g0 = 9.780325;
    double wie = 7.2921151467 * pow(10,-5);
    double RNh = earth1.RNh;
    double RMh = earth1.RMh;

    M1 << 0,0,0,
        -wie*sin(L),0,0,
        wie*cos(L),0,0;
    M2 << 0,0,vn/RMh/RMh,
        0,0,-ve/RNh/RNh,
        ve*(sec(L)*sec(L))/RNh,0,-ve*tan(L)/RNh/RNh;
    M3 << 0,0,0,
        -2*beta3*h*cos(2*L),0,-beta3*sin(2*L),
        -g0 * (beta - 4*beta1 * cos(2*L)) * sin(2*L),0,beta2;
    
    Maa = -anti(earth1.wnie + earth1.wnen);
    Mav << 0,-1.0/RMh,0,
            1/RNh,0,0,
            tan(L)/RNh,0,0;
    Map = M1 + M2;
    Mva = anti(fnst);
    Mvv = anti(v1) * Mav - anti(2*earth1.wnie + earth1.wnen);
    Mvp = anti(v1) * (2*M1 + M2) + M3;
    Mpv << 0,1.0/RMh,0,
            sec(L)/RNh,0,0,
            0,0,1;
    Mpp << 0,0,-vn/RMh/RMh,
            ve*sec(L)*tan(L)/RNh,0,-ve*sec(L)/RNh/RNh,
            0,0,0;
    Matrix15d F;
    F.block<3,3>(0,0) = Maa;
    F.block<3,3>(0,3) = Mav;
    F.block<3,3>(0,6) = Map;
    F.block<3,3>(0,9) = -Cnb;
    F.block<3,3>(0,12) = zeros3;

    F.block<3,3>(3,0) = Mva;
    F.block<3,3>(3,3) = Mvv;
    F.block<3,3>(3,6) = Mvp;
    F.block<3,3>(3,9) = zeros3;
    F.block<3,3>(3,12) = Cnb;

    F.block<3,3>(6,0) = zeros3;
    F.block<3,3>(6,3) = Mpv;
    F.block<3,3>(6,6) = Mpp;
    F.block<3,3>(6,9) = zeros3;
    F.block<3,3>(6,12) = zeros3;

    Eigen::Matrix<double, 6, 15> zeros6_15 = Eigen::Matrix<double, 6, 15>::Zero();
    F.block<6,15>(9,0) = zeros6_15;
    this->F = F;

    // 计算G
    Matrix156d G;// 15*6的矩阵
    G.block<3,3>(0,0) = -Cnb;
    G.block<3,3>(0,3) = zeros3;
    G.block<3,3>(3,0) = zeros3;
    G.block<3,3>(3,3) = Cnb;
    Eigen::Matrix<double, 9, 6> zeros9_6 = Eigen::Matrix<double, 9, 6>::Zero();
    G.block<9,6>(6,0) = zeros9_6;
    this->G = G;

}

void ESKFSolver::init(double pk0,Vector6d q,Vector6d r,int type,double T){
    // 滤波器初始化，包括P0，Q，R，H(根据状态量决定)和T(双子样求解周期)确定
    this->T = T;
    this->type = type;
    Matrix3d zeros3 = Matrix3d::Zero();
    Matrix3d eye3 = Matrix3d::Identity();
    this->Q = MatrixXd::Zero(6,6);
    this->Q.diagonal() = q;
    this->P0 = Matrix15d::Identity(15,15) * pk0;
    this->Pk_1 = P0;

    // type = 1，2，3分别是6状态观测，3速度观测和3位置观测
    if(type == 1){

        // Kk大小设置
        this->Kk.resize(15,6);

        // 观测矩阵H设置
        this->H.resize(6,15);

        this->H.block<3,3>(0,0) = Matrix3d::Zero();
        this->H.block<3,3>(0,3) = eye3;
        this->H.block<3,3>(0,6) = zeros3;
        this->H.block<3,3>(0,9) = zeros3;
        this->H.block<3,3>(0,12) = zeros3;

        this->H.block<3,3>(3,0) = zeros3;
        this->H.block<3,3>(3,3) = zeros3;
        this->H.block<3,3>(3,6) = eye3;
        this->H.block<3,3>(3,9) = zeros3;
        this->H.block<3,3>(3,12) = zeros3;
        // 观测噪声R矩阵设置
        this->R = MatrixXd::Zero(6,6);
        this->R.diagonal() = r;

    }else if (type == 2)
    {
        this->Kk.resize(15,3);

        this->H.resize(3,15);
        this->H.block<3,3>(0,0) = zeros3;
        this->H.block<3,3>(0,3) = eye3;
        this->H.block<3,3>(0,6) = zeros3;
        this->H.block<3,3>(0,9) = zeros3;
        this->H.block<3,3>(0,12) = zeros3;

        this->R = MatrixXd::Zero(3,3);
        this->R.diagonal() = r.block<3,1>(0,0);
    }else if (type == 3)
    {
        this->Kk.resize(15,3);

        this->H.resize(3,15);
        this->H.block<3,3>(0,0) = zeros3;
        this->H.block<3,3>(0,3) = zeros3;
        this->H.block<3,3>(0,6) = eye3;
        this->H.block<3,3>(0,9) = zeros3;
        this->H.block<3,3>(0,12) = zeros3;

        this->R = MatrixXd::Zero(3,3);
        this->R.diagonal() = r.block<3,1>(3,0);
    }
    
    
}
Vector10d ESKFSolver::avpUpdate(Vector10d avp2,Vector15d Xk){
    Vector3d att2,deltaAtt;
    att2 << avp2(0),avp2(1),avp2(2);
    deltaAtt = Xk.block<3,1>(0,0);
    Matrix3d Cn0b,Cnn_,Cnn;
    Cn0b = att2mat(att2);
    Cnn_ = att2mat(deltaAtt);
    Cnn = Cnn_ * Cn0b;

    Vector10d avp2_bar;
    avp2_bar.block<3,1>(0,0) = mat2att(Cnn);
    avp2_bar.block<3,1>(3,0) = avp2.block<3,1>(3,0) - Xk.block<3,1>(3,0);
    avp2_bar.block<3,1>(6,0) = avp2.block<3,1>(6,0) - Xk.block<3,1>(6,0);
    avp2_bar(9) = avp2(9);
    return avp2_bar;
}
Vector10d ESKFSolver::solveAvpBar(MatrixXd uwbData,Matrix62d wvm,Vector10d avp1,Vector10d avp2,double* imuBias){
    Vector3d uwbVel,uwbPos,insVel,insPos;
    insVel << avp2(3),avp2(4),avp2(5);
    insPos << avp2(6),avp2(7),avp2(8);

    MatrixXd Z;
    // 先计算这个时刻的FG
    this->solveFG(wvm,avp1,this->T);
    //判断滤波器观测类型
    if (this->type == 1)
    {
        Z.resize(6,1);
        uwbVel << uwbData(3,0),uwbData(4,0),uwbData(5,0);
        uwbPos << uwbData(0,0),uwbData(1,0),uwbData(2,0);
        Z.block<3,1>(0,0) = insVel - uwbVel;
        Z.block<3,1>(3,0) = insPos - uwbPos;
    }else if (this->type == 2)
    {
        Z.resize(3,1);
        uwbVel << uwbData(3,0),uwbData(4,0),uwbData(5,0);
        Z.block<3,1>(0,0) = insVel - uwbVel;
    }else if (this->type == 3)
    {
        Z.resize(3,1);
        uwbPos << uwbData(0,0),uwbData(1,0),uwbData(2,0);
        Z.block<3,1>(0,0) = insPos - uwbPos;
        // Z(2,0) = 0;
    }
    
    // Kalman滤波计算
    Vector15d Xk_1;
    Xk_1 = Vector15d::Zero();
    
    Matrix15d mm1,mm2;
    mm1 = F * Pk_1 * F.transpose();
    mm2 = G * Q * G.transpose();
    Xkk_1 = F * Xk_1;
    Pkk_1 = mm1 + mm2;
    Kk = Pkk_1 * H.transpose() * ( H * Pkk_1 * H.transpose() + R).inverse();
    Xk = Xkk_1 + Kk * (Z - H*Xkk_1);
    Pk = ( Matrix15d::Identity() - Kk*H) * Pkk_1;
    
    this->Pk_1 = Pk; // 方差阵更新
    this->solveCounts += 1;
    Vector10d avp_bar;
    for (int i = 0; i < 6; i++)
    {
        *(imuBias + i) = Xk(9+i,0);
    }
    
    avp_bar = avpUpdate(avp2,Xk); // 惯导解算结果修正

    
    PointCG pointcg2(22.799674,113.959796);
    PointCG pointcg2_bar(22.799674,113.959796);
    pointcg2.set(avp2(6)/PI*180,avp2(7)/PI*180);
    pointcg2_bar.set(avp_bar(6)/PI*180,avp_bar(7)/PI*180);

    return avp_bar;
    
}
// ***********************************eskf(误差状态卡尔曼滤波)****************************

// ***********************************一些补充的数学运算**********************************
double sec(double x){
    double result = 1.0 / cos(x);
    return result;
}
// ***********************************一些补充的数学运算**********************************