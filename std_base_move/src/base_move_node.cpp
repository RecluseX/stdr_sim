#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/transform_broadcaster.h"
#include<tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_broadcaster.h"
#include "BaseBridge.h"

#include "MyTypeDefine.h"

#include "BaseTimeApi.h"

#include "std_base_move/vel_pub.h"


#define DEG2RAD(x) x*M_PI/180.0
#define D2R M_PI/180.0
#define R2D 180.0/M_PI

bool gResetFlag = false;
#define wheel_base_dist 0.520
#define wheel_radius 0.170/2.0
#define decel_ratio 25
//define function
using namespace std;
ros::Time ToROSTime(S64 timeTick)
{
    return ros::Time((S64)timeTick/1000.0,long(timeTick%1000)*1000000);
}
float  RESOLUTION  = 1440.0/360.0 ;
float _maskDeg_[3][2] = {0};

void ldsDataFilter(const std::vector<float> pilarRange , float ldsOffset)
{
	if(pilarRange.size() != 6 )
		return;
	_maskDeg_[0][0] = (pilarRange[0] +  ldsOffset)*RESOLUTION;
	_maskDeg_[0][1] = (pilarRange[1]  +  ldsOffset)*RESOLUTION;
	_maskDeg_[1][0] = (pilarRange[2] +  ldsOffset)*RESOLUTION;
	_maskDeg_[1][1] = (pilarRange[3] +  ldsOffset)*RESOLUTION;

	_maskDeg_[2][0] = (pilarRange[4] +  ldsOffset)*RESOLUTION;
	_maskDeg_[2][1] = (pilarRange[5] +   ldsOffset)*RESOLUTION;
	return ;
}


class Node{

    public:
        Node();
        ~Node();
        bool initialize();
        void spinForever();
    private:
        string mcu_comm_name_;
        string lds_comm_name_;
        Odom_Type lastReading_;
        float xx_;
        float yy_;
        float theta_;

        ros::Publisher vp_pub_;


        ros::Publisher odom_pub_;
        ros::Publisher scan_pub_;
        ros::Subscriber cmd_sub_;
        ros::NodeHandle node_handle_;
        ros::NodeHandle private_nh_;
        vector<ros::WallTimer> wallTimers_;
        tf2_ros::TransformBroadcaster odom_broadcaster_;
        BaseBridgeAPI* base_bridge_;


        //function define
        void PublishOdom(const ros::WallTimerEvent& event);
        void PublishScan(const ros::WallTimerEvent& event);
        void HandleVelCMD(const geometry_msgs::TwistConstPtr&  vel_cmd);

        void test_vel(const ros::WallTimerEvent& event);

        ros::Time last_odom_;
        double last_odom_sec;

        vector<ros::Time> vec_odo_t;
        vector<double> vec_wz;
   

        double _wz;
        double _vx;

        double cmd_vl_;
        double cmd_vr_;
        double odom_vl_;
        double odom_vr_;

        /* print vel in prt_vel msg by vp_pub Publisher, add in 2018/05/29 by Qiu */
        ros::NodeHandle nh2;

        ros::Time last_vt_;
        double last_cmd_vl;
        double last_cmd_vr;
        static const double coe_1lf_v = 0.6;

        ros::Time last_ang_t_;
        double last_ang_;
        vector<ros::Time> angular_time_;
        vector<double> angular_;
        vector<double> angular_acc_;
        static const int max_angular_size = 7;
        static const double acc_lim_theta = 2.5;
        static const double period = 0.05;
        double acc_theta;

        double acc_abs_plus;
        double acc_plus;
        double angular_filter(double wz);
        static const double max_ang_diff = 0.3;
        static const double coe_ang_1lf = 0.7;

        int count_ = 0;
        double c_wz = 72.0 * D2R;

};

Node::Node():private_nh_("~")
{
   private_nh_.param("/base_move_node/mcu_comm",mcu_comm_name_,std::string("/dev/ttyUSB0"));
   private_nh_.param("/base_move_node/lds_comm",lds_comm_name_,std::string("/dev/ttyUSB1"));

   lastReading_.LDist = 0;
   lastReading_.RDist = 0;
   lastReading_.Gyro = 0;

   xx_=yy_=theta_=0.0;
  // last_odom_sec = 0.0;
}
Node::~Node()
{
    if(base_bridge_ != NULL)
    {
       delete base_bridge_;
    }
}

double Node::angular_filter(double wz)
{
    ros::Time now_ = ros::Time::now();

    if(last_ang_t_ + ros::Duration(1) < now_)
    {
        last_ang_ = wz;
        last_ang_t_ = now_;
        ROS_INFO("received start angle");
        return wz;
    }

    double dt = now_.toSec() - last_ang_t_.toSec();
    if(fabs(wz - last_ang_) > acc_lim_theta * dt) return last_ang_;

    double res = coe_ang_1lf * wz + (1 - coe_ang_1lf) * last_ang_;
	
    last_ang_ = res; last_ang_t_ = now_;
    return res;

}

S32 deltaTheta(S32 a, S32 b)
{
    S32 c = 0.0;
    c = a - b;
    if(std::abs(c) > 18000)
    {
        return c + c/std::abs(c) * (-36000);
    }
    return c;
}

bool Node::initialize()
{
    base_bridge_ = new BaseBridgeAPI;
    ROS_INFO("mcu_comm_name_:%s",mcu_comm_name_.c_str());
    int rstFlag = 0;

    vp_pub_ = nh2.advertise<std_base_move::vel_pub>("pub_vel1", 1);

    if(base_bridge_->MCUInit(mcu_comm_name_) !=0 )
    {
        ROS_WARN("please check your MCU port!");

    }
    else{
        rstFlag += 1;
       usleep(100);
      base_bridge_->GYIODistanceReset(0);

      gResetFlag = true;
      wallTimers_.push_back(node_handle_.createWallTimer(ros::WallDuration(0.02),&Node::PublishOdom,this));
    //   wallTimers_.push_back(node_handle_.createWallTimer(ros::WallDuration(0.02),&Node::test_vel,this));

      odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>("/odom", 10);
      cmd_sub_ = node_handle_.subscribe("/cmd_vel",1,&Node::HandleVelCMD,this);

    }
/*    ROS_INFO("lds_comm_name_:%s",lds_comm_name_.c_str());
    lds_bridge_ = new SensorInterfaceAPI;
    if( lds_bridge_->SensorInterfaceInit(lds_comm_name_))
    {
        ROS_WARN("please check your lds port!");
    }
    else{
      rstFlag += 1;
      usleep(100);
      scan_pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan",1);
      wallTimers_.push_back(node_handle_.createWallTimer(ros::WallDuration(0.06),&Node::PublishScan,this));

    }
  */  if(rstFlag > 0)
    return true;
    else return false;
}
void Node::HandleVelCMD(const geometry_msgs::TwistConstPtr&  vel_cmd)
{
    geometry_msgs::Twist msg = *vel_cmd;
    double vx = msg.linear.x;
    double wz = msg.angular.z;
    double dl = 0.282;

    _wz = wz;
    _vx = vx;

    // ROS_INFO("wz: %f, vx: %f\n", _wz, _vx);
    // S16 vl;
    // S16 vr;


    S16 vl = ((2.0f*vx - wz*dl)/2.0f)*1000;
    S16 vr = ((2.0f*vx + wz*dl)/2.0f)*1000;

     cmd_vl_ = vl/1000.0;
     cmd_vr_ = vr/1000.0;

    // //S16 vr = floor(((wz*dl + 2.0*vx)/2.0)*1000);
    // // S16 vl = floor((2.0*vx - vr/1000.0 )*1000);


//     /* one order lag algorithm, add in 2018/05/31 ---------------------------*/
//     ros::Time vt = ros::Time::now();
//     if(last_vt_ + ros::Duration(1) < vt)
//     {
//       last_cmd_vl = 0.0;
//       last_cmd_vr = 0.0;
//     }
//     last_vt_ = vt;

//     if(last_cmd_vl == 0.0)
//     {
//       cmd_vl_ = (2.0f*vx - wz*dl)/2.0f;
//       vl = cmd_vl_ * 1000;
//       cmd_vr_ = (2.0f*vx + wz*dl)/2.0f;
//       vr = cmd_vr_ * 1000;
//     }
//     else
//     {
//     //   cmd_vl_ = (2.0f*vx - wz*dl)/2.0f;
//       cmd_vl_ = coe_1lf_v * (2.0f*vx - wz*dl)/2.0f + (1 - coe_1lf_v) * last_cmd_vl;
//       vl = cmd_vl_ * 1000;
//       cmd_vr_ = coe_1lf_v * (2.0f*vx + wz*dl)/2.0f + (1 - coe_1lf_v) * last_cmd_vr;
//       vr = cmd_vr_ * 1000;

//     //    ROS_INFO("cmd_vl_: %f, cmd_vr_: %f\n", cmd_vl_, cmd_vr_);
//     }

//     last_cmd_vl = cmd_vl_;
//     last_cmd_vr = cmd_vr_;
// /* --------------------------------------------------------------------------*/


    // ROS_INFO_THROTTLE(4,"vr:%d,vl:%d,vx:%f,wz:%f",vr,vl,vx,wz);
    if(base_bridge_ != NULL)
    {
        base_bridge_->NavyMCUMoveCtrl(vl,vr,600,600);
    }
    return;
}
void Node::PublishOdom(const ros::WallTimerEvent& event)
{
    if(base_bridge_ == NULL)
        return;
	OtherSensorData_S  rawOdom;

    if(base_bridge_->GetSensorData(&rawOdom) == false)
    {
	//ROS_WARN_THROTTLE(2,"cannot receive the odom from mcu");
        return;
   }


    if(gResetFlag == true)
    {
	lastReading_.LDist = rawOdom.s32LDistance;
	lastReading_.RDist = rawOdom.s32RDistance;
	lastReading_.Gyro = rawOdom.s32IMU;

    last_odom_ = ros::Time::now();
    // last_odom_sec = ros::Time::now().toSec();
    vec_odo_t.clear();
    vec_wz.clear();
    vec_odo_t.push_back(last_odom_);
    vec_wz.push_back(rawOdom.s32IMU);

        gResetFlag = false;
        return;
    }

    float  vL = rawOdom.s32LSpeed*M_PI*wheel_radius/60.0/decel_ratio*2.0;
    float vR = rawOdom.s32RSpeed*M_PI*wheel_radius/60.0/decel_ratio*2.0;

    odom_vl_ = vL;
    odom_vr_ = vR;

   // ROS_WARN_THROTTLE(3,"rawOdom_vl , rawOdom_vR:%d,%d,;%f,%f",rawOdom.vL,rawOdom.vR,vL,vR);
    float vx_b = (vL + vR)/2.0 ;
    float wz_b = (vR - vL)/wheel_base_dist;//rawOdom.wz;//(vL - vR)/wheel_base_dist;
    // ROS_INFO("--------wz_b origin: %f\n", wz_b);

    ros::Time new_ = ros::Time::now();
    // double new_sec = ros::Time::now().toSec();

    float dL = float(rawOdom.s32LDistance - lastReading_.LDist)/1000.0;
    float dR = float(rawOdom.s32RDistance - lastReading_.RDist)/1000.0;

    double deltaYaw = double(rawOdom.s32IMU - lastReading_.Gyro)/100.0;

   /* print vel timestamp in prt_vel msg by vel_prt Publisher, add in 2018/05/29 by Qiu */
    std_base_move::vel_pub vp;
    vp.twist_cmd.linear.x = _vx;
    // vp.twist_cmd.angular.z = _wz * 0.6;
    vp.twist_cmd.angular.z = _wz;
    vp.header.stamp = ros::Time::now();
    vp.twist_odom.linear.x = vx_b;
   // vp.twist_odom.linear.y = wz_b * 0.6;//calculated by velocity
    vp.cmd_vl = cmd_vl_;
    vp.cmd_vr = cmd_vr_;

    vp.odom_vl = odom_vl_ ;
    vp.odom_vr = odom_vr_ ;



    /* using odom output wz_b */
    wz_b = rawOdom.s32IMU/10000.0;//raw angular

    //  vp.twist_odom.angular.z  = wz_b;
    /* angular filter */
    vp.twist_odom.angular.x = wz_b;//raw

    wz_b = angular_filter(wz_b);
    vp.twist_odom.angular.z = wz_b;
    // vp.cmd_vl = acc_plus;
    // vp.cmd_vr = acc_abs_plus;

    vp_pub_.publish(vp);





     deltaYaw =-1*DEG2RAD(deltaYaw);


     float deltaS = (dL+dR)/2.0;
     float deltaX = deltaS*std::cos(deltaYaw);
     float deltaY = deltaS*std::sin(deltaYaw);

     float dx_world = std::cos(theta_)*deltaX - std::sin(theta_)*deltaY;
     float dy_world = std::sin(theta_)*deltaX + std::cos(theta_)*deltaY;

     float vx_world = std::cos(theta_)*vx_b;
     float vy_world = std::sin(theta_)*vx_b;

     xx_ += dx_world; yy_ += dy_world; theta_ += deltaYaw;


     //publish tf
     geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(theta_);
     geometry_msgs::TransformStamped odom_trans;
     odom_trans.header.stamp = ToROSTime(rawOdom.s32Timestamp);
     odom_trans.header.frame_id = "odom";
     odom_trans.child_frame_id = "base_link";
     odom_trans.transform.translation.x = xx_;
     odom_trans.transform.translation.y = yy_;
     odom_trans.transform.translation.z = 0.0;
     odom_trans.transform.rotation = yaw_quat;

     //publish odometry
     nav_msgs::Odometry odometry;
     odometry.header.stamp = ToROSTime(rawOdom.s32Timestamp);
     odometry.header.frame_id = "odom";

     odometry.pose.pose.position.x = xx_;
     odometry.pose.pose.position.y = yy_;
     odometry.pose.pose.position.z = 0.0;
     odometry.pose.pose.orientation = yaw_quat;

     odometry.child_frame_id = "base_link";
     odometry.twist.twist.linear.x = vx_b;
     odometry.twist.twist.linear.y = 0;
     odometry.twist.twist.linear.z = 0;

     odometry.twist.twist.angular.x = 0;
     odometry.twist.twist.angular.y = 0 ;
     odometry.twist.twist.angular.z = wz_b;



     odom_pub_.publish(odometry);
     odom_broadcaster_.sendTransform(odom_trans);
     lastReading_.LDist = rawOdom.s32LDistance;
     lastReading_.RDist = rawOdom.s32RDistance;
     lastReading_.Gyro = rawOdom.s32IMU;

}

void Node::spinForever()
{
	tf2_ros::StaticTransformBroadcaster br_static;
	tf2::Quaternion qua( tf2::Quaternion(1, 0, 0, 0));
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.child_frame_id = "base_laser";
	transformStamped.header.frame_id = "base_link";
	transformStamped.transform.translation.x = -0.08;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.w = qua.getW();
	transformStamped.transform.rotation.x = qua.getX();
	transformStamped.transform.rotation.y = qua.getY();
	transformStamped.transform.rotation.z = qua.getZ();
	transformStamped.header.stamp = ToROSTime(BaseGetTimeTick());
	br_static.sendTransform(transformStamped);




    ros::spin();
}

void Node::test_vel(const ros::WallTimerEvent& event)
{
    ros::NodeHandle n;
    ros::Publisher vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.1;
    cmd_vel.linear.y = 0.0;
    if(count_ > 250) {c_wz *= -1; count_ = 0;}
    cmd_vel.angular.z = c_wz;
    count_++;

    vel_publisher.publish(cmd_vel);
}

int  main(int argc, char **argv)
{
    ros::init(argc,argv,"base_move_node");
    Node nn;

   if( nn.initialize()== false)
   {
       ROS_INFO("initial failed");
       return 1;
   }
    nn.spinForever();

    return 0;
}
