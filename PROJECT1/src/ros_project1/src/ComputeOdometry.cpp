#include <ros/ros.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h> 
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h> 
#include "ros1/WheelSpeed.h" 
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include "ros1/integrationConfig.h" 
#include "ros1/reset_pos.h"
#include "ros1/reset_ticks.h"
#include "ros1/calibration.h"

class Pub_sub_odometry {

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    ros::Publisher vel_pub; 
    ros::Subscriber vel_sub; 
    ros::Publisher wheel_speed_pub;
    ros::ServiceServer resetService;
    ros::ServiceServer resetServiceTicks;
    ros::ServiceServer setCalibration;

    ros::Time lastTime;
    double deltaT; 
    double x,y,th;
    double ticks_fl_prev, ticks_fr_prev, ticks_rr_prev, ticks_rl_prev;
    double r, l, w;
    int T, N;
   
    tf2_ros::TransformBroadcaster odom_broadcaster; 

    int integrationType;
    

public:
       
    Pub_sub_odometry() {
        sub = n.subscribe("/wheel_states", 1, &Pub_sub_odometry::read_vel, this);  // to read the velocities from bag
        vel_pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1); // to publish velocities
        odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1); // to publish the odometry on topic /odom (as requested)
        vel_sub=n.subscribe("/cmd_vel", 1, &Pub_sub_odometry::computeControl, this); // to read the msg published on topic cmd_vel and compute the odometry
        wheel_speed_pub=n.advertise<ros1::WheelSpeed>("/wheels_rpm", 1); // to publish the computed wheel speeds on topic wheels_rpm
        resetService = n.advertiseService("reset_pos" , &Pub_sub_odometry::resetPos, this); // to reset initial pose
        resetServiceTicks = n.advertiseService("reset_ticks" , &Pub_sub_odometry::resetTicks, this); // to reset initial ticks
        setCalibration = n.advertiseService("calibration" , &Pub_sub_odometry::Calibration, this); // to set calibration parameters
        lastTime = ros::Time::now();

        //Initial Variables
        r= 0.07;  // wheel radius
        l= 0.2;   // wheel position along x
        w=0.169;  // wheel position along y
        T=5;      // gear ratio
        N=42;     // CPR encoder

        // std initial ticks values (from bag1, use the proper service command for the other bags)
        
        ticks_fl_prev=17313.0;
        ticks_fr_prev=11359.0;
        ticks_rr_prev=13209.0;
        ticks_rl_prev=15421.0;

        //Reads param from launch file
        n.getParam("/InitialX", x);
        n.getParam("/InitialY", y);
        n.getParam("/InitialTheta", th);
        integrationType = 0; // Default Euler 
    }
    

    // callback for sub defined before
    void read_vel(const sensor_msgs::JointState::ConstPtr& msg){
       
        double dt;
        double ticks_fl, ticks_fr, ticks_rr, ticks_rl;
        double dticks_fl, dticks_fr, dticks_rr, dticks_rl;
        double rpm_fl, rpm_fr, rpm_rr, rpm_rl;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg->header.stamp;

        //Computes dt from last message
        dt = (currentTime - lastTime).toSec();
        deltaT=dt;

        // Reads ticks from msg type sensor_msgs/JointState
        

        ticks_fl=msg->position[0];
        ticks_fr=msg->position[1];
        ticks_rl=msg->position[2];
        ticks_rr=msg->position[3];
        

    // Computes the delta_ticks
        dticks_fl=ticks_fl-ticks_fl_prev;
        dticks_fr=ticks_fr-ticks_fr_prev;
        dticks_rr=ticks_rr-ticks_rr_prev;
        dticks_rl=ticks_rl-ticks_rl_prev;
        

        // update
        ticks_fl_prev=ticks_fl;
        ticks_fr_prev=ticks_fr;
        ticks_rr_prev=ticks_rr;
        ticks_rl_prev=ticks_rl;

        // Convert ticks (more accurate) into rpm 
        rpm_fl=2*M_PI*dticks_fl/dt/N/T;
        rpm_fr=2*M_PI*dticks_fr/dt/N/T;
        rpm_rl=2*M_PI*dticks_rl/dt/N/T;
        rpm_rr=2*M_PI*dticks_rr/dt/N/T;         

        // (If you want to use the given rpm, but less precise)
        /*rpm_fl=(msg->velocity[0])/60/T;
        rpm_fr=(msg->velocity[1])/60/T;
        rpm_rl=(msg->velocity[2])/60/T;
        rpm_rr=(msg->velocity[3])/60/T;*/


        //Publish cmd_vel (publishing the velocities)
        publishVel(rpm_fl,rpm_fr,rpm_rr,rpm_rl, dt, currentTime);
        

        //Updates last time
        lastTime= currentTime;
    }
    
    // to publish Vx, Vy, Wz
    void publishVel(double rpm_fl, double rpm_fr, double rpm_rr, double rpm_rl, double dt, ros::Time currentTime){
     
        geometry_msgs::TwistStamped vel;
        double Vx=r/4*(rpm_fl+rpm_fr+rpm_rl+rpm_rr);
        double Vy=r/4*(-rpm_fl+rpm_fr+rpm_rl-rpm_rr);
        double Wz=r/4*(-rpm_fl+rpm_fr-rpm_rl+rpm_rr)/(l+w);
        dt = (currentTime - lastTime).toSec();
        // Publishing:
        vel.header.stamp = currentTime;
        vel.header.frame_id = "vel";
        vel.twist.linear.x=Vx;
        vel.twist.linear.y=Vy;
        vel.twist.linear.z=0.0;
        vel.twist.angular.x=0.0;
        vel.twist.angular.y=0.0;
        vel.twist.angular.z=Wz;
        vel_pub.publish(vel);
        
        //Updates last time
        lastTime= currentTime;

    }
    

    // to Compute from control variables the rpm of the wheels (also to compute odometry)
    void computeControl(const geometry_msgs::TwistStamped::ConstPtr& msg){
        double rpm_fl, rpm_fr, rpm_rr, rpm_rl;
        double Vx, Vy, Wz;
        ros::Time currentTime = msg->header.stamp;
        double dt=deltaT; 
        
        Vx=msg->twist.linear.x;
        Vy=msg->twist.linear.y;
        Wz=msg->twist.angular.z;


        //Integration 
        if(integrationType == 0) { //EULER
            x += (Vx * cos(th) - Vy * sin(th)) * dt;
            y += (Vx * sin(th) + Vy * cos(th)) * dt;
            th += Wz * dt;
        }
        else if(integrationType == 1){ //RUNGE-KUTTA
            x += (Vx * cos(th + Wz * dt / 2) - Vy * sin(th + Wz * dt / 2)) * dt;
            y += (Vx * sin(th + Wz * dt / 2) + Vy * cos(th + Wz * dt / 2)) * dt;
            th += Wz * dt;        
        }
        

        rpm_fl=60*T*(Vx-Vy-(l+w)*Wz)/r;
        rpm_fr=60*T*(Vx+Vy+(l+w)*Wz)/r;
        rpm_rr=60*T*(Vx-Vy+(l+w)*Wz)/r;
        rpm_rl=60*T*(Vx+Vy-(l+w)*Wz)/r;
        
        //Publish odometry message and tf2
        publishOdometry(Vx, Vy, Wz, currentTime);
        publishTfTransformation(x, y, th, currentTime);  
        publishWheelSpeed(rpm_fl, rpm_fr, rpm_rr, rpm_rl, currentTime);
        //Updates last time
        lastTime= currentTime;

    }

    // to publish computed wheels speed
    void publishWheelSpeed(double rpm_fl,double rpm_fr,double rpm_rr,double rpm_rl,ros::Time currentTime){
        ros1::WheelSpeed wheelSpeed;

        // Publishing
        wheelSpeed.header.stamp=currentTime;
        wheelSpeed.header.frame_id="Wheel_speed";
        wheelSpeed.rpm_fl=rpm_fl;
        wheelSpeed.rpm_fr=rpm_fr;
        wheelSpeed.rpm_rr=rpm_rr;
        wheelSpeed.rpm_rl=rpm_rl;
        wheel_speed_pub.publish(wheelSpeed);
    }

   
    // to publish odometry 
    void publishOdometry(double Vx,double Vy,double Wz, ros::Time currentTime){
        nav_msgs::Odometry odometry;
        tf2::Quaternion q;
        geometry_msgs::Quaternion quat_msg; 
        q.setRPY(0, 0, th);
        quat_msg = tf2::toMsg(q); 
        
       
       // PUBLISHING:

        //set header
        odometry.header.stamp = currentTime;
        odometry.header.frame_id = "odom";
        //set pose
        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = quat_msg;
        //set the velocity
        odometry.child_frame_id = "base_link";
        odometry.twist.twist.linear.x = Vx;
        odometry.twist.twist.linear.y = Vy;
        odometry.twist.twist.angular.z = Wz;
       
        //publish odometry
        odom_pub.publish(odometry);
        
    }

    // tf2
    void publishTfTransformation(double x, double y, double th,ros::Time currentTime){
        geometry_msgs::TransformStamped odometryTransformation;
        tf2::Quaternion q;
        geometry_msgs::Quaternion quat_msg; 
        q.setRPY(0, 0, th);
        quat_msg = tf2::toMsg(q); 


        //PUBLISHING:

        //set header
        odometryTransformation.header.stamp = currentTime;
        odometryTransformation.header.frame_id = "odom";
        odometryTransformation.child_frame_id = "base_link";
        //set transformation
        odometryTransformation.transform.translation.x = x;
        odometryTransformation.transform.translation.y = y;
        odometryTransformation.transform.translation.z = 0;
        odometryTransformation.transform.rotation =quat_msg;

        //publish transformation
        odom_broadcaster.sendTransform(odometryTransformation);
    }

    // configure integration type
    void setIntegration(ros1::integrationConfig &config){
        integrationType = config.integration;
    }

    
    // reset initial pose
    bool resetPos(ros1::reset_pos::Request  &req,
               ros1::reset_pos::Response &res)
    {
        x = req.x;
        y = req.y;
        th = req.th;
        ROS_INFO("x reset to: %f", x);
        ROS_INFO("y reset to: %f", y);
        ROS_INFO("th reset to: %f", th);
        return true;
    }

    // reset initial ticks
    bool resetTicks(ros1::reset_ticks::Request  &req,
               ros1::reset_ticks::Response &res)
    {
        ticks_fl_prev = req.ticks_fl_prev;
        ticks_fr_prev = req.ticks_fr_prev;
        ticks_rl_prev = req.ticks_rl_prev;
        ticks_rr_prev = req.ticks_rr_prev;

        return true;
    }

    // set different parameters for calibration
    bool Calibration(ros1::calibration::Request  &req,
               ros1::calibration::Response &res)
    {
       r = req.r;  // wheel radius
       l = req.l;   // wheel position along x
       w = req.w;  // wheel position along y
       T = req.T;      // gear ratio
       N = req.N;     // CPR encoder

        return true;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ComputeOdometry");

    Pub_sub_odometry pubSubOdometry;

    
    dynamic_reconfigure::Server<ros1::integrationConfig> server;
    dynamic_reconfigure::Server<ros1::integrationConfig>::CallbackType f;

    f = boost::bind(&Pub_sub_odometry::setIntegration, &pubSubOdometry, _1);
    server.setCallback(f);

    ros::spin();
    return 0;
}