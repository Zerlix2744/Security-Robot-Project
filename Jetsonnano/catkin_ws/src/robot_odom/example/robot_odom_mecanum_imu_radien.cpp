#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"

float x = 0;
float y = 0;
float th = 0;

float dist_left;
float dist_right;

float enFL , enFR ,enBL , enBR;
float enFL_old , enFR_old ,enBL_old , enBR_old;


double radFL = 0;
double radFR = 0;
double radBL = 0;
double radBR = 0;

float yaw, vyaw, last_yaw;
double DistancePerCount = (3.14159265 * 0.15) / 4000;

void callbackEncoder(const geometry_msgs::Twist::ConstPtr &odom_msg)
{
    radFL  = -(odom_msg->linear.x);
    radFR = (odom_msg->linear.y);
    radBL  = -(odom_msg->angular.x);
    radBR = (odom_msg->angular.y);
}

void callbackYaw(const geometry_msgs::Twist::ConstPtr &yaw_msg)
{
    yaw = yaw_msg->linear.x;
    vyaw = yaw_msg->linear.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber sub_left = n.subscribe("ard_encoder", 1000, callbackEncoder);
    ros::Subscriber sub_left2 = n.subscribe("ard_imu", 1000, callbackYaw);

    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10.0);
    while (n.ok())
    {

        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();

    


        // forward kinematic
        double rr = 0.075;
        double lx = 0.195;
        double ly = 0.21;

        double lin_vel_x = (rr / 4) * (radFL + radFR + radBL + radBR);
        double lin_vel_y = (rr / 4) * (-radFL + radFR + radBL - radBR);
        double ang_vel = (rr / 4) * (1 / (lx + ly)) * (-radFL + radFR - radBL + radBR);

        double rota = yaw - last_yaw;

        // update old en value
        last_yaw = yaw;

        // Find dt time of ros   ? / time.sec
        double dt = (current_time - last_time).toSec();
        double delta_x = lin_vel_x * dt;
        double delta_y = lin_vel_y * dt;

        double delta_theta = ang_vel * dt;

        // i guess it would be rotation matix answer = Vx , Vy
        double rota_x = (delta_x * cos(th)) - ( delta_y * sin(th) ) ;
        double rota_y = (delta_x * sin(th)) + ( delta_y * cos(th) ) ;
        // it can be Vw
        double rota_th = rota;

        // compute odometry in a typical way given the velocities of the robot
        // S of Sx , Sy , Sw or Sth
        x += rota_x;
        y += rota_y;
        // yaw = Sth by defaut
        th = yaw;

        // since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        // set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = rota_x ;
        odom.twist.twist.linear.y = rota_y ;
        odom.twist.twist.angular.z = rota_th ;

        // publish the message
        odom_pub.publish(odom);

        last_time = current_time;

        r.sleep();
    }
}