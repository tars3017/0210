#include "mainpp.h"
#include "ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "STM32Hardware.h"

geometry_msgs::Twist pub_out_msg;
void sub_vel_cb(const geometry_msgs::Twist &msg)
{
   get_vel_x = msg.linear.x;
   get_vel_y = msg.linear.y;
   get_vel_z = msg.angular.z;
}

void callback(const std_msgs::Int64 &msg)
{
   count = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub_vel("/cmd_vel", sub_vel_cb);
ros::Publisher pub_vel("/base_speed", &pub_out_msg);
ros::Subscriber<std_msgs::Int64> sub("counting", callback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->flush();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->reset_rbuf();
}

void setup(void)
{
    nh.initNode();
//    sub_vel = nh.subscribe("/cmd_vel", 1, sub_vel_cb);
//    pub_vel = nh.advertise<geometry_msgs::Twist>("/base_speed", 1);
    nh.subscribe(sub_vel);
    nh.subscribe(sub);
}
void loop(void)
{
    nh.spinOnce();
}

void publish_vel(double x, double y, double z)
{

	pub_out_msg.linear.x = x;
	pub_out_msg.linear.y = y;
	pub_out_msg.angular.z = z;
	pub_vel.publish(&pub_out_msg);
}
