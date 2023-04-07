#include "mainpp.h"
#include "ros.h"
#include "std_msgs/Int64.h"
#include "STM32Hardware.h"

void callback(const std_msgs::Int64 &msg)
{
   count = msg.data;
}

ros::NodeHandle nh;
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
    nh.subscribe(sub);
}
void loop(void)
{
    nh.spinOnce();
}
