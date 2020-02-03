#include <stdio.h>
#include <pigpio.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define SERVO_PIN 18
#define DC_PIN 17

#define SERVO_MID 1500
#define DC_MID 1500

void statusChangerGPIO(const geometry_msgs::Twist car_twist)
{

    gpioPWM(SERVO_PIN, SERVO_MID+car_twist.angular.z*800);
    gpioPWM(DC_PIN, DC_MID+car_twist.linear.x*200);

  ROS_INFO("Changed to ... angular: %f\tlinear: %f", car_twist.angular.z, car_twist.linear.x);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "indicator_node");
  ROS_INFO("Starting ... indicator node");
  ros::NodeHandle n;
  if (gpioInitialise() < 0){ROS_INFO("PiGPIO init Failed");} //pigpio initialization
    else
    {
      ROS_INFO("PiGPIO init SUCCESS");
      //gpioSetMode(SERVO_PIN, PI_OUTPUT);
      //gpioSetMode(DC_PIN, PI_OUTPUT);
      gpioSetPWMrange(SERVO_PIN, 20000);
      gpioSetPWMrange(DC_PIN, 20000);
      gpioSetPWMfrequency(SERVO_PIN, 50);
      gpioSetPWMfrequency(DC_PIN, 50);
      gpioPWM(SERVO_PIN, SERVO_MID);
      gpioPWM(DC_PIN, DC_MID);

    }
  ros::Subscriber sub = n.subscribe("cmd_vel", 10, statusChangerGPIO);

  ros::spin();

  return 0;
}
