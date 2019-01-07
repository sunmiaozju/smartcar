/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "ros_gpio_control/gpio.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hall_sensor");
  int gpio_number;
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.param("gpio_number", gpio_number, 296);
  GPIO hall_sensor(gpio_number);
  hall_sensor.edge(GPIO::EDGE_BOTH);
  ros::Publisher hall_sensor_pub = n.advertise<std_msgs::Bool>("hall_sensor", 10);
  ros::Rate loop_rate(60);

  while (ros::ok())
  {
    hall_sensor.poll(100);
    std_msgs::Bool msg;
    msg.data = hall_sensor;
    hall_sensor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
