### 安装arduino
```
sudo apt-get install arduino
```
### 安装rosserial-arduino

用途：用于arduino的ROS通讯桥接

二进制方式安装在ROS工作站(推荐)
```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
rospack profile
```
源码方式安装在ROS工作站
```
cd <ws>/src
git clone https://github.com/ros-drivers/rosserial.git
cd <ws>
catkin_make
```

### 安装ros_lib到Arduino IDE开发环境
上面的安装会得到ros_lib,它需要复制到Arduino的开发环境，以启用Arduino与ROS通讯。

linux系统下，生成ros_lib到Linux下的Arduino库目录sketchbook/libraries(一般在用户的home目录下)
```
cd <sketchbook>/libraries
rm -rf ros_lib #如之前有，可以先删除
rosrun rosserial_arduino make_libraries.py .
```
然后，可以打开arduino IDE,找到file-examples-ros_lib是否存在，如果这一不没安装好的话，会找不到ros.h的头文件

### 运行ROS-arduino
写好arduino的代码之后，就可以试着运行看看

运行roscore
```
roscore
```

新终端运行，/dev/ttyUSB0为Arduino设备
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
或
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
```
设备可能是/dev/ttyUSB0,也可能是/dev/ttyACM0,需要自己确认
