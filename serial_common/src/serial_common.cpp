/****************************************
*    @author  : LD
*    @date    : 201780124
*    @ROS     : Kinetic
*    @Version : 1.0.0
****************************************/

#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include "Serial_common_config.h"

#ifdef INFRANTRY_MODE
  #include <serial_common/Infantry.h>
#endif
#ifdef GUARD_MODE
  #include <serial_common/Guard.h>
#endif

serial::Serial ser; //声明串口对象
#ifdef INFRANTRY_MODE
void write_callback(const serial_common::Infantry::ConstPtr& msg)
{
    uint8_t Buffer[10];
    Buffer[0] = msg-> kaishi;
    Buffer[1] = msg-> panduan;
    Buffer[2] = msg-> xlocation&0xff;
    Buffer[3] = msg-> xlocation>>8;
    Buffer[4] = msg-> ylocation&0xff;
    Buffer[5] = msg-> ylocation>>8;
    Buffer[6] = msg-> shijie_z&0xff;
    Buffer[7] = msg-> shijie_z>>8;
    Buffer[8] = msg-> fankui1;
    Buffer[9] = msg-> fankui2;

    ser.write(Buffer,10);   //发送串口数据
}
#endif

#ifdef GUARD_MODE
void write_callback(const serial_common::Guard::ConstPtr& msg)
{
    uint8_t Buffer[6];
    Buffer[0] = msg-> xlocation&0xff;
    Buffer[1] = msg-> xlocation>>8;
    Buffer[2] = msg-> ylocation&0xff;
    Buffer[3] = msg-> ylocation>>8;
    Buffer[4] = msg-> depth&0xff;
    Buffer[5] = msg-> depth>>8;
    ser.write(Buffer,6);   //发送串口数据
}
#endif

int main (int argc, char** argv)
{
  //初始化节点
  ros::init(argc, argv, "serial_common_node");
  //声明节点句柄
  ros::NodeHandle nh;
  //订阅主题，并配置回调函数
  ros::Subscriber write_sub = nh.subscribe("write", 33, write_callback);
#ifdef INFRANTRY_MODE
  //发布主题
  ros::Publisher read_pub = nh.advertise<std_msgs::String>("/serial/read", 33);
#endif

#ifdef GUARD_MODE
  //发布主题
  ros::Publisher read_pub = nh.advertise<std_msgs::String>("/serial/read", 33);
#endif

  try
  {
  //设置串口属性，并打开串口
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }
  //检测串口是否已经打开，并给出提示信息
  if(ser.isOpen())
  {
      ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
      return -1;
  }


  ros::Rate loop_rate(50);
  while(ros::ok())
  {
      if(ser.available()){
          ROS_INFO_STREAM("Reading from serial port\n");
          std_msgs::String result;
          result.data = ser.read(ser.available());
          ROS_INFO_STREAM("Read: " << result.data);
          read_pub.publish(result);
      }

      //处理ROS的信息，比如订阅消息,并调用回调函数
      ros::spinOnce();
      loop_rate.sleep();

  }



}
