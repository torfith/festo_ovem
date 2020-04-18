#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <string>

#include <ethercat/master.h>
#include <iolink/master.h>
#include <festo_ovem/festo_ovem.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "festo_ovem");
  ros::NodeHandle nh;

  std::string ifName;
  nh.getParam("/festo_ovem_node/interface_name", ifName);

  ethercat::Master em;
  em.monitor(); // create thread to handle slave errors in OP
  if (!em.open(ifName))
  {
    ROS_FATAL("No socket connection on interface %s. Use launch-prefix=\"ethercat_grant\".\n", ifName.c_str());
    ros::shutdown();
  }

  iolink::Master im;
  im.open(); // autodetect IO-Link master
  if (!im.open())
  {
    ROS_FATAL("IO-Link master not found.\n");
    ros::shutdown();
  }

  iolink::PortName portName = iolink::PortName::X01;
  iolink::Port port = im.port(portName);
  FestoOvem ovem(port);
 
  ros::Publisher isOutA_pub = nh.advertise<std_msgs::Bool>("is_out_a", 1);
  ros::Publisher isOutB_pub = nh.advertise<std_msgs::Bool>("is_out_b", 1);
  ros::Publisher pressure_pub = nh.advertise<std_msgs::Float32>("pressure", 1);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    if (em.spinOnce())
    {
      if (port.isDeviceAvailable())
      {
        if (port.isInputValid())
        {
          std_msgs::Bool isOutA_msg;
          isOutA_msg.data = ovem.isOutA();
          isOutA_pub.publish(isOutA_msg);

          std_msgs::Bool isOutB_msg;
          isOutB_msg.data = ovem.isOutB();
          isOutB_pub.publish(isOutB_msg);

          std_msgs::Float32 pressure_msg;
          pressure_msg.data = ovem.pressureBar();
          pressure_pub.publish(pressure_msg);

          ROS_INFO("isOutA: %d\n", ovem.isOutA());
          ROS_INFO("isOutB: %d\n", ovem.isOutB());
          ROS_INFO("pressureBar: %f", pressure_msg.data);
        }
      }
      else
        ROS_ERROR("IO-Link Port X0%d: Device not available.\n", portName + 1);

      if (port.isDeviceError())
        ROS_WARN("IO-Link Port X0%d: %s.\n", portName + 1, port.errorString().c_str());
    }
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}