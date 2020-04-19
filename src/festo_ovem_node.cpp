#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <string>

#include <ethercat/master.h>
#include <iolink/master.h>
#include <festo_ovem/festo_ovem.h>

static bool setEjectionOn = false;
static bool setSuctionOn = false;

void setEjectionOn_callback(const std_msgs::BoolConstPtr& set_ejection_on)
{
  setEjectionOn = set_ejection_on->data;
}

void setSuctionOn_callback(const std_msgs::BoolConstPtr& set_suction_on)
{
  setSuctionOn = set_suction_on->data;
}

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
    ROS_FATAL_STREAM("No socket connection on interface " << ifName << ". Use launch-prefix=\"ethercat_grant\".");
    ros::shutdown();
  }

  iolink::Master im;
  im.open(); // autodetect IO-Link master
  if (!im.open())
  {
    ROS_FATAL_STREAM("IO-Link master not found.");
    ros::shutdown();
  }

  iolink::PortName portName = iolink::PortName::X01;
  iolink::Port port = im.port(portName);
  FestoOvem ovem(port);
 
  ros::Publisher isOutA_pub = nh.advertise<std_msgs::Bool>("is_out_a", 1);
  ros::Publisher isOutB_pub = nh.advertise<std_msgs::Bool>("is_out_b", 1);
  ros::Publisher pressure_pub = nh.advertise<std_msgs::Float32>("pressure", 1);
  ros::Subscriber setEjectionOn_sub = nh.subscribe<std_msgs::Bool>("set_ejection_on", 1, setEjectionOn_callback);
  ros::Subscriber setSuctionOn_sub = nh.subscribe<std_msgs::Bool>("set_suction_on", 1, setSuctionOn_callback);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ROS_DEBUG_STREAM("setEjectionOn: " << setEjectionOn);
    ROS_DEBUG_STREAM("setSuctionOn: " << setSuctionOn);

    if (setEjectionOn)
      ovem.setEjectionOn();
    else
      ovem.setEjectionOff();

    if (setSuctionOn)
      ovem.setSuctionOn();
    else
      ovem.setSuctionOff();

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

          ROS_DEBUG_STREAM("isOutA: " << ovem.isOutA());
          ROS_DEBUG_STREAM("isOutB: " << ovem.isOutB());
          ROS_DEBUG_STREAM("pressureBar: " << pressure_msg.data);
        }
      }
      else
        ROS_ERROR_STREAM_THROTTLE(1,"IO-Link Port X0" << portName+1 << ": Device not available.");

      if (port.isDeviceError())
        ROS_WARN_STREAM("IO-Link Port X0" << portName+1 << ": " << port.errorString());
    }
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}