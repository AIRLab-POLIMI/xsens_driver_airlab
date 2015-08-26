//This node manages the IMU (xsens MTi)
//It interfaces with the device and publishes its measures on a topic
//Topic published:
// /magnetic  - Message: sensor_msgs/MagneticField
// /imu/data  - Message: sensor_msgs/Imu
//Parameters: device - path of the IMU to open and manage
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include "xSensImu/XsensDriver.hpp"

#define NAME_OF_THIS_NODE "xsensImu"

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Publisher PublisherMag, PublisherImu;
  xsens_imu::XsensDriver driver;
  std::string dev;
  int rate;
  unsigned long seq;
public:
  ROSnode();
  int getRate();
  bool Prepare();
  void getData();
};

ROSnode::ROSnode() :  Handle("~") {
  
}

int ROSnode::getRate(){
    return rate;
}

bool ROSnode::Prepare() {
  seq = 0;
  //Retrieve parameters
  if (Handle.getParam("device", dev)) {
    ROS_INFO("Node %s: retrieved parameter device.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter device.", ros::this_node::getName().c_str());
    return false;
  }
  
  
  if (Handle.getParam("rate", rate)) {
    ROS_INFO("Node %s: retrieved parameter rate.", ros::this_node::getName().c_str());
  }
  else {
    ROS_WARN("Node %s: unable to retrieve parameter rate. Setting default value to 20", ros::this_node::getName().c_str());
    rate = 20;
  }

  PublisherMag = Handle.advertise<sensor_msgs::MagneticField>("mag", 50);
  PublisherImu = Handle.advertise<sensor_msgs::Imu>("imu", 50);

  driver.setFrequency(rate);

  //Open and initialize the device
  if(!driver.open(dev) ||
     !driver.setReadingMode(xsens_imu::CAL_AND_ORI_DATA) ||
     !driver.setTimeout(100))
    return false;
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  return true;
}

void ROSnode::getData() {
  //Read measures from IMU
  int ret = driver.getReading();
  if(ret == xsens_imu::NO_ERROR) {
    sensor_msgs::MagneticField msg;
    msg.header.seq = seq;
    msg.header.frame_id = "/imu";
    msg.header.stamp = ros::Time::now();
    msg.magnetic_field.x = driver.getCalibratedMagData().x();
    msg.magnetic_field.y = driver.getCalibratedMagData().y();
    msg.magnetic_field.z = driver.getCalibratedMagData().z();
    boost::array<double, 9> covariance = {0};
    covariance[0] = 1;
    covariance[4] = 1;
    covariance[8] = 1;
    msg.magnetic_field_covariance = covariance;
    PublisherMag.publish(msg);
    sensor_msgs::Imu msgImu;
    msgImu.header.seq = seq;
    msgImu.header.frame_id = "/imu";
    msgImu.header.stamp = msg.header.stamp;
    msgImu.orientation.w = driver.getOrientation().w();
    msgImu.orientation.x = driver.getOrientation().x();
    msgImu.orientation.y = driver.getOrientation().y();
    msgImu.orientation.z = driver.getOrientation().z();
    msgImu.angular_velocity.x = driver.getCalibratedGyroData().x();
    msgImu.angular_velocity.y = driver.getCalibratedGyroData().y();
    msgImu.angular_velocity.z = driver.getCalibratedGyroData().z();
    msgImu.linear_acceleration.x = driver.getCalibratedAccData().x();
    msgImu.linear_acceleration.y = driver.getCalibratedAccData().y();
    msgImu.linear_acceleration.z = driver.getCalibratedAccData().z();
    PublisherImu.publish(msgImu);
    seq++;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  
  if(!node.Prepare())
    return 1;

  ros::Rate loopRate(node.getRate());
  
  while(ros::ok()) {
    node.getData();
//    ros::spinOnce();
    loopRate.sleep();
  }

  return (0);
}
