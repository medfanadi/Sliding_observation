#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
 
#include <iostream>
 
ros::Publisher pub;
bool airborne;
 
// Forces callback function
void forcesCb(ConstContactsPtr &_msg){
    geometry_msgs::Vector3 msgForce;
    // What to do when callback
    if ( airborne == false ) {
        //std::cout << "Collision but not airborn" <<  '\n';
        msgForce.x = 0;
        msgForce.y = 0;
        msgForce.z = 0;
    } else if ( _msg->contact_size() !=0 ) {
        //std::cout << "Collision" << '\n';
        msgForce.x = _msg->contact(0).wrench().Get(0).body_1_wrench().force().x();
        msgForce.y = _msg->contact(0).wrench().Get(0).body_1_wrench().force().y();
        msgForce.z = _msg->contact(0).wrench().Get(0).body_1_wrench().force().z();
    } else {
        //std::cout << "No collision" << '\n';
        msgForce.x = 0;
        msgForce.y = 0;
        msgForce.z = 0;
    }
    pub.publish(msgForce);
}
 
// Position callback function
void positionCb(const nav_msgs::Odometry::ConstPtr& msg2){
    if (msg2->pose.pose.position.z > 0.3) {
        airborne = true;
    } else {
        airborne = false;
    }
}
 
int main(int _argc, char **_argv){
  // Set variables
  airborne = false;
 
  // Load Gazebo & ROS
  gazebo::setupClient(_argc, _argv);
  ros::init(_argc, _argv, "talker");
 
  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
 
  // Create ROS node and init
  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::Vector3>("forces", 1000);
 
  // Listen to Gazebo contacts topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/quadrotor/base_link/quadrotor_bumper/contacts", forcesCb);
 
  // Listen to ROS for position
  ros::Subscriber sub2 = n.subscribe("/ground_truth/state", 1000, positionCb);
 
  // Busy wait loop
  while (true) {
    gazebo::common::Time::MSleep(20);
    ros::spinOnce();
  }
 
  // Mayke sure to shut everything down.
  gazebo::shutdown();
}
