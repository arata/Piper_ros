#include <iostream>
#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <sensor_msgs/JointState.h>

class PiperRobot : public hardware_interface::RobotHW
{
public:
  PiperRobot(ros::NodeHandle& nh) 
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_joint1("joint1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_joint1);

    hardware_interface::JointStateHandle state_handle_joint2("joint2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_joint2);

    hardware_interface::JointStateHandle state_handle_joint3("joint3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_joint3);

    hardware_interface::JointStateHandle state_handle_joint4("joint4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_joint4);

    hardware_interface::JointStateHandle state_handle_joint5("joint5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_joint5);

    hardware_interface::JointStateHandle state_handle_joint6("joint6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_joint6);

    hardware_interface::JointStateHandle state_handle_joint7("joint7", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(state_handle_joint7);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_joint1(jnt_state_interface.getHandle("joint1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_joint1);

    hardware_interface::JointHandle pos_handle_joint2(jnt_state_interface.getHandle("joint2"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_joint2);

    hardware_interface::JointHandle pos_handle_joint3(jnt_state_interface.getHandle("joint3"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_joint3);

    hardware_interface::JointHandle pos_handle_joint4(jnt_state_interface.getHandle("joint4"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_joint4);

    hardware_interface::JointHandle pos_handle_joint5(jnt_state_interface.getHandle("joint5"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_joint5);

    hardware_interface::JointHandle pos_handle_joint6(jnt_state_interface.getHandle("joint6"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_joint6);

    hardware_interface::JointHandle pos_handle_joint7(jnt_state_interface.getHandle("joint7"), &cmd[6]);
    jnt_pos_interface.registerHandle(pos_handle_joint7);

    registerInterface(&jnt_pos_interface);

    // Initialize publisher
    joint_ctrl_pub = nh.advertise<sensor_msgs::JointState>("joint_ctrl_single", 10);
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t j = 0; j < 7; ++j) {
        if (msg->name[i] == joint_names[j]) {
          pos[j] = msg->position[i];
          vel[j] = msg->velocity[i];
          eff[j] = msg->effort[i];
        }
      }
    }
  }

  void read() {
    std::cout << "Reading joint states..." << std::endl;
  }

  void write() {
    std::cout << "Writing commands: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << cmd[i] << (i < 6 ? ", " : "]\n");
    }

    // Publish the command as a JointState message
    sensor_msgs::JointState joint_ctrl_msg;
    joint_ctrl_msg.header.stamp = ros::Time::now();
    joint_ctrl_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"};
    joint_ctrl_msg.position = {cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[6]};
    joint_ctrl_pub.publish(joint_ctrl_msg);
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[7] = {0};
  double pos[7] = {0};
  double vel[7] = {0};
  double eff[7] = {0};
  const std::string joint_names[7] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

  ros::Publisher joint_ctrl_pub;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "piper_control_node");
  ros::NodeHandle nh;

  PiperRobot robot(nh);
  controller_manager::ControllerManager cm(&robot);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &PiperRobot::jointStateCallback, &robot);

  ros::Time prev_time = ros::Time::now();
  ros::Time time;
  ros::Duration period;
  
  ros::Rate rate(100);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    ros::spinOnce();

    time = ros::Time::now();
    period = time - prev_time;
    prev_time = time;

    robot.read();
    cm.update(time, period);
    robot.write();

    rate.sleep();
  }
  spinner.stop();
  
  return 0;
}
