#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <sensor_msgs/JointState.h>
#include <intera_core_msgs/JointCommand.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_sub(nh,"joint_states",1);
  ros::Publisher intera_cmd_pub=nh.advertise<intera_core_msgs::JointCommand>("joint_command",1);
  ros::Publisher js_cmd_pub=nh.advertise<sensor_msgs::JointState>("joint_target",1);

  ROS_INFO("waiting joint states...");
  if (!js_sub.waitForANewData(ros::Duration(10)))
  {
    ROS_ERROR("No joint states messages...");
    return 0;
  }
  sensor_msgs::JointState msg=js_sub.getData();
  unsigned int ndof=msg.name.size();
  ROS_INFO("received a joint states with %u joints:",ndof);

  intera_core_msgs::JointCommand cmd;
  cmd.mode=cmd.TRAJECTORY_MODE;
  cmd.names=msg.name;
  cmd.position=msg.position;
  for (unsigned int idx=0;idx<ndof;idx++)
  {
    std::cout << msg.name.at(idx) << " position: " << msg.position.at(idx) << std::endl;
  }

  std::string controlled_joint="ur5_elbow_joint";
  double amplitude=0.1;
  double period=3;
  double repetitions=1;
  double sampling_period=1e-3;

  if (!nh.getParam("controlled_joint",controlled_joint))
  {
    ROS_ERROR("controlled_joint is not set");
    return 0;
  }
  if (!nh.getParam("amplitude",amplitude))
  {
    ROS_ERROR("amplitude is not set");
    return 0;
  }
  if (!nh.getParam("period",period))
  {
    ROS_ERROR("period is not set");
    return 0;
  }
  if (!nh.getParam("repetitions",repetitions))
  {
    ROS_ERROR("repetitions is not set");
    return 0;
  }
  if (!nh.getParam("sampling_period",sampling_period))
  {
    ROS_ERROR("sampling_period is not set");
    return 0;
  }
  bool found=false;
  unsigned int joint_idx=0;
  for (unsigned int idx=0;idx<ndof;idx++)
  {
    if (!msg.name.at(idx).compare(controlled_joint))
    {
      joint_idx=idx;
      found=true;
    }
  }
  if (!found)
  {
    ROS_ERROR("controlled joint %s is not member of joint_states",controlled_joint.c_str());
    return 0;
  }

//  ros::WallRate lp(1.0/sampling_period);
  ros::WallRate lp(1000);
  double omega=2*M_PI/period;
  ros::WallTime t0=ros::WallTime::now();
  double test_time=(period*repetitions);
  double pos0=msg.position.at(joint_idx);
  msg.velocity.resize(ndof);
  std::fill(msg.velocity.begin(),msg.velocity.end(),0);
  msg.effort.resize(ndof);
  std::fill(msg.effort.begin(),msg.effort.end(),0);


  cmd.velocity.resize(ndof,0);
  cmd.acceleration.resize(ndof,0);
  cmd.effort.resize(ndof,0);


  double t=0;
  while (ros::ok() && t<=test_time)
  {
    sensor_msgs::JointState fb=js_sub.getData();
    t=(ros::WallTime::now()-t0).toSec();
    double pos=pos0+0.5*amplitude*(1-std::cos(omega*t));
    double vel=0.5*amplitude*omega*sin(omega*t);
    double acc=0.5*amplitude*std::pow(omega,2)*cos(omega*t);
    msg.position.at(joint_idx)=pos;
    msg.velocity.at(joint_idx)=vel;

    cmd.position.at(joint_idx)=pos;
    cmd.velocity.at(joint_idx)=vel;
    cmd.acceleration.at(joint_idx)=acc;

//    cmd.velocity.at(joint_idx)=vel;
    msg.header.stamp=cmd.header.stamp=fb.header.stamp;
    intera_cmd_pub.publish(cmd);
    js_cmd_pub.publish(msg);
    lp.sleep();
    ros::spinOnce();
  }
  msg.position.at(joint_idx)=pos0;
  msg.velocity.at(joint_idx)=0;
  intera_cmd_pub.publish(msg);

  return 0;
}
