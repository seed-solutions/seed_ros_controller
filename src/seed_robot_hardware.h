#ifndef _SEED_ROBOT_HW_H_
#define _SEED_ROBOT_HW_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include "seed_solutions_sdk/seed3_command.h"

#include <mutex>

namespace seed_robot_hardware
{
  class SeedRobotHW : public hardware_interface::RobotHW
  {
  public:
    SeedRobotHW(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    ~SeedRobotHW();

    ros::Time getTime() const { return ros::Time::now(); }
    double getPeriod() { return ((double)CONTROL_PERIOD_US_) / (1000 * 1000); }

    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

  private:
    int   CONTROL_PERIOD_US_;
    std::mutex mutex_;
    int number_of_joints_;
    int joint_mode_; // position, velocity, or effort

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    joint_limits_interface::PositionJointSaturationInterface position_joint_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface position_joint_soft_limits_interface_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;

    //-- for seed driver --
    seed::controller::SeedCommand *seed_;
    bool is_seed_;

    void initSeedDriver(ros::NodeHandle &robot_hw_nh);

    struct SeedDriver {
      std::vector<std::string> name;
      std::vector<int> id;
      std::vector<std::string> type;
      std::vector<double> pulse_ratio;
      bool is_connected;
    }seed_driver;

    std::map<std::string, double> pulse_ratio_;
  };
}

#endif // #ifndef _SEED_ROBOT_HW_H_
