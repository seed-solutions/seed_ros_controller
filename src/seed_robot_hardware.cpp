#include "seed_robot_hardware.h"
#include <urdf/model.h>

namespace seed_robot_hardware
{
  SeedRobotHW::SeedRobotHW(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
  {
    seed_ = new seed::controller::SeedCommand();
    initSeedDriver(robot_hw_nh);
    
    // Resize vectors
    joint_position_.resize(number_of_joints_);
    joint_velocity_.resize(number_of_joints_);
    joint_effort_.resize(number_of_joints_);
    joint_position_command_.resize(number_of_joints_);
    joint_velocity_command_.resize(number_of_joints_);
    joint_effort_command_.resize(number_of_joints_);

    // Initialize Controller 
    for (int i = 0; i < number_of_joints_; ++i) {
      std::string jointname = seed_driver.name[i];

      // Create joint state interface
      hardware_interface::JointStateHandle jointStateHandle(jointname, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create position joint interface
      hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
      position_joint_interface_.registerHandle(jointPositionHandle);
      joint_limits_interface::JointLimits limits;
      joint_limits_interface::SoftJointLimits soft_limits;
      joint_limits_interface::getJointLimits(jointname, root_nh, limits);
			//soft_limits
//      joint_limits_interface::PositionJointSoftLimitsHandle positionJointSoftLimitsHandle(jointPositionHandle,limits,soft_limits);
//      position_joint_soft_limits_interface_.registerHandle(positionJointSoftLimitsHandle);
      //saturation limits
      joint_limits_interface::PositionJointSaturationHandle positionJointSaturationHandle(jointPositionHandle,limits);
      position_joint_sat_interface_.registerHandle(positionJointSaturationHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    // soft_limits
//    registerInterface(&position_joint_soft_limits_interface_);
    // saturation limits
    registerInterface(&position_joint_sat_interface_);
  }

  SeedRobotHW::~SeedRobotHW()
  {
    if(seed_->is_open_){
      seed_->closeCom();
      seed_->closePort();
    }
  }

  void SeedRobotHW::initSeedDriver(ros::NodeHandle &robot_hw_nh)
  {
    seed_driver.is_connected = true;
    std::string port_name("/dev/ttyACM0");

    if (robot_hw_nh.hasParam("port_name")) {
      robot_hw_nh.getParam("port_name", port_name);
    }
    if (robot_hw_nh.hasParam("controller_rate")) {
      double rate;
      robot_hw_nh.getParam("controller_rate", rate);
      CONTROL_PERIOD_US_ = (1000*1000)/rate;
    } else {
      CONTROL_PERIOD_US_ = 50*1000; // 50ms
    }

    robot_hw_nh.getParam("/hardware_interface/joint_settings/name",seed_driver.name);
    robot_hw_nh.getParam("/hardware_interface/joint_settings/id",seed_driver.id);
    robot_hw_nh.getParam("/hardware_interface/joint_settings/actuator",seed_driver.type);
    number_of_joints_ = seed_driver.name.size();

    pulse_ratio_["BA"] = 3200.0 / (3.141592 * 2.0);  //[rad] -> [pulse]
    pulse_ratio_["HA"] = (3200.0 * 100.0) / (3.141592 * 2.0); //[rad] -> [pulse]
    pulse_ratio_["BA42"] = 3200.0 / (0.002); //[m] -> [pulse]
    pulse_ratio_["BA25"] = 3200.0 / (0.001); //[m] -> [pulse]

    seed_driver.pulse_ratio.resize(number_of_joints_);
    for(int i = 0; i < number_of_joints_ ; ++i){
      seed_driver.pulse_ratio[i] = pulse_ratio_[seed_driver.type[i]];
    }

    seed_->openPort(port_name,115200);
    if(seed_->is_open_){
      seed_->openCom();

      seed_->getConnectedId();

      std::cout << "Connected SEED Drivers :";
      for(int i=0; i < seed_->connected_id_.size()-1 ; ++i){
        std::cout << "ID " << (int)seed_->connected_id_[i] << ", ";
      }
      std::cout << "ID "  << (int)seed_->connected_id_.back();
      std::cout << std::endl;
      
      for(int i=0; i < number_of_joints_ ; ++i){
        auto scan = std::find(seed_->connected_id_.begin(), seed_->connected_id_.end(),seed_driver.id[i]); 
        if(scan == seed_->connected_id_.end()){
          ROS_ERROR("ID %d is not connected, please check.", seed_driver.id[i]);
          seed_driver.is_connected = false;
        }
      }
            
      for (int i = 0; i < number_of_joints_; i++) seed_->onServo(seed_driver.id[i],1);

      bool calibration;
      robot_hw_nh.getParam("calibration",calibration);
      if(calibration == true){
        for (int i = 0; i < number_of_joints_; i++) seed_->runScript(seed_driver.id[i],1);
        seed_->waitForScriptEnd(number_of_joints_);
        std::cout << "\033[32m Calibration has been completed! \033[m" << std::endl;
      }

    }
    else{
      ROS_ERROR("CAN MS is not connected, please check.");
    }
  }

  void SeedRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
  }

  void SeedRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
		// soft_limits
//    position_joint_soft_limits_interface_.enforceLimits(period);
    // saturation limits
    position_joint_sat_interface_.enforceLimits(period);

    std::array<int,3> motor_state;



    if(seed_->is_open_){
      mutex_.lock();    
      for (int i = 0; i < number_of_joints_; i++) {
        //actuation
        if(joint_position_command_[i] != joint_position_[i])seed_->actuateContinuousAbsolutePosition(seed_driver.id[i],
          (int)(period.toSec() * 1000),static_cast<int>(joint_position_command_[i]*seed_driver.pulse_ratio[i]));

        //get position
        motor_state = seed_->getPosition(seed_driver.id[i]);
        if(motor_state[0]) joint_position_[i] = motor_state[2]/seed_driver.pulse_ratio[i];
      }
      mutex_.unlock();
    }
  }

  

}
