#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "seed_robot_hardware.h"

using namespace seed_robot_hardware;

#define MAIN_THREAD_PERIOD_MS    50000 //50ms (20Hz)
#define NSEC_PER_SEC    1000000000L

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seed_ros_controller");

  ros::NodeHandle nh;
  ros::NodeHandle robot_nh("~");

  SeedRobotHW hw(nh, robot_nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  double period = hw.getPeriod();
  controller_manager::ControllerManager cm(&hw, nh);
  ROS_INFO("ControllerManager start with %f Hz", 1.0/period);

  int cntr = 0;
  long main_thread_period_ns = period*1000*1000*1000;
  double max_interval = 0.0;
  double ave_interval = 0.0;
  timespec m_t;
  clock_gettime( CLOCK_MONOTONIC, &m_t );

  ros::Rate r(1/period);
  ros::Time tm = ros::Time::now();


  while (ros::ok()){
    {
      struct timespec tm;
      tm.tv_nsec = m_t.tv_nsec;
      tm.tv_sec  = m_t.tv_sec;
      tm.tv_nsec += main_thread_period_ns;
      while( tm.tv_nsec >= NSEC_PER_SEC ){
        tm.tv_nsec -= NSEC_PER_SEC;
        tm.tv_sec++;
      }
      clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &tm, NULL );

      if(cntr > 100) {
        ROS_INFO("max: %f [ms], ave: %f [ms]", max_interval/1000, ave_interval/1000);
        cntr = 0;
        max_interval = 0.0;
      }
      static timespec n_t;
      clock_gettime( CLOCK_MONOTONIC, &n_t );
      const double measured_interval = ((n_t.tv_sec - m_t.tv_sec)*NSEC_PER_SEC + (n_t.tv_nsec - m_t.tv_nsec))/1000.0; // usec
      if (measured_interval > max_interval) max_interval = measured_interval;
      if(ave_interval == 0.0) {
        ave_interval = measured_interval;
      } else {
        ave_interval = (measured_interval + (100 - 1)*ave_interval)/100.0;
      }
      m_t.tv_sec  = n_t.tv_sec;
      m_t.tv_nsec = n_t.tv_nsec;
      cntr++;
    }
    ros::Time now = ros::Time::now();
    ros::Duration period = now - tm;

    hw.read(now,period);
    cm.update(now, period);
    hw.write(now,period);
    tm = now;
  }
  spinner.stop();
  
  return 0;
}
