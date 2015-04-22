#include "motion_task.h"

Motion_Task::Motion_Task()
{
	nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);
	INFO("Setting up Nao motion task services" << std::endl);
	srv_kill_move_ = nh_->advertiseService("motion/kill_move", &Motion_Task::killMove, this);
  srv_kill_all_ = nh_->advertiseService("motion/kill_all", &Motion_Task::killAll, this);
  
}

Motion_Task::~Motion_Task()
{
  ros::shutdown();  
}

bool Motion_Task::killMove(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
{
  mProxy_->killMove();
}

bool Motion_Task::killAll(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
{
  mProxy_->killAll();
}
