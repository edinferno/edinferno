/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-22
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Add file description...
*/

#ifndef SIT_SERVER_H_
#define SIT_SERVER_H_

#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <motion_planning/SitAction.h>
#include "motion/GetPostureFamily.h"
#include "motion/SetPosture.h"

#include "definitions.h"

class SitAction {
 protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning::SitAction> as_;
  std::string action_name_;
  motion_planning::SitFeedback feedback_;
  motion_planning::SitResult result_;

 public:
  SitAction(std::string name);

  ~SitAction(void);

  void executeCB(const motion_planning::SitGoalConstPtr &goal);

 private:
  std_srvs::Empty wake_up_srv_;
  ros::ServiceClient wake_up_client_;
  motion::GetPostureFamily get_posture_family_srv_;
  ros::ServiceClient get_posture_family_client_;
  std_srvs::Empty stopSrv;
  ros::ServiceClient stopMoveClient;
  motion::SetPosture set_posture_srv_;
  ros::ServiceClient set_posture_client_;

};


#endif /* SIT_SERVER_H_ */
