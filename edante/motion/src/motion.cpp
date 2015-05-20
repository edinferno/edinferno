/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Main motion node
*/

#include <ros/ros.h>

#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>

#include "stiffness_control.h"
#include "joint_control.h"
#include "locomotion_control.h"
#include "cartesian_control.h"
#include "body_balancer.h"
#include "fall_manager.h"
#include "motion_task.h"
#include "robot_posture.h"
#include "definitions.h"

boost::shared_ptr<AL::ALBroker> naoqiBroker(std::string brokerName, int pt) {
  const std::string parentBrokerIP = "127.0.0.1";
  int parentBrokerPort = 9559;
  int brokerPort = pt;
  const std::string brokerIp   = "0.0.0.0";

  boost::shared_ptr<AL::ALBroker> broker;
  try {
    broker = AL::ALBroker::createBroker( brokerName, brokerIp, brokerPort,
                                         parentBrokerIP, parentBrokerPort, 0);
  } catch (const AL::ALError& /* e */) {
    std::cerr << "Faild to connect broker to: "
              << parentBrokerIP << ":" << parentBrokerPort << std::endl;
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
  }

  AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
  AL::ALBrokerManager::getInstance()->addBroker(broker);
  return broker;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  setlocale(LC_NUMERIC, "C");

  boost::shared_ptr<AL::ALBroker> StiffnessControlBroker =
    naoqiBroker("StiffnessControl", 55000);
  boost::shared_ptr<StiffnessControl> StiffnessControlTest =
    AL::ALModule::createModule<StiffnessControl>(StiffnessControlBroker,
        "StiffnessControl");
  StiffnessControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> JointControlBroker =
    naoqiBroker("JointControl", 55100);
  boost::shared_ptr<JointControl> JointControlTest =
    AL::ALModule::createModule<JointControl>(JointControlBroker,
        "JointControl");
  JointControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> LocomotionBroker =
    naoqiBroker("LocomotionControl", 55200);
  boost::shared_ptr<LocomotionControl> LocomotionControlTest =
    AL::ALModule::createModule<LocomotionControl>(LocomotionBroker,
        "LocomotionControl");
  LocomotionControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> CartesianControlBroker =
    naoqiBroker("CartesianControl", 55300);
  boost::shared_ptr<CartesianControl> CartesianControlTest =
    AL::ALModule::createModule<CartesianControl>(CartesianControlBroker,
        "CartesianControl");
  CartesianControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> BodyBalancerBroker =
    naoqiBroker("BodyBalancer", 55400);
  boost::shared_ptr<BodyBalancer> BodyBalancerTest =
    AL::ALModule::createModule<BodyBalancer>(BodyBalancerBroker,
        "BodyBalancer");
  BodyBalancerTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> FallManagerBroker =
    naoqiBroker("FallManager", 55500);
  boost::shared_ptr<FallManager> FallManagerTest =
    AL::ALModule::createModule<FallManager>(FallManagerBroker,
        "FallManager");
  FallManagerTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> MotionTaskBroker =
    naoqiBroker("MotionTask", 55600);
  boost::shared_ptr<MotionTask> MotionTaskTest =
    AL::ALModule::createModule<MotionTask>(MotionTaskBroker,
        "MotionTask");
  MotionTaskTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> RobotPostureBroker =
    naoqiBroker("RobotPosture", 55700);
  boost::shared_ptr<RobotPosture> RobotPostureTest =
    AL::ALModule::createModule<RobotPosture>(RobotPostureBroker,
        "RobotPosture");
  RobotPostureTest->rosSetup(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  AL::ALBrokerManager::getInstance()->killAllBroker();
  AL::ALBrokerManager::kill();
}
