/**
 * @file      motion.cpp
 * @brief     Main motion node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#include <ros/ros.h>

#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>

#include <motion/stiffness_control.hpp>
#include <motion/joint_control.hpp>
#include <motion/locomotion_control.hpp>
#include <motion/cartesian_control.hpp>
#include <motion/body_balancer.hpp>
#include <motion/fall_manager.hpp>
#include <motion/motion_task.hpp>
#include <motion/robot_posture.hpp>


boost::shared_ptr<AL::ALBroker> naoqiBroker(std::string naoqi_ip,
                                            int naoqi_port,
                                            std::string brokerName, int pt) {
  const std::string parentBrokerIP = naoqi_ip;
  int parentBrokerPort = naoqi_port;
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

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  setlocale(LC_NUMERIC, "C");

  std::string naoqi_ip_;
  int naoqi_port_;
  ros::param::param<std::string>("/naoqi_ip", naoqi_ip_, "127.0.0.1");
  ros::param::param("/naoqi_port", naoqi_port_, 9559);

  boost::shared_ptr<AL::ALBroker> StiffnessControlBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "StiffnessControl", 55000);
  boost::shared_ptr<StiffnessControl> StiffnessControlTest =
    AL::ALModule::createModule<StiffnessControl>(StiffnessControlBroker,
                                                 "StiffnessControl");
  StiffnessControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> JointControlBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "JointControl", 55100);
  boost::shared_ptr<JointControl> JointControlTest =
    AL::ALModule::createModule<JointControl>(JointControlBroker,
                                             "JointControl");
  JointControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> LocomotionBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "LocomotionControl", 55200);
  boost::shared_ptr<LocomotionControl> LocomotionControlTest =
    AL::ALModule::createModule<LocomotionControl>(LocomotionBroker,
                                                  "LocomotionControl");
  LocomotionControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> CartesianControlBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "CartesianControl", 55300);
  boost::shared_ptr<CartesianControl> CartesianControlTest =
    AL::ALModule::createModule<CartesianControl>(CartesianControlBroker,
                                                 "CartesianControl");
  CartesianControlTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> BodyBalancerBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "BodyBalancer", 55400);
  boost::shared_ptr<BodyBalancer> BodyBalancerTest =
    AL::ALModule::createModule<BodyBalancer>(BodyBalancerBroker,
                                             "BodyBalancer");
  BodyBalancerTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> FallManagerBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "FallManager", 55500);
  boost::shared_ptr<FallManager> FallManagerTest =
    AL::ALModule::createModule<FallManager>(FallManagerBroker,
                                            "FallManager");
  FallManagerTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> MotionTaskBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "MotionTask", 55600);
  boost::shared_ptr<MotionTask> MotionTaskTest =
    AL::ALModule::createModule<MotionTask>(MotionTaskBroker,
                                           "MotionTask");
  MotionTaskTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> RobotPostureBroker =
    naoqiBroker(naoqi_ip_, naoqi_port_, "RobotPosture", 55700);
  boost::shared_ptr<RobotPosture> RobotPostureTest =
    AL::ALModule::createModule<RobotPosture>(RobotPostureBroker,
                                             "RobotPosture");
  RobotPostureTest->rosSetup(&nh);

  ros::Rate r(30);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  AL::ALBrokerManager::getInstance()->killAllBroker();
  AL::ALBrokerManager::kill();
}
