/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Add file description...
*/
#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>

#include "sensing/sonar.hpp"
#include "sensing/touch.hpp"
#include "sensing/power.hpp"
#include "sensing/fsr.hpp"

boost::shared_ptr<AL::ALBroker> naoqiBroker(std::string naoqi_ip,
                                            int naoqi_port,
                                            std::string brokerName, int pt) {
  const std::string parentBrokerIP = naoqi_ip;
  int parentBrokerPort = naoqi_port;
  int brokerPort = pt;
  const std::string brokerIp   = "0.0.0.0";

  boost::shared_ptr<AL::ALBroker> broker;
  try {
    broker = AL::ALBroker::createBroker(brokerName, brokerIp, brokerPort,
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
  ros::init(argc, argv, "sensing");
  ros::NodeHandle nh("sensing");
  setlocale(LC_NUMERIC, "C");

  std::string naoqi_ip_;
  int naoqi_port_;
  ros::param::param<std::string>("/naoqi_ip", naoqi_ip_, "127.0.0.1");
  ros::param::param("/naoqi_port", naoqi_port_, 9559);

  boost::shared_ptr<AL::ALBroker> SonarBroker = naoqiBroker(naoqi_ip_,
                                                            naoqi_port_,
                                                            "Sonar", 54200);
  boost::shared_ptr<Sonar> SonarTest =
    AL::ALModule::createModule<Sonar>(SonarBroker, "Sonar");
  SonarTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> TouchBroker = naoqiBroker(naoqi_ip_,
                                                            naoqi_port_,
                                                            "Touch", 54400);
  boost::shared_ptr<Touch> TouchTest =
    AL::ALModule::createModule<Touch>(TouchBroker, "Touch");
  TouchTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> PowerBroker = naoqiBroker(naoqi_ip_,
                                                            naoqi_port_,
                                                            "Power", 54600);
  boost::shared_ptr<Power> PowerTest =
    AL::ALModule::createModule<Power>(PowerBroker, "Power");
  PowerTest->rosSetup(&nh);

  boost::shared_ptr<AL::ALBroker> FsrBroker = naoqiBroker(naoqi_ip_, naoqi_port_,

                                                          "Fsr", 54800);
  boost::shared_ptr<Fsr> FsrTest =
    AL::ALModule::createModule<Fsr>(FsrBroker, "Fsr");
  FsrTest->rosSetup(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    SonarTest->spin();
    FsrTest->spin();
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
  AL::ALBrokerManager::getInstance()->killAllBroker();
  AL::ALBrokerManager::kill();
}
