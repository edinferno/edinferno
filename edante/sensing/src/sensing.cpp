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

#include "sonar.h"
#include "touch.h"

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
  ros::init(argc, argv, "sensing");
  ros::NodeHandle nh("sensing");

  setlocale(LC_NUMERIC, "C");

  boost::shared_ptr<AL::ALBroker> SonarBroker = naoqiBroker("Sonar", 54000);
  boost::shared_ptr<Sonar> SonarTest =
    AL::ALModule::createModule<Sonar>(SonarBroker, "Sonar");

  boost::shared_ptr<AL::ALBroker> TouchBroker = naoqiBroker("Touch", 54100);
  boost::shared_ptr<Touch> TouchTest =
    AL::ALModule::createModule<Touch>(TouchBroker, "Touch");

  SonarTest->rosSetup(&nh);
  TouchTest->rosSetup(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    SonarTest->spin();
    ros::spinOnce();
    r.sleep();
  }

}
