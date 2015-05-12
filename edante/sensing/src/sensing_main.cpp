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
#include <alcommon/altoolsmain.h>

#include "sensing_test.h"

extern "C"
{
  int _createModule(boost::shared_ptr<AL::ALBroker> pBroker) {
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);
    AL::ALModule::createModule<Sensing_Test>( pBroker, "Sensing_Test");
    return 0;
  }
  int _closeModule() {
    return 0;
  }
}

int main(int argc, char *argv[]) {
  // pointer to createModule
  TMainType sig;
  sig = &_createModule;
  // call main
  ALTools::mainFunction("Sensing_Test", argc, argv, sig);
}
