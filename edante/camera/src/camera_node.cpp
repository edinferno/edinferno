/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The node which exposes the Nao cameras to ROS.
*/
#include "camera/camera_node.hpp"

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

#include <boost/thread.hpp>

#include <sensor_msgs/image_encodings.h>

boost::thread* module_thread;
bool is_closing;

// NaoQi module entry point
extern "C" {
  int _createModule(boost::shared_ptr<AL::ALBroker> broker) {
    // init broker with the main broker instance
    // from the parent executable
    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);
    // create module instances
    is_closing = false;
    AL::ALModule::createModule<CameraNode>(broker, "CameraNode");
    return 0;
  }
  int _closeModule() {
    is_closing = true;
    module_thread->join();
    return 0;
  }
}

CameraNode::CameraNode(boost::shared_ptr<AL::ALBroker> broker,
                       const std::string& name) :
  AL::ALModule(broker, name),
  name_(name) {

  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "camera_node");
  nh_ = new ros::NodeHandle("camera");
  camera_proxy_ = new AL::ALVideoDeviceProxy(broker);
  name_ = camera_proxy_->subscribeCamera(name, 0, AL::kVGA,
                                         AL::kBGRColorSpace, 30);

  image_pub_ = nh_->advertise<sensor_msgs::Image>("image", 1);

  module_thread = new boost::thread(boost::bind(&CameraNode::Spin, this));
}

CameraNode::~CameraNode() {}

void CameraNode::Spin() {
  ros::Rate rate(5);

  image_.header.frame_id = "camera";
  image_.height = 480;
  image_.width = 640;
  image_.encoding = sensor_msgs::image_encodings::BGR8;
  image_.is_bigendian = false;
  image_.step = image_.width * 3;
  image_.data.resize(image_.height * image_.step);

  while (!is_closing) {
    AL::ALImage* alimage = (AL::ALImage*)camera_proxy_->getImageLocal(name_);
    memcpy(image_.data.data(), alimage->getData(), image_.height * image_.step);
    image_pub_.publish(image_);
    camera_proxy_->releaseImage(name_);
    ros::spinOnce();
    rate.sleep();
  }
}
