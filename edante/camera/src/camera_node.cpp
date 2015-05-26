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

using std::auto_ptr;

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
                       const std::string& module_name) :
  AL::ALModule(broker, module_name),
  module_name_(module_name),
  camera_proxy_(new AL::ALVideoDeviceProxy(broker)) {
  Init();
}

CameraNode::~CameraNode() {
  // TODO: Figure out when NaoQi calls _closeModule
  // delete active_rate_;
  // delete bot_cam_;
  // delete top_cam_;
  // delete it_;
  // delete nh_;
  // delete camera_proxy_;
}

void CameraNode::Init() {
  // Initialise ROS
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "camera_node");

  // Create NodeHandle
  nh_ = new ros::NodeHandle("camera");

  // Create ImageTransport
  it_ = new image_transport::ImageTransport(*nh_);

  // Advertise the camera
  image_pub_ = it_->advertiseCamera("image", 1);

  // Create both cameras
  top_cam_ = new Camera(*nh_, AL::kTopCamera, "top");
  bot_cam_ = new Camera(*nh_, AL::kBottomCamera, "bottom");

  // Set the default active camera to be the top one
  active_cam_ = top_cam_;
  active_resolution_ = AL::k4VGA;
  active_color_space_ = AL::kYUV422ColorSpace;
  active_fps_ = 30;
  active_rate_ = new ros::Rate(active_fps_);

  // Subscribe to the active Nao camera
  module_name_ = camera_proxy_->subscribeCamera(module_name_,
                 active_cam_->id(), active_resolution_,
                 active_color_space_, active_fps_);

  // Allocate the required amount of memory
  UpdateSensorMsgImage();

  // Spawn the thread for the node
  module_thread = new boost::thread(boost::bind(&CameraNode::Spin, this));
}

void CameraNode::Spin() {
  AL::ALImage* alimage;
  while (!is_closing) {
    // Read the camera frame
    alimage = (AL::ALImage*)(camera_proxy_->getImageLocal(module_name_));
    // Copy the image into the pre-allocated message
    image_.header.stamp = ros::Time::now();
    memcpy(image_.data.data(), alimage->getData(), image_.data.size());

    // Create valid camera info
    sensor_msgs::CameraInfo cam_info = active_cam_->cam_info();
    // TODO: Check if camera info matches the image size
    cam_info.header.frame_id = image_.header.frame_id;
    cam_info.header.stamp = image_.header.stamp;

    // Publish the image with cam_info
    image_pub_.publish(image_, cam_info);

    camera_proxy_->releaseImage(module_name_);
    //   camera_proxy_->setActiveCamera(name_, (++camera_idx) % 2);
    //   camera_proxy_->setResolution(name_, AL::kVGA);
    ros::spinOnce();
    active_rate_->sleep();
  }


}

void CameraNode::UpdateSensorMsgImage() {
  // Set the image frame_id
  image_.header.frame_id = active_cam_->frame_id();

  // Set the image size
  switch (active_resolution_) {
    case AL::kQQVGA:
      image_.width = 160;
      image_.height = 120;
      break;
    case AL::kQVGA:
      image_.width = 320;
      image_.height = 240;
      break;
    case AL::kVGA:
      image_.width = 640;
      image_.height = 480;
      break;
    case AL::k4VGA:
      image_.width = 1280;
      image_.height = 960;
      break;
  }

  int pixel_size;
  //Set the image encoding
  switch (active_color_space_) {
    case AL::kYUV422ColorSpace:
      image_.encoding = "yuv422";
      pixel_size = 2;
      break;
    case AL::kYUVColorSpace:
      image_.encoding = "yuv8";
      pixel_size = 3;
      break;
    case AL::kRGBColorSpace:
      image_.encoding = "rgb8";
      pixel_size = 3;
      break;
    case AL::kHSYColorSpace:
      image_.encoding = "hsy8";
      pixel_size = 3;
      break;
    case AL::kBGRColorSpace:
      image_.encoding = "bgr8";
      pixel_size = 3;
      break;
  }

  // Allocate memory
  image_.is_bigendian = false;
  image_.step = image_.width * pixel_size;
  image_.data.resize(image_.height * image_.step);
}
