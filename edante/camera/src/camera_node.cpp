/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The node which exposes the Nao cameras to ROS.
*/
#include "camera/camera_node.hpp"

#include <fstream>

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

#include <boost/thread.hpp>

#include <sensor_msgs/image_encodings.h>

using std::auto_ptr;

boost::thread* module_thread;
bool is_closing;

const char* CameraNode::table_file_name_ = "/home/nao/config/camera/table.c64";

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
  delete active_rate_;
  delete bot_cam_;
  delete top_cam_;
  delete it_;
  delete nh_;
  delete camera_proxy_;
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

  // Advertise services
  set_active_camera_server_ = nh_->advertiseService(
                                "set_active_camera",
                                &CameraNode::set_active_camera,
                                this);
  set_resolution_server_ = nh_->advertiseService(
                             "set_resolution",
                             &CameraNode::set_resolution,
                             this);
  set_frame_rate_server_ = nh_->advertiseService(
                             "set_frame_rate",
                             &CameraNode::set_frame_rate,
                             this);
  set_color_space_server_ = nh_->advertiseService(
                              "set_color_space",
                              &CameraNode::set_color_space,
                              this);
  set_color_table_server_ = nh_->advertiseService(
                              "set_color_table",
                              &CameraNode::set_color_table,
                              this);
  // TODO(svepe): Add a service to control the rest of the camera params

  // Create both cameras
  top_cam_ = new Camera(*nh_, AL::kTopCamera, "top");
  bot_cam_ = new Camera(*nh_, AL::kBottomCamera, "bottom");

  // Set the default active camera to be the top one
  active_cam_ = top_cam_;
  active_resolution_ = AL::kVGA;
  active_color_space_ = AL::kYUV422ColorSpace;
  active_fps_ = 30;
  active_rate_ = new ros::Rate(active_fps_);

  // Subscribe to the active Nao camera
  module_name_ = camera_proxy_->subscribeCamera(
                   module_name_,
                   active_cam_->id(), active_resolution_,
                   active_color_space_, active_fps_);

  // Update cached information
  Update();

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

    // Update the timestamp of the camera info
    active_cam_info_.header.stamp = image_.header.stamp;

    // Publish the image with cam_info
    image_pub_.publish(image_, active_cam_info_);

    // Release the ALImage
    camera_proxy_->releaseImage(module_name_);

    // Spin ROS
    ros::spinOnce();
    active_rate_->sleep();
  }
}

void CameraNode::Update() {
  UpdateImage();
  UpdateCameraInfo();
}

void CameraNode::UpdateImage() {
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
  // Set the image encoding
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

void CameraNode::UpdateCameraInfo() {
  active_cam_info_ = active_cam_->cam_info();
  active_cam_info_.header.frame_id = image_.header.frame_id;

  // Check if the stored camera info matches the current settings
  if (active_cam_info_.width != image_.width ||
      active_cam_info_.height != image_.height) {
    // The sizes do not match so generate and uncalibrated camera settings
    active_cam_info_ = sensor_msgs::CameraInfo();
    active_cam_info_.width = image_.width;
    active_cam_info_.height = image_.height;
  }
}

bool CameraNode::set_active_camera(camera::SetActiveCamera::Request&  req,
                                   camera::SetActiveCamera::Response& res) {
  int new_active_cam_id;
  Camera* new_active_cam;
  switch (req.active_camera) {
    case 0:
      new_active_cam_id = AL::kTopCamera;
      new_active_cam = top_cam_;
      break;
    case 1:
      new_active_cam_id = AL::kBottomCamera;
      new_active_cam = bot_cam_;
      break;
    default:
      res.result = false;
      return true;
  }

  res.result = camera_proxy_->setActiveCamera(module_name_, new_active_cam_id);

  if (res.result) {
    active_cam_ = new_active_cam;
    camera_proxy_->setColorSpace(module_name_, active_color_space_);
    camera_proxy_->setResolution(module_name_, active_resolution_);
    camera_proxy_->setFrameRate(module_name_, active_fps_);

    Update();
  }

  return true;
}

bool CameraNode::set_resolution(camera::SetResolution::Request&  req,
                                camera::SetResolution::Response& res) {
  int new_resolution;
  switch (req.resolution) {
    case 0:
      new_resolution = AL::kQQVGA;
      break;
    case 1:
      new_resolution = AL::kQVGA;
      break;
    case 2:
      new_resolution = AL::kVGA;
      break;
    case 3:
      new_resolution = AL::k4VGA;
      break;
    default:
      res.result = false;
      return true;
  }

  res.result = camera_proxy_->setResolution(module_name_, new_resolution);

  if (res.result) {
    active_resolution_ = new_resolution;
    Update();
  }

  return true;
}

bool CameraNode::set_frame_rate(camera::SetFrameRate::Request&  req,
                                camera::SetFrameRate::Response& res) {
  if (req.frame_rate < 1 || req.frame_rate > 30) {
    res.result = false;
    return true;
  }

  res.result = camera_proxy_->setFrameRate(module_name_, req.frame_rate);

  if (res.result) {
    active_fps_ = req.frame_rate;
    delete active_rate_;
    active_rate_ = new ros::Rate(active_fps_);
  }
}

bool CameraNode::set_color_space(camera::SetColorSpace::Request&  req,
                                 camera::SetColorSpace::Response& res) {
  int new_color_space;
  switch (req.color_space) {
    case 0:
      new_color_space = AL::kYUV422ColorSpace;
      break;
    case 1:
      new_color_space = AL::kYUVColorSpace;
      break;
    case 2:
      new_color_space = AL::kRGBColorSpace;
      break;
    case 3:
      new_color_space = AL::kHSYColorSpace;
      break;
    case 4:
      new_color_space = AL::kBGRColorSpace;
      break;
    default:
      res.result = false;
      return true;
  }

  res.result = camera_proxy_->setColorSpace(module_name_, new_color_space);

  if (res.result) {
    active_color_space_ = new_color_space;
    Update();
  }

  return true;
}

bool CameraNode::set_color_table(camera::SetColorTable::Request&  req,
                                 camera::SetColorTable::Response& res) {
  std::ofstream table_file;
  table_file.open(table_file_name_, std::ios::binary);

  if (!table_file.is_open()) {
    res.result = false;
    return true;
  }

  table_file.write(reinterpret_cast<char*>(req.table.data()), req.table.size());
  table_file.close();
}
