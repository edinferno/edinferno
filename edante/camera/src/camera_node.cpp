/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The node which exposes the Nao cameras to ROS.
*/
#include "camera/camera_node.hpp"

// System
#include <fstream>

// NaoQi
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

// ROS
#include <ros/serialization.h>
#include <sensor_msgs/image_encodings.h>

using std::vector;
using std::string;

using boost::interprocess::open_or_create;
using boost::interprocess::read_write;
using boost::interprocess::mapped_region;
using boost::interprocess::named_mutex;

using ros::serialization::OStream;
using ros::serialization::Serializer;


const char* CameraNode::table_file_name_ = "/home/nao/config/camera/table.c64";

// NaoQi module entry point
extern "C" {
  /**
   * @brief This function is called from NaoQi when the module is created.
   *
   * @param broker The local NaoQi broker
   * @return Returns 0 on successful creation.
   */
  int _createModule(boost::shared_ptr<AL::ALBroker> broker) {
    // Init broker with the main broker instance from the parent executable
    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);
    // Create module instance
    AL::ALModule::createModule<CameraNode>(broker, "CameraNode");
    return 0;
  }
  /**
   * @brief This function is called from NaoQi when the module is to be closed.
   * @details For some reason this function is not called when NaoQi
   *          is interrupted with Ctrl^C.
   * @return Return 0 on successful closing.
   */
  int _closeModule() {
    return 0;
  }
}

CameraNode::CameraNode(boost::shared_ptr<AL::ALBroker> broker,
                       const std::string& module_name) :
  AL::ALModule(broker, module_name),
  module_name_(module_name),
  cameras_proxy_(new AL::ALVideoDeviceProxy(broker)),
  motion_proxy_(new AL::ALMotionProxy(broker)),
  active_shdmem_(open_or_create, "active_camera", read_write),
  grey_shdmem_(open_or_create, "grey_camera", read_write) {
  Init();
}

CameraNode::~CameraNode() {
  // Stop the spinning thread
  is_module_closing_ = true;
  module_thread_->join();

  // Delete allocated memory
  delete active_rate_;
  delete bot_cam_;
  delete top_cam_;
  delete it_;
  delete nh_;
  delete cameras_proxy_;
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

  segmented_image_pub_ = it_->advertiseCamera("segmented_image", 1);
  segmented_rgb_image_pub_ = it_->advertise("segmented_rgb_image", 1);

  // Advertise services
  set_active_camera_server_ = nh_->advertiseService(
                                "set_active_camera",
                                &CameraNode::SetActiveCameraService,
                                this);
  set_resolution_server_ = nh_->advertiseService(
                             "set_resolution",
                             &CameraNode::SetResolutionService,
                             this);
  set_frame_rate_server_ = nh_->advertiseService(
                             "set_frame_rate",
                             &CameraNode::SetFrameRateServce,
                             this);
  set_color_space_server_ = nh_->advertiseService(
                              "set_color_space",
                              &CameraNode::SetColorSpaceServce,
                              this);
  set_color_table_server_ = nh_->advertiseService(
                              "set_color_table",
                              &CameraNode::SetColorTableServce,
                              this);
  get_color_table_server_ = nh_->advertiseService(
                              "get_color_table",
                              &CameraNode::GetColorTableServce,
                              this);
  // TODO(svepe): Add a service to control the rest of the camera params

  // Create both cameras
  top_cam_ = new Camera(*nh_, "top", "CameraTop",
                        AL::kTopCamera,
                        AL::kQVGA,
                        AL::kYUV422ColorSpace);
  bot_cam_ = new Camera(*nh_, "bottom", "CameraBottom",
                        AL::kBottomCamera,
                        AL::kQVGA,
                        AL::kYUV422ColorSpace);

  // Set the default active camera to be the top one
  active_cam_ = top_cam_;
  active_rate_ = new ros::Rate(default_fps_);

  camera_ids_.push_back(top_cam_->id());
  camera_ids_.push_back(bot_cam_->id());
  camera_resolutions_.push_back(top_cam_->resolution());
  camera_resolutions_.push_back(bot_cam_->resolution());
  camera_color_spaces_.push_back(top_cam_->color_space());
  camera_color_spaces_.push_back(bot_cam_->color_space());

  // Subscribe to the active Nao camera
  module_name_ = cameras_proxy_->subscribeCameras(
                   module_name_,
                   camera_ids_,
                   camera_resolutions_,
                   camera_color_spaces_,
                   default_fps_);

  // Read the color table
  LoadColorTable();

  // Allocate shared memory
  active_shdmem_.truncate(kShdMemSize);
  active_shdmem_region_ = new mapped_region(active_shdmem_, read_write);
  active_shdmem_ptr_ = static_cast<uint8_t*>(
                         active_shdmem_region_->get_address());
  named_mutex::remove("active_camera");
  active_shdmem_mtx_ = new named_mutex(open_or_create, "active_camera");

  // Allocate shared memory
  grey_shdmem_.truncate(kShdMemSize);
  grey_shdmem_region_ = new mapped_region(grey_shdmem_, read_write);
  grey_shdmem_ptr_ = static_cast<uint8_t*>(
                       grey_shdmem_region_->get_address());
  named_mutex::remove("grey_camera");
  grey_shdmem_mtx_ = new named_mutex(open_or_create, "grey_camera");

  // Spawn the thread for the node
  module_thread_ = new boost::thread(boost::bind(&CameraNode::Spin, this));
  is_module_closing_ = false;
}
void CameraNode::LoadColorTable() {
  PixelClass* table_ptr = reinterpret_cast<PixelClass*>(table_);

  std::ifstream table_file;
  table_file.open(table_file_name_, std::ios::binary);

  if (!table_file.is_open()) {
    ROS_WARN("Unable to open color table file. Using empty color table.");
    for (size_t i = 0; i < kTableLen; ++i) { table_ptr[i] = Nothing; }
    return;
  }

  size_t table_pos = 0;
  do {
    // Read the sequence length
    uint32_t len;
    table_file.read(reinterpret_cast<char*>(&len), sizeof(len));

    // Read the class of the sequence
    int current_class;
    table_file.read(reinterpret_cast<char*>(&current_class),
                    sizeof(current_class));

    // The end of the sequence cannot be out of the table.
    if (table_pos + len > kTableLen) {
      ROS_ERROR("Unable to load color table.");
      return;
    }

    // Populate the color table array
    for (size_t i = table_pos; i < table_pos + len; ++i) {
      table_ptr[i] = static_cast<PixelClass>(current_class);
    }

    // Move to the next segment in the color table
    table_pos += len;
  } while (table_pos < kTableLen);
}

void CameraNode::Spin() {

  ros::Time stamp;
  vector<float> transform;
  vector<float> head_angles;
  vector<string> joint_names;
  joint_names.push_back("HeadYaw");
  joint_names.push_back("HeadPitch");

  while (!is_module_closing_) {
    stamp = ros::Time::now();

    //------------------------------ Top Camera --------------------------------
    // Read the frames from both cameras
    AL::ALValue val = cameras_proxy_->getImagesLocal(module_name_);

    // Make a greyscale image from the top camera
    top_cam_->SetGreyscaleImage(reinterpret_cast<const AL::ALImage*>(
                                  static_cast<int>(val[0])),
                                stamp);


    transform = motion_proxy_->getTransform(active_cam_->frame_name(),
                                            2,  // FRAME_ROBOT
                                            true);
    head_angles = motion_proxy_->getAngles(joint_names, true);

    WriteToSharedMemory(grey_shdmem_mtx_,
                        grey_shdmem_ptr_,
                        top_cam_->greyscale_image(),
                        top_cam_->cam_info(),
                        transform,
                        head_angles);

    //----------------------------- Active Camera ------------------------------
    if (image_pub_.getNumSubscribers() > 0) {
      active_cam_->SetImage(reinterpret_cast<const AL::ALImage*>(
                              static_cast<int>(val[active_cam_->id()])),
                            stamp);
      image_pub_.publish(active_cam_->image(), active_cam_->cam_info());
    }

    active_cam_->SetSegmentedImage(reinterpret_cast<const AL::ALImage*>(
                                     static_cast<int>(val[active_cam_->id()])),
                                   table_, stamp);

    WriteToSharedMemory(active_shdmem_mtx_,
                        active_shdmem_ptr_,
                        active_cam_->segmented_image(),
                        active_cam_->cam_info(),
                        transform,
                        head_angles);

    segmented_image_pub_.publish(active_cam_->segmented_image(),
                                 active_cam_->cam_info());

    // Publish segmented RGB image if necessary
    if (segmented_rgb_image_pub_.getNumSubscribers() > 0) {
      segmented_rgb_image_pub_.publish(active_cam_->segmented_rgb_image());
    }

    // Release the ALImage
    cameras_proxy_->releaseImages(module_name_);

    // Spin ROS
    ros::spinOnce();
    active_rate_->sleep();
  }

  cameras_proxy_->unsubscribe(module_name_);
}

void CameraNode::WriteToSharedMemory(boost::interprocess::named_mutex* mtx,
                                     uint8_t* shdmem_ptr,
                                     const sensor_msgs::Image& image,
                                     const sensor_msgs::CameraInfo& cam_info,
                                     const std::vector<float>& transform,
                                     const std::vector<float>& head_angles) {
  // Gain access to the shared memory
  mtx->lock();
  uint8_t* ptr = shdmem_ptr;

  // Write the size in bytes of the cam_info
  uint32_t cam_info_size = ros::serialization::serializationLength(cam_info);
  memcpy(ptr, &cam_info_size, sizeof(cam_info_size));
  ptr += sizeof(cam_info_size);

  // Write the cam_info
  OStream cam_info_stream(ptr, cam_info_size);
  Serializer<sensor_msgs::CameraInfo>::write(cam_info_stream, cam_info);
  ptr += cam_info_size;

  // Write the size in bytes of the image
  uint32_t image_size = ros::serialization::serializationLength(image);
  memcpy(ptr, &image_size, sizeof(image_size));
  ptr += sizeof(image_size);

  // Write the image
  OStream image_stream(ptr, image_size);
  Serializer<sensor_msgs::Image>::write(image_stream, image);
  ptr += image_size;

  // Write the frame transformation
  memcpy(ptr, transform.data(), sizeof(float) * transform.size());
  ptr += sizeof(float) * transform.size();

  // Write the head angles
  memcpy(ptr, head_angles.data(), sizeof(float) * head_angles.size());

  // Release the shared memory
  mtx->unlock();
}

bool CameraNode::SetActiveCameraService(
  camera_msgs::SetActiveCamera::Request& req,
  camera_msgs::SetActiveCamera::Response& res) {
  res.result = true;
  switch (req.active_camera) {
    case 0:
      active_cam_ = top_cam_;
      break;
    case 1:
      active_cam_ = bot_cam_;
      break;
    default:
      res.result = false;
      return true;
  }
  return true;
}

bool CameraNode::SetResolutionService(
  camera_msgs::SetResolution::Request&  req,
  camera_msgs::SetResolution::Response& res) {
  res.result = true;
  switch (req.resolution) {
    case 0:
      active_cam_->resolution(AL::kQQVGA);
      break;
    case 1:
      active_cam_->resolution(AL::kQVGA);
      break;
    case 2:
      active_cam_->resolution(AL::kVGA);
      break;
    case 3:
      active_cam_->resolution(AL::k4VGA);
      break;
    default:
      res.result = false;
      return true;
  }
  camera_resolutions_[active_cam_->id()] = active_cam_->resolution();
  cameras_proxy_->setResolutions(module_name_, camera_resolutions_);

  return true;
}

bool CameraNode::SetFrameRateServce(
  camera_msgs::SetFrameRate::Request&  req,
  camera_msgs::SetFrameRate::Response& res) {
  if (req.frame_rate < 1 || req.frame_rate > 30) {
    res.result = false;
    return true;
  }

  res.result = cameras_proxy_->setFrameRate(module_name_, req.frame_rate);

  if (res.result) {
    delete active_rate_;
    active_rate_ = new ros::Rate(req.frame_rate);
  }
  return true;
}

bool CameraNode::SetColorSpaceServce(
  camera_msgs::SetColorSpace::Request&   req,
  camera_msgs::SetColorSpace::Response& res) {
  res.result = true;
  switch (req.color_space) {
    case 0:
      active_cam_->color_space(AL::kYUV422ColorSpace);
      break;
    case 1:
      active_cam_->color_space(AL::kYUVColorSpace);
      break;
    case 2:
      active_cam_->color_space(AL::kRGBColorSpace);
      break;
    case 3:
      active_cam_->color_space(AL::kHSYColorSpace);
      break;
    case 4:
      active_cam_->color_space(AL::kBGRColorSpace);
      break;
    default:
      res.result = false;
      return true;
  }
  camera_color_spaces_[active_cam_->id()] = active_cam_->color_space();
  cameras_proxy_->setColorSpaces(module_name_, camera_color_spaces_);

  return true;
}

bool CameraNode::SetColorTableServce(
  camera_msgs::SetColorTable::Request&   req,
  camera_msgs::SetColorTable::Response& res) {
  std::ofstream table_file;
  table_file.open(table_file_name_, std::ios::binary);

  if (!table_file.is_open()) {
    res.result = false;
    return true;
  }

  int len = sizeof(uint32_t) * req.table.size();
  table_file.write(reinterpret_cast<char*>(req.table.data()), len);
  table_file.close();

  LoadColorTable();

  res.result = true;
  return true;
}

bool CameraNode::GetColorTableServce(
  camera_msgs::GetColorTable::Request&   req,
  camera_msgs::GetColorTable::Response& res) {
  PixelClass* table_ptr = reinterpret_cast<PixelClass*>(table_);
  for (size_t i = 0; i < kTableLen; ++i) {
    PixelClass c = table_ptr[i];
    size_t len = 1;
    while (table_ptr[i + len] == c) {
      ++len;
      if (i + len == kTableLen) break;
    }
    i += len - 1;
    res.table.push_back(len);
    res.table.push_back(c);
  }
  return true;
}
