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

#include <sensor_msgs/image_encodings.h>

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
/**
   * @brief Constructor
   * @details Initialises and starts the camera node.
   *
   * @param broker Broker used for creating the module.
   * @param module_name The name of the module.
   */
CameraNode::CameraNode(boost::shared_ptr<AL::ALBroker> broker,
                       const std::string& module_name) :
  AL::ALModule(broker, module_name),
  module_name_(module_name),
  camera_proxy_(new AL::ALVideoDeviceProxy(broker)) {
  Init();
}
/**
  * @brief Destructor
  * @details Stops the module thread and releases memory.
  */
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
  delete camera_proxy_;
}

/**
 * @brief Initialised the module
 * @details The function initialises all components of the modue.
 *          It initialises ROS and advertises the supported services and
 *          topics. Allocates memory for the current images and eventually
 *          starts the spinning thread.
 */
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
  get_color_table_server_ = nh_->advertiseService(
                              "get_color_table",
                              &CameraNode::get_color_table,
                              this);
  // TODO(svepe): Add a service to control the rest of the camera params

  // Create both cameras
  top_cam_ = new Camera(*nh_, AL::kTopCamera, "top");
  bot_cam_ = new Camera(*nh_, AL::kBottomCamera, "bottom");

  // Set the default active camera to be the top one
  active_cam_ = top_cam_;
  active_resolution_ = AL::kVGA;
  active_color_space_ = AL::kYUV422ColorSpace;
  active_fps_ = 15;
  active_rate_ = new ros::Rate(active_fps_);

  // Subscribe to the active Nao camera
  module_name_ = camera_proxy_->subscribeCamera(
                   module_name_,
                   active_cam_->id(), active_resolution_,
                   active_color_space_, active_fps_);
  // Update cached information
  Update();

  // Read the color table
  LoadColorTable();

  // Spawn the thread for the node
  module_thread_ = new boost::thread(boost::bind(&CameraNode::Spin, this));
  is_module_closing_ = false;
}
/**
   * @brief Loads the color table from a file.
   * @details Loads the color table from a file into the table array.
   *          The location of the file is stored in table_file_name_.
   *          Currently it is set to /home/nao/config/camera/table.c64
   */
void CameraNode::LoadColorTable() {
  std::ifstream table_file;
  table_file.open(table_file_name_, std::ios::binary);

  if (!table_file.is_open()) {
    ROS_ERROR("Unable to open color table file.");
    return;
  }

  PixelClass* table_ptr = reinterpret_cast<PixelClass*>(table_);
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
/**
   * @brief Worker function of the module
   * @details The function reads images from the camera, applies segmentation
   *          and then publishes them over ROS. It runs on a separate thread.
   */
void CameraNode::Spin() {
  AL::ALImage* alimage;
  while (!is_module_closing_) {
    // Read the camera frame
    alimage = (AL::ALImage*)(camera_proxy_->getImageLocal(module_name_));
    // Copy the image into the pre-allocated message
    image_.header.stamp = ros::Time::now();
    memcpy(image_.data.data(), alimage->getData(), image_.data.size());

    // Update the timestamp of the camera info
    active_cam_info_.header.stamp = image_.header.stamp;

    // Publish the image with cam_info
    image_pub_.publish(image_, active_cam_info_);

    // Segment the image and publish it
    if (segmented_image_pub_.getNumSubscribers() > 0) {
      SegmentImage(image_, segmented_image_);
      segmented_image_.header.stamp = image_.header.stamp;
      segmented_image_pub_.publish(segmented_image_, active_cam_info_);
    }

    // Publish segmented RGB image if necessary
    if (segmented_rgb_image_pub_.getNumSubscribers() > 0) {
      if (segmented_image_pub_.getNumSubscribers() == 0) {
        SegmentImage(image_, segmented_image_);
      }
      ColorSegmentedImage(segmented_image_, segmented_rgb_image_);
      segmented_rgb_image_.header.stamp = image_.header.stamp;
      segmented_rgb_image_pub_.publish(segmented_rgb_image_);
    }

    // Release the ALImage
    camera_proxy_->releaseImage(module_name_);

    // Spin ROS
    ros::spinOnce();
    active_rate_->sleep();
  }

  camera_proxy_->unsubscribe(module_name_);
}
/**
   * @brief Segments the image using the color look up table
   * @details The 3 color channels of a pixel are used as indecies to the color
   *          lookup table in order to classify them.
   * @param raw A YUV442 encoded input image.
   * @param seg The segmented image in MONO8 encoding. The values are determined
   *            by the PixelClass enumeration.
   */
void CameraNode::SegmentImage(const sensor_msgs::Image& raw,
                              sensor_msgs::Image& seg) {
  for (size_t i = 0, j = 0; i < seg.data.size(); i += 2, j += 4) {
    // First pixel of the YUY'V pair
    // Divde by 4, i.e. shift right by two since the size
    // of each dimension of our lookup color table is 64.
    uint8_t y = raw.data[j + 0] >> 2;
    uint8_t u = raw.data[j + 1] >> 2;
    uint8_t v = raw.data[j + 3] >> 2;
    seg.data[i] = table_[y][u][v];
    // Second pixel of the YUY'V pair
    y = raw.data[j + 2] >> 2;
    seg.data[i + 1] = table_[y][u][v];
  }
}
/**
 * @brief Color the segmented image for visualisation only.
 * @details Each pixel is colored according to its class.
 *
 * @param seg The input segmented image. It should be in MONO8 encoding.
 * @param rgb The output color image.
 */
void CameraNode::ColorSegmentedImage(const sensor_msgs::Image& seg,
                                     sensor_msgs::Image& rgb) {
  // Popoulate the new image buffer
  for (size_t i = 0, j = 0; i < seg.data.size(); ++i, j += 3) {
    // Choose color based on the pixel class
    switch (seg.data[i]) {
      case Nothing: {
        // Gray
        rgb.data[j]     = 128;
        rgb.data[j + 1] = 128;
        rgb.data[j + 2] = 128;
        break;
      }
      case Ball: {
        // Orange
        rgb.data[j]     = 255;
        rgb.data[j + 1] = 128;
        rgb.data[j + 2] = 0;
        break;
      }
      case GoalAndLines: {
        // White
        rgb.data[j]     = 255;
        rgb.data[j + 1] = 255;
        rgb.data[j + 2] = 255;
        break;
      }
      case Field: {
        // Field
        rgb.data[j]     = 64;
        rgb.data[j + 1] = 255;
        rgb.data[j + 2] = 64;
        break;
      }
      case TeamRed: {
        // Red
        rgb.data[j]     = 255;
        rgb.data[j + 1] = 0;
        rgb.data[j + 2] = 0;
        break;
      }
      case TeamBlue: {
        // Blue
        rgb.data[j]     = 0;
        rgb.data[j + 1] = 0;
        rgb.data[j + 2] = 255;
        break;
      }
    }
  }
}
/**
 * @brief Updates the manually allocated images and camera_info.
 */
void CameraNode::Update() {
  UpdateImage();
  UpdateCameraInfo();
}
/**
 * @brief Updates the allocated images to reflect the currently active camera settings.
 */
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

  int pixel_size = 3;
  // Set the image encoding
  switch (active_color_space_) {
    case AL::kYUV422ColorSpace:
      image_.encoding = "yuv422";
      // Adjust pixel size
      pixel_size = 2;
      break;
    case AL::kYUVColorSpace:
      image_.encoding = "yuv8";
      break;
    case AL::kRGBColorSpace:
      image_.encoding = "rgb8";
      break;
    case AL::kHSYColorSpace:
      image_.encoding = "hsy8";
      break;
    case AL::kBGRColorSpace:
      image_.encoding = "bgr8";
      break;
  }

  // Allocate memory
  image_.is_bigendian = false;
  image_.step = image_.width * pixel_size;
  image_.data.resize(image_.height * image_.step);

  // Allocate segmented image
  segmented_image_.header.frame_id = image_.header.frame_id;
  segmented_image_.width = image_.width;
  segmented_image_.height = image_.height;
  segmented_image_.encoding = "mono8";
  segmented_image_.is_bigendian = image_.is_bigendian;
  segmented_image_.step = segmented_image_.width;
  segmented_image_.data.resize(segmented_image_.height * segmented_image_.step);

  // Allocate segmented RGB image
  segmented_rgb_image_.header.frame_id = image_.header.frame_id;
  segmented_rgb_image_.width = image_.width;
  segmented_rgb_image_.height = image_.height;
  segmented_rgb_image_.encoding = "rgb8";
  segmented_rgb_image_.is_bigendian = image_.is_bigendian;
  segmented_rgb_image_.step = 3 * segmented_rgb_image_.width;
  segmented_rgb_image_.data.resize(segmented_rgb_image_.height *
                                   segmented_rgb_image_.step);
}
/**
 * @brief Updates the currently active camera_info to reflect the active camera settings.
 */
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
/**
 * @brief Set the currently active camera (top or bottom).
 *
 * @param req Service request.
 * @param res Service response.
 *
 * @return Return true on successful completion.
 */
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
/**
 * @brief Set the currently active resolution.
 *
 * @param req Service request.
 * @param res Service response.
 *
 * @return Return true on successful completion.
 */
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
/**
* @brief Set the currently active frame rate.
*
* @param req Service request.
* @param res Service response.
*
* @return Return true on successful completion.
*/
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

  return true;
}
/**
* @brief Set the color space in which images are captured.
*
* @param req Service request.
* @param res Service response.
*
* @return Return true on successful completion.
*/
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
/**
 * @brief Set the color table to be used.
 * @details The service receives a serialised color table and stores
 *          it to a file. The filename is stored in table_file_name_
 *          and currently is /home/nao/config/camera/table.c64
 *          Once the file is created, the table is loaded from there.
 *
 * @param req Service request.
 * @param res Service response.
 *
 * @return Return true on successful completion.
 */
bool CameraNode::set_color_table(camera::SetColorTable::Request&  req,
                                 camera::SetColorTable::Response& res) {
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
/**
 * @brief Returns the currently used color table.
 * @details The current color table is serialised and sent back.
 *
 * @param req Service request.
 * @param res Service response.
 *
 * @return Return true on successful completion.
 */
bool CameraNode::get_color_table(camera::GetColorTable::Request&  req,
                                 camera::GetColorTable::Response& res) {
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
