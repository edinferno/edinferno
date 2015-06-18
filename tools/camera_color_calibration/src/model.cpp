/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-29
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: The camera_color_calibration application follows the
*             Model View Controller (MVC) approach and this class
*             represents the Model which implements the program logic.
*/
#include "camera_color_calibration/model.hpp"
#include "camera_color_calibration/controller.hpp"

#include <camera_msgs/SetActiveCamera.h>
#include <camera_msgs/SetColorSpace.h>
#include <camera_msgs/SetColorTable.h>
#include <camera_msgs/GetColorTable.h>

Model::Model(Controller* controller) {
  controller_ = controller;
}

void Model::Build(int argc, char** argv) {
  // Initialise ROS
  ros::init(argc, argv, "camera_color_calibration_node");

  // Create NodeHandle
  nh_ = new ros::NodeHandle("camera");

  // Create ImageTransport
  it_ = new image_transport::ImageTransport(*nh_);

  // Subscribe to camera images
  image_sub_ = it_->subscribe("image", 1, &Model::ImageCallback, this);
  SetupCamera();
  LoadTable();
}

void Model::SetupCamera() {
  // Set top camera as active
  active_camera_ = 1;
  SwitchCamera();

  // Set color space to be YUV422
  ros::ServiceClient client =
    nh_->serviceClient<camera_msgs::SetColorSpace>("set_color_space");
  camera_msgs::SetColorSpace srv;
  srv.request.color_space = 0;
  client.call(srv);
}

void Model::SwitchCamera() {
  ros::ServiceClient client =
    nh_->serviceClient<camera_msgs::SetActiveCamera>("set_active_camera");
  camera_msgs::SetActiveCamera srv;
  srv.request.active_camera = (active_camera_ + 1) % 2;
  if (client.call(srv))
    active_camera_ = (active_camera_ + 1) % 2;
}
void Model::ClearTable() {
  PixelClass* table_ptr = reinterpret_cast<PixelClass*>(table_);
  for (size_t i = 0; i < kTableLen; ++i) {
    table_ptr[i] = Nothing;
  }
}
bool Model::LoadTable() {
  ros::ServiceClient client =
    nh_->serviceClient<camera_msgs::GetColorTable>("get_color_table");
  camera_msgs::GetColorTable srv;
  if (!client.call(srv)) {
    ROS_ERROR("Unable to receive color table.");
    return false;
  }

  PixelClass* table_ptr = reinterpret_cast<PixelClass*>(table_);
  size_t table_pos = 0, msg_pos = 0;
  do {
    // Read the sequence length
    uint32_t len = srv.response.table[msg_pos];
    ++msg_pos;
    // Read the class of the sequence
    int current_class = srv.response.table[msg_pos];
    ++msg_pos;

    if (table_pos + len > kTableLen) {
      ROS_ERROR("Unable to load color table.");
      return false;
    }

    for (size_t i = table_pos; i < table_pos + len; ++i) {
      table_ptr[i] = static_cast<PixelClass>(current_class);
    }

    table_pos += len;
  } while (table_pos < kTableLen);

  return false;
}

bool Model::SendTable() {
  ros::ServiceClient client =
    nh_->serviceClient<camera_msgs::SetColorTable>("set_color_table");
  camera_msgs::SetColorTable srv;

  PixelClass* table_ptr = reinterpret_cast<PixelClass*>(table_);
  for (size_t i = 0; i < kTableLen; ++i) {
    PixelClass c = table_ptr[i];
    size_t len = 1;
    while (table_ptr[i + len] == c) {
      ++len;
      if (i + len == kTableLen) break;
    }
    i += len - 1;
    srv.request.table.push_back(len);
    srv.request.table.push_back(c);
  }

  // Send the serialised color table to the robot
  return  client.call(srv);
}


void Model::AddNewPixelClass(double x, double y, PixelClass pixel_class) {
  int y_px = y * (raw_image_.height - 1);
  int x_px = x * (raw_image_.width - 1);
  int pixel_index = 2 * (y_px * raw_image_.width + x_px);

  uint y_val, u_val, v_val;
  y_val = raw_image_.data[pixel_index] / 4;
  if (pixel_index % 4 == 0) {
    u_val = raw_image_.data[pixel_index + 1] / 4;
    v_val = raw_image_.data[pixel_index + 3] / 4;
  } else if (pixel_index % 4 == 2) {
    u_val = raw_image_.data[pixel_index - 1] / 4;
    v_val = raw_image_.data[pixel_index + 1] / 4;
  }

  const unsigned int expand = 4;
  uint y_min = (y_val - expand < 0) ? 0 : y_val - expand;
  uint y_max = (y_val + expand > kTableSize) ? kTableSize : y_val + expand;
  uint u_min = (u_val - expand < 0) ? 0 : u_val - expand;
  uint u_max = (u_val + expand > kTableSize) ? kTableSize : u_val + expand;
  uint v_min = (v_val - expand < 0) ? 0 : v_val - expand;
  uint v_max = (v_val + expand > kTableSize) ? kTableSize : v_val + expand;

  for (y_val = y_min; y_val < y_max; ++y_val) {
    for (u_val = u_min; u_val < u_max; ++u_val) {
      for (v_val = v_min; v_val < v_max; ++v_val) {
        table_[y_val][u_val][v_val] = pixel_class;
      }
    }
  }
}

void Model::SegmentImage(const sensor_msgs::Image& raw,
                         sensor_msgs::Image& seg,
                         PixelClass table[kTableSize][kTableSize][kTableSize]) {
  if (seg.width != raw.width || seg.height != raw.height) {
    // Create new segmented image
    seg.width = raw.width;
    seg.height = raw.height;
    seg.encoding = "mono8";
    seg.step = seg.width;
    seg.data.clear();
    seg.data.resize(seg.width * seg.height, 0);
  }

  for (size_t i = 0, j = 0; i < seg.data.size(); i += 2, j += 4) {
    // First pixel of the YUY'V pair
    uint8_t y = raw.data[j + 0] / 4;
    uint8_t u = raw.data[j + 1] / 4;
    uint8_t v = raw.data[j + 3] / 4;
    seg.data[i] = table[y][u][v];
    // Second pixel of the YUY'V pair
    y = raw.data[j + 2] / 4;
    seg.data[i + 1] = table[y][u][v];
  }
}

void Model::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (msg->encoding != "yuv422") {
    ROS_WARN_STREAM(std::string("You must use the yuv422 colorspace. Run:\n") +
                    std::string("rosservice call /camera/set_color_space 0."));
    return;
  }
  raw_image_ = *msg;
  controller_->OnNewRawImage(raw_image_);
  SegmentImage(raw_image_, seg_image_, table_);
  controller_->OnNewSegmentedImage(seg_image_);
}

