
#include "pyicg/dummy_camera.h"

namespace icg {

/**
 * Things to manually set in app code:
 * Before setup time:
 * - intrinsics
 * For new images:
 * - image
 * 
*/

/**
 * DummyColorCamera implementation
*/

DummyColorCamera::DummyColorCamera(const std::string &name,
                                           bool use_depth_as_world_frame)
    : ColorCamera(name),
      use_depth_as_world_frame_(use_depth_as_world_frame) {}

DummyColorCamera::DummyColorCamera(
    const std::string &name, const std::filesystem::path &metafile_path)
    : ColorCamera(name, metafile_path) {}

DummyColorCamera::~DummyColorCamera() {}

bool DummyColorCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!extrinsics_set_)
  {
    std::cerr << "DummyColorCamera::set_image color2depth_pose or depth2color_pose not set" << std::endl;
    return false;
  }
  if (use_depth_as_world_frame_)
    set_camera2world_pose(color2depth_pose_);
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  return true;
}

void DummyColorCamera::set_use_depth_as_world_frame(bool use_depth_as_world_frame) {
  use_depth_as_world_frame_ = use_depth_as_world_frame;
  set_up_ = false;
}

void DummyColorCamera::set_image(const cv::Mat& img){
  if (img.channels() != 3){
    std::cerr << "DummyColorCamera::set_image requires a 3-channel color image, provided: " << img.channels() << std::endl;
  }
  image_ = img;
}

void DummyColorCamera::set_intrinsics(const Intrinsics& _intrinsics)
{
  intrinsics_ = _intrinsics;
}

void DummyColorCamera::set_color2depth_pose(const Transform3fA& color2depth_pose) {
  extrinsics_set_ = true;
  color2depth_pose_ = color2depth_pose;
  depth2color_pose_ = color2depth_pose.inverse();
}

void DummyColorCamera::set_depth2color_pose(const Transform3fA& depth2color_pose) {
  extrinsics_set_ = true;
  depth2color_pose_ = depth2color_pose;
  color2depth_pose_ = depth2color_pose.inverse();
}

bool DummyColorCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up dummy color camera " << name_ << " first"
              << std::endl;
    return false;
  }
  if (image_.empty()) {
    std::cerr << "DummyColorCamera " << name_ << " image was not set" << std::endl;
    return false;
  }

  // do nothing here, the image has to be manually set from the application code

  SaveImageIfDesired();
  return true;
}

bool DummyColorCamera::use_depth_as_world_frame() const {
  return use_depth_as_world_frame_;
}

const Intrinsics& DummyColorCamera::get_intrinsics() const {
  return intrinsics_;
}

const Transform3fA& DummyColorCamera::get_color2depth_pose() const {
  return color2depth_pose_;
}

const Transform3fA& DummyColorCamera::get_depth2color_pose() const {
  return depth2color_pose_;
}

bool DummyColorCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "use_depth_as_world_frame",
                            &use_depth_as_world_frame_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}


/**
 * DummyDepthCamera implementation
*/

DummyDepthCamera::DummyDepthCamera(const std::string &name,
                                   bool use_color_as_world_frame,
                                   float depth_scale)
    : DepthCamera{name},
      use_color_as_world_frame_{use_color_as_world_frame} {
  set_depth_scale(depth_scale);
}

DummyDepthCamera::DummyDepthCamera(
    const std::string &name, const std::filesystem::path &metafile_path)
    : DepthCamera{name, metafile_path} {}

DummyDepthCamera::~DummyDepthCamera() {}

bool DummyDepthCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!extrinsics_set_)
  {
    std::cerr << "DummyDepthCamera::set_image color2depth_pose or depth2color_pose not set" << std::endl;
    return false;
  }
  if (use_color_as_world_frame_)
    set_camera2world_pose(depth2color_pose_);
  set_up_ = true;
  initial_set_up_ = true;
  return true;
}

void DummyDepthCamera::set_use_color_as_world_frame(
    bool use_color_as_world_frame) {
  use_color_as_world_frame_ = use_color_as_world_frame;
  set_up_ = false;
}

void DummyDepthCamera::set_image(const cv::Mat& img){
  if (img.channels() != 1){
    std::cerr << "DummyDepthCamera::set_image requires a 1-channel depth image, provided: " << img.channels() << std::endl;
  }
  image_ = img;
}

void DummyDepthCamera::set_intrinsics(const Intrinsics& _intrinsics)
{
  intrinsics_ = _intrinsics;
}

void DummyDepthCamera::set_color2depth_pose(const Transform3fA& color2depth_pose) {
  extrinsics_set_ = true;
  color2depth_pose_ = color2depth_pose;
  depth2color_pose_ = color2depth_pose.inverse();
}

void DummyDepthCamera::set_depth2color_pose(const Transform3fA& depth2color_pose) {
  extrinsics_set_ = true;
  depth2color_pose_ = depth2color_pose;
  color2depth_pose_ = depth2color_pose.inverse();
}

void DummyDepthCamera::set_depth_scale(float depth_scale) {
  depth_scale_ = depth_scale;
}

bool DummyDepthCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up dummy depth camera " << name_ << " first"
              << std::endl;
    return false;
  }
  if (image_.empty()) {
    std::cerr << "DummyDepthCamera " << name_ << " image was not set" << std::endl;
    return false;
  }

  // do nothing here, the image has to be manually set from the application code

  SaveImageIfDesired();
  return true;
}

bool DummyDepthCamera::use_color_as_world_frame() const {
  return use_color_as_world_frame_;
}

const Intrinsics& DummyDepthCamera::get_intrinsics() const {
  return intrinsics_;
}

const Transform3fA& DummyDepthCamera::get_color2depth_pose() const {
  return color2depth_pose_;
}

const Transform3fA& DummyDepthCamera::get_depth2color_pose() const {
  return depth2color_pose_;
}

bool DummyDepthCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "use_color_as_world_frame",
                            &use_color_as_world_frame_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

}  // namespace icg
