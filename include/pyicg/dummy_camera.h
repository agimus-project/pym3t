
#ifndef ICG_INCLUDE_ICG_dummy_camera_H_
#define ICG_INCLUDE_ICG_dummy_camera_H_

#include <filesystem/filesystem.h>
#include <icg/camera.h>
#include <icg/common.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace icg {


/**
 * \brief \ref Camera that implements trivial implementations of ColorCamera
 *
 * @param use_depth_as_world_frame specifies the depth camera frame as world
 * frame and automatically defines `camera2world_pose` as `color2depth_pose`.
 */
class DummyColorCamera : public ColorCamera {
 public:
  // Constructors, destructor, and setup method
  DummyColorCamera(const std::string &name,
                       bool use_depth_as_world_frame = false);
  DummyColorCamera(const std::string &name,
                       const std::filesystem::path &metafile_path);
  ~DummyColorCamera();
  bool SetUp() override;

  // Setters
  void set_use_depth_as_world_frame(bool use_depth_as_world_frame);
  void set_image(const cv::Mat& img);
  void set_intrinsics(const Intrinsics& intr);
  void set_color2depth_pose(const Transform3fA & color2depth_pose);
  void set_depth2color_pose(const Transform3fA & depth2color_pose);

  // Main method -> does nothing in this implementation
  bool UpdateImage(bool synchronized) override;

  // Getters
  bool use_depth_as_world_frame() const;
  const Intrinsics& get_intrinsics() const;
  const Transform3fA& get_color2depth_pose() const;
  const Transform3fA& get_depth2color_pose() const;

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  bool use_depth_as_world_frame_ = false;
  bool initial_set_up_ = false;
  bool extrinsics_set_ = false;

  // extrinsics
  Transform3fA color2depth_pose_{Transform3fA::Identity()};
  Transform3fA depth2color_pose_{Transform3fA::Identity()};
};

/**
 * \brief \ref Camera that allows getting depth images from a \ref Trivial
 * camera.
 *
 * @param use_color_as_world_frame specifies the color camera frame as world
 * frame and automatically defines `camera2world_pose` as `depth2color_pose`.
 */
class DummyDepthCamera : public DepthCamera {
 public:
  // Constructors, destructor, and setup method
  DummyDepthCamera(const std::string &name,
                       bool use_color_as_world_frame = true);
  DummyDepthCamera(const std::string &name,
                       const std::filesystem::path &metafile_path);
  ~DummyDepthCamera();
  bool SetUp() override;

  // Setters
  void set_use_color_as_world_frame(bool use_color_as_world_frame);
  void set_image(const cv::Mat& img);
  void set_intrinsics(const Intrinsics& intr);
  void set_color2depth_pose(const Transform3fA & color2depth_pose);
  void set_depth2color_pose(const Transform3fA & depth2color_pose);

  // Main method -> does nothing in this implementation
  bool UpdateImage(bool synchronized) override;

  // Getters
  bool use_color_as_world_frame() const;
  const Intrinsics& get_intrinsics() const;
  const Transform3fA& get_color2depth_pose() const;
  const Transform3fA& get_depth2color_pose() const;

 private:
  // Helper methods
  bool LoadMetaData();

  bool use_color_as_world_frame_ = true;
  bool initial_set_up_ = false;
  bool extrinsics_set_ = false;

  // extrinsics
  Transform3fA color2depth_pose_{Transform3fA::Identity()};
  Transform3fA depth2color_pose_{Transform3fA::Identity()};
};

}  // namespace icg

#endif  // ICG_INCLUDE_ICG_dummy_camera_H_
