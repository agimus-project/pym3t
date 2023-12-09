
#ifndef M3t_INCLUDE_M3t_dummy_camera_H_
#define M3t_INCLUDE_M3t_dummy_camera_H_

#include <filesystem>
#include <m3t/camera.h>
#include <m3t/common.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace m3t {


/**
 * \brief \ref Camera that implements a trivial ColorCamera (simply stores a color image, intrinsics and extrinsics)
 *
 */
class DummyColorCamera : public ColorCamera {
 public:
  // Constructors, destructor, and setup method
  DummyColorCamera(const std::string &name);
  DummyColorCamera(const std::string &name,
                   const std::filesystem::path &metafile_path);
  ~DummyColorCamera();
  bool SetUp() override;

  // Setters
  void set_image(const cv::Mat& img);
  void set_intrinsics(const Intrinsics& intr);

  // Main method -> does nothing in this implementation
  bool UpdateImage(bool synchronized) override;

  // Getters
  const Intrinsics& get_intrinsics() const;

 private:
  // Helper methods
  bool LoadMetaData();

  // Data
  bool initial_set_up_ = false;
};

/**
 * \brief \ref Camera that implements a trivial DepthCamera (simply stores a depth image, intrinsics, and extrinsics)
 *
 */
class DummyDepthCamera : public DepthCamera {
 public:
  // Constructors, destructor, and setup method
  DummyDepthCamera(const std::string &name,
                   float depth_scale);
  DummyDepthCamera(const std::string &name,
                   const std::filesystem::path &metafile_path);
  ~DummyDepthCamera();
  bool SetUp() override;

  // Setters
  void set_image(const cv::Mat& img);
  void set_intrinsics(const Intrinsics& intr);
  void set_depth_scale(float depth_scale);

  // Main method -> does nothing in this implementation
  bool UpdateImage(bool synchronized) override;

  // Getters
  const Intrinsics& get_intrinsics() const;

 private:
  // Helper methods
  bool LoadMetaData();

  bool initial_set_up_ = false;

};

}  // namespace m3t

#endif  // M3t_INCLUDE_M3t_dummy_camera_H_
