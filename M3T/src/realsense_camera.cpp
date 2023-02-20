// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/realsense_camera.h>

namespace m3t {

RealSense &RealSense::GetInstance() {
  static RealSense realsense;
  return realsense;
}

RealSense::~RealSense() {
  if (initial_set_up_) {
    pipe_.stop();
  }
}

void RealSense::UseColorCamera() { use_color_camera_ = true; }

void RealSense::UseDepthCamera() { use_depth_camera_ = true; }

int RealSense::RegisterID() {
  const std::lock_guard<std::mutex> lock{mutex_};
  update_capture_ids_.insert(std::pair<int, bool>{next_id_, true});
  return next_id_++;
}

bool RealSense::UnregisterID(int id) {
  const std::lock_guard<std::mutex> lock{mutex_};
  return update_capture_ids_.erase(id);
}

bool RealSense::SetUp() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!initial_set_up_) {
    // Configure camera
    if (use_color_camera_)
      config_.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_BGR8, 60);
    if (use_depth_camera_)
      config_.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 60);

    // Start camera
    try {
      profile_ = pipe_.start(config_);
    } catch (std::exception &e) {
      std::cerr << e.what() << std::endl;
      return false;
    }

    // Get extrinsics and calculate pose
    if (use_color_camera_ && use_depth_camera_) {
      const rs2_extrinsics extrinsics =
          profile_.get_stream(RS2_STREAM_COLOR)
              .get_extrinsics_to(profile_.get_stream(RS2_STREAM_DEPTH));
      Eigen::Matrix3f rot{extrinsics.rotation};
      Eigen::Vector3f trans{extrinsics.translation};
      color2depth_pose_.setIdentity();
      color2depth_pose_.translate(trans);
      color2depth_pose_.rotate(rot);
      depth2color_pose_ = color2depth_pose_.inverse();
    }

    // Load multiple images to adjust to white balance
    constexpr int kNumberImagesDropped = 10;
    for (int i = 0; i < kNumberImagesDropped; ++i) {
      pipe_.try_wait_for_frames(&frameset_);
    }
    initial_set_up_ = true;
  }
  return true;
}

bool RealSense::UpdateCapture(int id, bool synchronized) {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!initial_set_up_) return false;

  if (update_capture_ids_.at(id)) {
    if (synchronized)
      pipe_.try_wait_for_frames(&frameset_);
    else
      pipe_.poll_for_frames(&frameset_);
    for (auto &[_, v] : update_capture_ids_) v = false;
  }
  update_capture_ids_.at(id) = true;
  return true;
}

bool RealSense::use_color_camera() const { return use_color_camera_; }

bool RealSense::use_depth_camera() const { return use_depth_camera_; }

const rs2::frameset &RealSense::frameset() const { return frameset_; }

const rs2::pipeline_profile &RealSense::profile() const { return profile_; }

const Transform3fA *RealSense::color2depth_pose() const {
  if (initial_set_up_)
    return &color2depth_pose_;
  else
    return nullptr;
}

const Transform3fA *RealSense::depth2color_pose() const {
  if (initial_set_up_)
    return &depth2color_pose_;
  else
    return nullptr;
}

RealSenseColorCamera::RealSenseColorCamera(const std::string &name,
                                           bool use_depth_as_world_frame)
    : ColorCamera{name},
      use_depth_as_world_frame_{use_depth_as_world_frame},
      realsense_{RealSense::GetInstance()} {
  realsense_.UseColorCamera();
  realsense_id_ = realsense_.RegisterID();
}

RealSenseColorCamera::RealSenseColorCamera(
    const std::string &name, const std::filesystem::path &metafile_path)
    : ColorCamera{name, metafile_path}, realsense_{RealSense::GetInstance()} {
  realsense_.UseColorCamera();
  realsense_id_ = realsense_.RegisterID();
}

RealSenseColorCamera::~RealSenseColorCamera() {
  realsense_.UnregisterID(realsense_id_);
}

bool RealSenseColorCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!initial_set_up_ && !realsense_.SetUp()) return false;
  if (use_depth_as_world_frame_)
    set_camera2world_pose(*realsense_.color2depth_pose());
  GetIntrinsics();
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  return UpdateImage(true);
}

void RealSenseColorCamera::set_use_depth_as_world_frame(
    bool use_depth_as_world_frame) {
  use_depth_as_world_frame_ = use_depth_as_world_frame;
  set_up_ = false;
}

bool RealSenseColorCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up real sense color camera " << name_ << " first"
              << std::endl;
    return false;
  }

  // Get frameset and copy data to image
  realsense_.UpdateCapture(realsense_id_, synchronized);
  cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_8UC3,
          (void *)realsense_.frameset().get_color_frame().get_data(),
          cv::Mat::AUTO_STEP}
      .copyTo(image_);

  SaveImageIfDesired();
  return true;
}

bool RealSenseColorCamera::use_depth_as_world_frame() const {
  return use_depth_as_world_frame_;
}

const Transform3fA *RealSenseColorCamera::color2depth_pose() const {
  return realsense_.color2depth_pose();
}

const Transform3fA *RealSenseColorCamera::depth2color_pose() const {
  return realsense_.depth2color_pose();
}

bool RealSenseColorCamera::LoadMetaData() {
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

void RealSenseColorCamera::GetIntrinsics() {
  const rs2_intrinsics intrinsics = realsense_.profile()
                                        .get_stream(RS2_STREAM_COLOR)
                                        .as<rs2::video_stream_profile>()
                                        .get_intrinsics();
  intrinsics_.fu = intrinsics.fx;
  intrinsics_.fv = intrinsics.fy;
  intrinsics_.ppu = intrinsics.ppx;
  intrinsics_.ppv = intrinsics.ppy;
  intrinsics_.width = intrinsics.width;
  intrinsics_.height = intrinsics.height;
}

RealSenseDepthCamera::RealSenseDepthCamera(const std::string &name,
                                           bool use_color_as_world_frame)
    : DepthCamera{name},
      use_color_as_world_frame_{use_color_as_world_frame},
      realsense_{RealSense::GetInstance()} {
  realsense_.UseDepthCamera();
  realsense_id_ = realsense_.RegisterID();
}

RealSenseDepthCamera::RealSenseDepthCamera(
    const std::string &name, const std::filesystem::path &metafile_path)
    : DepthCamera{name, metafile_path}, realsense_{RealSense::GetInstance()} {
  realsense_.UseDepthCamera();
  realsense_id_ = realsense_.RegisterID();
}

RealSenseDepthCamera::~RealSenseDepthCamera() {
  realsense_.UnregisterID(realsense_id_);
}

bool RealSenseDepthCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!initial_set_up_ && !realsense_.SetUp()) return false;
  if (use_color_as_world_frame_)
    set_camera2world_pose(*realsense_.depth2color_pose());
  GetIntrinsics();
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  return UpdateImage(true);
}

void RealSenseDepthCamera::set_use_color_as_world_frame(
    bool use_color_as_world_frame) {
  use_color_as_world_frame_ = use_color_as_world_frame;
  set_up_ = false;
}

bool RealSenseDepthCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up real sense depth camera " << name_ << " first"
              << std::endl;
    return false;
  }

  // Get frameset and copy data to image
  realsense_.UpdateCapture(realsense_id_, synchronized);
  cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_16UC1,
          (void *)realsense_.frameset().get_depth_frame().get_data(),
          cv::Mat::AUTO_STEP}
      .copyTo(image_);

  SaveImageIfDesired();
  return true;
}

bool RealSenseDepthCamera::use_color_as_world_frame() const {
  return use_color_as_world_frame_;
}

const Transform3fA *RealSenseDepthCamera::color2depth_pose() const {
  return realsense_.color2depth_pose();
}

const Transform3fA *RealSenseDepthCamera::depth2color_pose() const {
  return realsense_.depth2color_pose();
}

bool RealSenseDepthCamera::LoadMetaData() {
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

void RealSenseDepthCamera::GetIntrinsics() {
  const rs2_intrinsics intrinsics = realsense_.profile()
                                        .get_stream(RS2_STREAM_DEPTH)
                                        .as<rs2::video_stream_profile>()
                                        .get_intrinsics();
  intrinsics_.fu = intrinsics.fx;
  intrinsics_.fv = intrinsics.fy;
  intrinsics_.ppu = intrinsics.ppx;
  intrinsics_.ppv = intrinsics.ppy;
  intrinsics_.width = intrinsics.width;
  intrinsics_.height = intrinsics.height;
  depth_scale_ = realsense_.profile()
                     .get_device()
                     .first<rs2::depth_sensor>()
                     .get_depth_scale();
}

}  // namespace m3t
