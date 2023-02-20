// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/azure_kinect_camera.h>

namespace m3t {

AzureKinect &AzureKinect::GetInstance() {
  static AzureKinect kinect;
  return kinect;
}

AzureKinect::~AzureKinect() {
  if (initial_set_up_) {
    device_.stop_cameras();
    device_.close();
  }
}

void AzureKinect::UseColorCamera() { use_color_camera_ = true; }

void AzureKinect::UseDepthCamera() { use_depth_camera_ = true; }

int AzureKinect::RegisterID() {
  const std::lock_guard<std::mutex> lock{mutex_};
  update_capture_ids_.insert(std::pair<int, bool>{next_id_, true});
  return next_id_++;
}

bool AzureKinect::UnregisterID(int id) {
  const std::lock_guard<std::mutex> lock{mutex_};
  return update_capture_ids_.erase(id);
}

bool AzureKinect::SetUp() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!initial_set_up_) {
    // Configure camera
    config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config_.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config_.synchronized_images_only = true;
    if (use_color_camera_) {
      config_.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
      config_.color_resolution = K4A_COLOR_RESOLUTION_720P;
    }
    if (use_depth_camera_) {
      config_.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    }

    // Check if camera is available
    if (k4a::device::get_installed_count() == 0) return false;

    // Start camera
    device_ = k4a::device::open(K4A_DEVICE_DEFAULT);
    device_.start_cameras(&config_);

    // Get calibration
    calibration_ =
        device_.get_calibration(config_.depth_mode, config_.color_resolution);

    // Get extrinsics and calculate pose
    if (use_color_camera_ && use_depth_camera_) {
      const k4a_calibration_extrinsics_t extrinsics{
          device_.get_calibration(config_.depth_mode, config_.color_resolution)
              .extrinsics[K4A_CALIBRATION_TYPE_COLOR]
                         [K4A_CALIBRATION_TYPE_DEPTH]};
      Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot{extrinsics.rotation};
      Eigen::Vector3f trans{extrinsics.translation};
      color2depth_pose_.setIdentity();
      color2depth_pose_.translate(trans * 0.001f);
      color2depth_pose_.rotate(rot);
      depth2color_pose_ = color2depth_pose_.inverse();
    }

    // Load multiple images to adjust to white balance
    constexpr int kTimeoutInMs = 100;
    constexpr int kNumberImagesDropped = 10;
    for (int i = 0; i < kNumberImagesDropped; ++i) {
      while (!device_.get_capture(&capture_,
                                  std::chrono::milliseconds{kTimeoutInMs}))
        ;
    }
    initial_set_up_ = true;
  }
  return true;
}

bool AzureKinect::UpdateCapture(int id, bool synchronized) {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (!initial_set_up_) return false;

  if (update_capture_ids_.at(id)) {
    if (synchronized)
      device_.get_capture(&capture_, std::chrono::milliseconds{-1});
    else
      device_.get_capture(&capture_, std::chrono::milliseconds{0});
    for (auto &[_, v] : update_capture_ids_) v = false;
  }
  update_capture_ids_.at(id) = true;
  return true;
}

bool AzureKinect::use_color_camera() const { return use_color_camera_; }

bool AzureKinect::use_depth_camera() const { return use_depth_camera_; }

const k4a::capture &AzureKinect::capture() const { return capture_; }

const k4a::calibration &AzureKinect::calibration() const {
  return calibration_;
}

const Transform3fA *AzureKinect::color2depth_pose() const {
  if (initial_set_up_)
    return &color2depth_pose_;
  else
    return nullptr;
}

const Transform3fA *AzureKinect::depth2color_pose() const {
  if (initial_set_up_)
    return &depth2color_pose_;
  else
    return nullptr;
}

AzureKinectColorCamera::AzureKinectColorCamera(const std::string &name,
                                               float image_scale,
                                               bool use_depth_as_world_frame)
    : ColorCamera{name},
      image_scale_{image_scale},
      use_depth_as_world_frame_{use_depth_as_world_frame},
      azure_kinect_{AzureKinect::GetInstance()} {
  azure_kinect_.UseColorCamera();
  azure_kinect_id_ = azure_kinect_.RegisterID();
}

AzureKinectColorCamera::AzureKinectColorCamera(
    const std::string &name, const std::filesystem::path &metafile_path)
    : ColorCamera{name, metafile_path},
      azure_kinect_{AzureKinect::GetInstance()} {
  azure_kinect_.UseColorCamera();
  azure_kinect_id_ = azure_kinect_.RegisterID();
}

AzureKinectColorCamera::~AzureKinectColorCamera() {
  azure_kinect_.UnregisterID(azure_kinect_id_);
}

bool AzureKinectColorCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!initial_set_up_ && !azure_kinect_.SetUp()) return false;
  if (use_depth_as_world_frame_)
    set_camera2world_pose(*azure_kinect_.color2depth_pose());
  GetIntrinsicsAndDistortionMap();
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  return UpdateImage(true);
}

void AzureKinectColorCamera::set_image_scale(float image_scale) {
  image_scale_ = image_scale;
  set_up_ = false;
}

void AzureKinectColorCamera::set_use_depth_as_world_frame(
    bool use_depth_as_world_frame) {
  use_depth_as_world_frame_ = use_depth_as_world_frame;
  set_up_ = false;
}

bool AzureKinectColorCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up azure kinect color camera " << name_ << " first"
              << std::endl;
    return false;
  }

  // Get and undistort image
  cv::Mat temp_image;
  azure_kinect_.UpdateCapture(azure_kinect_id_, synchronized);
  cv::cvtColor(
      cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_8UC4,
              (void *)azure_kinect_.capture().get_color_image().get_buffer(),
              cv::Mat::AUTO_STEP},
      temp_image, cv::COLOR_RGBA2RGB);
  cv::remap(temp_image, image_, distortion_map_, cv::Mat{}, cv::INTER_NEAREST,
            cv::BORDER_CONSTANT);

  SaveImageIfDesired();
  return true;
}

float AzureKinectColorCamera::image_scale() const { return image_scale_; }

bool AzureKinectColorCamera::use_depth_as_world_frame() const {
  return use_depth_as_world_frame_;
}

const Transform3fA *AzureKinectColorCamera::color2depth_pose() const {
  return azure_kinect_.color2depth_pose();
}

const Transform3fA *AzureKinectColorCamera::depth2color_pose() const {
  return azure_kinect_.depth2color_pose();
}

bool AzureKinectColorCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "image_scale", &image_scale_);
  ReadOptionalValueFromYaml(fs, "use_depth_as_world_frame",
                            &use_depth_as_world_frame_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

void AzureKinectColorCamera::GetIntrinsicsAndDistortionMap() {
  // Load intrinsics from camera
  const k4a_calibration_camera_t calibration{
      azure_kinect_.calibration().color_camera_calibration};
  const k4a_calibration_intrinsic_parameters_t::_param param =
      calibration.intrinsics.parameters.param;
  intrinsics_.fu = param.fx;
  intrinsics_.fv = param.fy;
  intrinsics_.ppu = param.cx;
  intrinsics_.ppv = param.cy;
  intrinsics_.width = calibration.resolution_width;
  intrinsics_.height = calibration.resolution_height;

  // Scale intrinsics according to image scale
  intrinsics_.fu *= image_scale_;
  intrinsics_.fv *= image_scale_;

  // Calculate distortion map
  cv::Mat1f camera_matrix(3, 3);
  camera_matrix << param.fx, 0, param.cx, 0, param.fy, param.cy, 0, 0, 1;
  cv::Mat1f new_camera_matrix(3, 3);
  new_camera_matrix << intrinsics_.fu, 0, intrinsics_.ppu, 0, intrinsics_.fv,
      intrinsics_.ppv, 0, 0, 1;
  cv::Mat1f distortion_coeff(1, 8);
  distortion_coeff << param.k1, param.k2, param.p1, param.p2, param.k3,
      param.k4, param.k5, param.k6;
  cv::Mat map1, map2, map3;
  cv::initUndistortRectifyMap(
      camera_matrix, distortion_coeff, cv::Mat{}, new_camera_matrix,
      cv::Size{intrinsics_.width, intrinsics_.height}, CV_32FC1, map1, map2);
  cv::convertMaps(map1, map2, distortion_map_, map3, CV_16SC2, true);
}

AzureKinectDepthCamera::AzureKinectDepthCamera(const std::string &name,
                                               float image_scale,
                                               bool use_color_as_world_frame,
                                               float depth_offset)
    : DepthCamera{name},
      image_scale_{image_scale},
      use_color_as_world_frame_{use_color_as_world_frame},
      depth_offset_{depth_offset},
      azure_kinect_{AzureKinect::GetInstance()} {
  azure_kinect_.UseDepthCamera();
  azure_kinect_id_ = azure_kinect_.RegisterID();
}

AzureKinectDepthCamera::AzureKinectDepthCamera(
    const std::string &name, const std::filesystem::path &metafile_path)
    : DepthCamera{name, metafile_path},
      azure_kinect_{AzureKinect::GetInstance()} {
  azure_kinect_.UseDepthCamera();
  azure_kinect_id_ = azure_kinect_.RegisterID();
}

AzureKinectDepthCamera::~AzureKinectDepthCamera() {
  azure_kinect_.UnregisterID(azure_kinect_id_);
}

bool AzureKinectDepthCamera::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;
  if (!initial_set_up_ && !azure_kinect_.SetUp()) return false;
  if (use_color_as_world_frame_)
    set_camera2world_pose(*azure_kinect_.depth2color_pose());
  GetIntrinsicsAndDistortionMap();
  SaveMetaDataIfDesired();
  set_up_ = true;
  initial_set_up_ = true;
  return UpdateImage(true);
}

void AzureKinectDepthCamera::set_image_scale(float image_scale) {
  image_scale_ = image_scale;
  set_up_ = false;
}

void AzureKinectDepthCamera::set_use_color_as_world_frame(
    bool use_color_as_world_frame) {
  use_color_as_world_frame_ = use_color_as_world_frame;
  set_up_ = false;
}

void AzureKinectDepthCamera::set_depth_offset(float depth_offset) {
  depth_offset_ = depth_offset;
}

bool AzureKinectDepthCamera::UpdateImage(bool synchronized) {
  if (!set_up_) {
    std::cerr << "Set up azure kinect depth camera " << name_ << " first"
              << std::endl;
    return false;
  }

  // Get and undistort image
  azure_kinect_.UpdateCapture(azure_kinect_id_, synchronized);
  cv::remap(
      cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_16UC1,
              (void *)azure_kinect_.capture().get_depth_image().get_buffer(),
              cv::Mat::AUTO_STEP},
      image_, distortion_map_, cv::Mat{}, cv::INTER_NEAREST,
      cv::BORDER_CONSTANT);

  // Add depth offset
  if (depth_offset_) {
    short depth_value_offset = depth_offset_ / depth_scale_;
    image_ += depth_value_offset;
  }

  SaveImageIfDesired();
  return true;
}

float AzureKinectDepthCamera::image_scale() const { return image_scale_; }

bool AzureKinectDepthCamera::use_color_as_world_frame() const {
  return use_color_as_world_frame_;
}

float AzureKinectDepthCamera::depth_offset() const { return depth_offset_; }

const Transform3fA *AzureKinectDepthCamera::color2depth_pose() const {
  return azure_kinect_.color2depth_pose();
}

const Transform3fA *AzureKinectDepthCamera::depth2color_pose() const {
  return azure_kinect_.depth2color_pose();
}

bool AzureKinectDepthCamera::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "image_scale", &image_scale_);
  ReadOptionalValueFromYaml(fs, "use_color_as_world_frame",
                            &use_color_as_world_frame_);
  ReadOptionalValueFromYaml(fs, "depth_offset", &depth_offset_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  world2camera_pose_ = camera2world_pose_.inverse();
  return true;
}

void AzureKinectDepthCamera::GetIntrinsicsAndDistortionMap() {
  // Load intrinsics from camera
  const k4a_calibration_camera_t calibration{
      azure_kinect_.calibration().depth_camera_calibration};
  const k4a_calibration_intrinsic_parameters_t::_param param =
      calibration.intrinsics.parameters.param;
  intrinsics_.fu = param.fx;
  intrinsics_.fv = param.fy;
  intrinsics_.ppu = param.cx;
  intrinsics_.ppv = param.cy;
  intrinsics_.width = calibration.resolution_width;
  intrinsics_.height = calibration.resolution_height;
  depth_scale_ = 0.001f;

  // Scale intrinsics according to image scale
  intrinsics_.fu *= image_scale_;
  intrinsics_.fv *= image_scale_;

  // Calculate distortion map
  cv::Mat1f camera_matrix(3, 3);
  camera_matrix << param.fx, 0, param.cx, 0, param.fy, param.cy, 0, 0, 1;
  cv::Mat1f new_camera_matrix(3, 3);
  new_camera_matrix << intrinsics_.fu, 0, intrinsics_.ppu, 0, intrinsics_.fv,
      intrinsics_.ppv, 0, 0, 1;
  cv::Mat1f distortion_coeff(1, 8);
  distortion_coeff << param.k1, param.k2, param.p1, param.p2, param.k3,
      param.k4, param.k5, param.k6;
  cv::Mat map1, map2, map3;
  cv::initUndistortRectifyMap(
      camera_matrix, distortion_coeff, cv::Mat{}, new_camera_matrix,
      cv::Size{intrinsics_.width, intrinsics_.height}, CV_32FC1, map1, map2);
  cv::convertMaps(map1, map2, distortion_map_, map3, CV_16SC2, true);
}

}  // namespace m3t
