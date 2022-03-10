// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/depth_model.h>

namespace icg {

DepthModel::DepthModel(const std::string &name,
                       const std::shared_ptr<Body> &body_ptr,
                       const std::filesystem::path &model_path,
                       float sphere_radius, int n_divides, int n_points,
                       float max_radius_depth_offset, float stride_depth_offset,
                       bool use_random_seed, int image_size)
    : Model{name,
            body_ptr,
            model_path,
            sphere_radius,
            n_divides,
            n_points,
            max_radius_depth_offset,
            stride_depth_offset,
            use_random_seed,
            image_size} {}

DepthModel::DepthModel(const std::string &name,
                       const std::filesystem::path &metafile_path,
                       const std::shared_ptr<Body> &body_ptr)
    : Model{name, metafile_path, body_ptr} {}

bool DepthModel::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }

  if (!DepthOffsetVariablesValid()) return false;
  if (use_random_seed_ || !LoadModel()) {
    if (!GenerateModel()) return false;
    if (!SaveModel()) return false;
  }
  set_up_ = true;
  return true;
}

bool DepthModel::GetClosestView(const Transform3fA &body2camera_pose,
                                const View **closest_view) const {
  if (!set_up_) {
    std::cerr << "Set up depth model " << name_ << " first" << std::endl;
    return false;
  }

  if (body2camera_pose.translation().norm() == 0.0f) {
    *closest_view = &views_[0];
    return true;
  }

  Eigen::Vector3f orientation{
      body2camera_pose.rotation().inverse() *
      body2camera_pose.translation().matrix().normalized()};

  float closest_dot = -1.0f;
  for (auto &view : views_) {
    float dot = orientation.dot(view.orientation);
    if (dot > closest_dot) {
      *closest_view = &view;
      closest_dot = dot;
    }
  }
  return true;
}

bool DepthModel::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!ReadRequiredValueFromYaml(fs, "model_path", &model_path_)) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "sphere_radius", &sphere_radius_);
  ReadOptionalValueFromYaml(fs, "n_divides", &n_divides_);
  ReadOptionalValueFromYaml(fs, "n_points", &n_points_);
  ReadOptionalValueFromYaml(fs, "max_radius_depth_offset",
                            &max_radius_depth_offset_);
  ReadOptionalValueFromYaml(fs, "stride_depth_offset", &stride_depth_offset_);
  ReadOptionalValueFromYaml(fs, "use_random_seed", &use_random_seed_);
  ReadOptionalValueFromYaml(fs, "image_size", &image_size_);
  fs.release();

  // Process parameters
  if (model_path_ == "INFER_FROM_NAME")
    model_path_ = metafile_path_.parent_path() / (name_ + ".bin");
  else if (model_path_.is_relative())
    model_path_ = metafile_path_.parent_path() / model_path_;
  return true;
}

bool DepthModel::GenerateModel() {
  // Generate camera poses
  std::vector<Transform3fA> camera2body_poses;
  GenerateGeodesicPoses(&camera2body_poses);

  // Create RendererGeometries in main thread to comply with GLFW thread safety
  // requirements
  std::vector<std::shared_ptr<RendererGeometry>> renderer_geometry_ptrs(
      omp_get_max_threads());
  for (auto &renderer_geometry_ptr : renderer_geometry_ptrs) {
    renderer_geometry_ptr = std::make_shared<RendererGeometry>("rg");
    renderer_geometry_ptr->SetUp();
  }

  // Generate template views
  std::cout << "Start generating model " << name_ << std::endl;
  views_.resize(camera2body_poses.size());
  bool cancel = false;
  std::atomic<int> count = 1;
#pragma omp parallel
  {
    std::shared_ptr<FullNormalRenderer> renderer_ptr;
    if (!SetUpRenderer(renderer_geometry_ptrs[omp_get_thread_num()],
                       &renderer_ptr))
      cancel = true;

#pragma omp for
    for (int i = 0; i < int(views_.size()); ++i) {
      if (cancel) continue;
      std::stringstream msg;
      msg << "Generate depth model " << name_ << ": view " << count++ << " of "
          << views_.size() << std::endl;
      std::cout << msg.str();

      // Render images
      renderer_ptr->set_camera2world_pose(camera2body_poses[i]);
      renderer_ptr->StartRendering();
      renderer_ptr->FetchNormalImage();
      renderer_ptr->FetchDepthImage();

      // Generate data
      views_[i].orientation =
          camera2body_poses[i].matrix().col(2).segment(0, 3);
      views_[i].data_points.resize(n_points_);
      if (!GeneratePointData(*renderer_ptr, camera2body_poses[i],
                             &views_[i].data_points))
        cancel = true;
    }
  }
  if (cancel) return false;
  std::cout << "Finish generating model " << name_ << std::endl;
  return true;
}

bool DepthModel::LoadModel() {
  std::ifstream ifs{model_path_, std::ios::in | std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cout << "Could not open model file " << model_path_ << std::endl;
    return false;
  }

  if (!LoadModelParameters(kVersionID, kModelType, &ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different model parameters" << std::endl;
    return false;
  }

  if (!LoadBodyData(&ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different body parameters" << std::endl;
    return false;
  }

  // Load view data
  size_t n_views;
  ifs.read((char *)(&n_views), sizeof(n_views));
  views_.clear();
  views_.reserve(n_views);
  for (size_t i = 0; i < n_views; i++) {
    View tv;
    tv.data_points.resize(n_points_);
    ifs.read((char *)(tv.data_points.data()), n_points_ * sizeof(DataPoint));
    ifs.read((char *)(tv.orientation.data()), sizeof(tv.orientation));
    views_.push_back(std::move(tv));
  }
  ifs.close();
  return true;
}

bool DepthModel::SaveModel() const {
  std::ofstream ofs{model_path_, std::ios::out | std::ios::binary};
  SaveModelParameters(kVersionID, kModelType, &ofs);
  SaveBodyData(&ofs);

  // Save main data
  size_t n_views = views_.size();
  ofs.write((const char *)(&n_views), sizeof(n_views));
  for (const auto &v : views_) {
    ofs.write((const char *)(v.data_points.data()),
              n_points_ * sizeof(DataPoint));
    ofs.write((const char *)(v.orientation.data()), sizeof(v.orientation));
  }
  ofs.flush();
  ofs.close();
  return true;
}

bool DepthModel::GeneratePointData(const FullNormalRenderer &renderer,
                                   const Transform3fA &camera2body_pose,
                                   std::vector<DataPoint> *data_points) const {
  // Compute silhouette
  std::vector<cv::Mat> normal_image_channels(4);
  cv::split(renderer.normal_image(), normal_image_channels);
  cv::Mat &silhouette_image{normal_image_channels[3]};
  if (!IsSilhouetteValid(silhouette_image)) return false;

  // Set up generator
  std::mt19937 generator{7};
  if (use_random_seed_)
    generator.seed(
        unsigned(std::chrono::system_clock::now().time_since_epoch().count()));

  // Calculate data for contour points
  for (auto data_point{begin(*data_points)}; data_point != end(*data_points);) {
    // Randomly sample point on contour and calculate 3D center and normal
    cv::Point2i center{
        SampleSurfacePointCoordinate(silhouette_image, generator)};
    Eigen::Vector3f center_f_camera{renderer.PointVector(center)};
    Eigen::Vector3f normal_f_camera{renderer.NormalVector(center)};
    data_point->center_f_body = camera2body_pose * center_f_camera;
    data_point->normal_f_body = camera2body_pose.rotation() * normal_f_camera;

    // Calculate depth offsets
    float pixel_to_meter = center_f_camera(2) / renderer.intrinsics().fu;
    CalculateDepthOffsets(renderer, center, pixel_to_meter,
                          &data_point->depth_offsets);

    data_point++;
  }
  return true;
}

bool DepthModel::IsSilhouetteValid(const cv::Mat &silhouette_image) {
  if (cv::countNonZero(silhouette_image) < kMinSilhouetteSize) {
    std::cerr << "No valid silhouette in image: silhouette is too small"
              << std::endl;
    return false;
  }
  return true;
}

cv::Point2i DepthModel::SampleSurfacePointCoordinate(
    const cv::Mat &silhouette_image, std::mt19937 &generator) {
  unsigned int n_pixels = silhouette_image.cols * silhouette_image.rows;
  while (true) {
    int idx = int(generator() % n_pixels);
    cv::Point2i coordinate{idx / silhouette_image.rows,
                           idx % silhouette_image.cols};
    if (silhouette_image.at<uchar>(coordinate)) return coordinate;
  }
}

}  // namespace icg
