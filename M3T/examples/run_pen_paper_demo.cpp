// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <m3t/generator.h>
#include <m3t/tracker.h>

/**
 * Class that visualizes the drawing of a stabilo pen on a paper
 */
class DrawingPublisher : public m3t::Publisher {
 private:
  // DIN A4 paper dimensions 210mmx297mm
  static constexpr int kDefaultImageWidth = 210;
  static constexpr int kDefaultImageHeight = 297;
  static constexpr float kA4PaperWidthOffset = 0.105f;
  static constexpr float kA4PaperHeightOffset = 0.1485f;
  static constexpr float kA4PaperDepthOffset = 0.0f;

 public:
  DrawingPublisher(const std::string& name,
                   const std::shared_ptr<m3t::Body>& paper_body_ptr,
                   const std::shared_ptr<m3t::Body>& stabilo_body_ptr,
                   const std::string& window_name = "AR Drawing",
                   float scaling_factor = 2.0f)
      : Publisher{name},
        paper_body_ptr_{paper_body_ptr},
        stabilo_body_ptr_{stabilo_body_ptr},
        window_name_{window_name},
        scaling_factor_{scaling_factor} {}

  void StartSavingImages(const std::filesystem::path& save_directory,
                         const std::string& save_image_type = "png") {
    save_images_ = true;
    save_directory_ = save_directory;
    save_image_type_ = save_image_type;
  }

  void StopSavingImages() { save_images_ = false; }

  bool SetUp() override {
    image_ = cv::Mat{cv::Size2i{int(kDefaultImageWidth * scaling_factor_),
                                int(kDefaultImageHeight * scaling_factor_)},
                     CV_8UC1, 255};
    set_up_ = true;
    return true;
  }

  bool UpdatePublisher(int iteration) override {
    // Calculate pose between tip and paper frame
    m3t::Transform3fA tip2paper_pose{paper_body_ptr_->world2body_pose() *
                                     stabilo_body_ptr_->body2world_pose()};
    Eigen::Vector3f tip2paper_trans{tip2paper_pose.translation()};

    // Draw on image if stabilo is close enough to paper
    if (tip2paper_trans.x() < kA4PaperWidthOffset &&
        tip2paper_trans.x() > -kA4PaperWidthOffset &&
        tip2paper_trans.y() < kA4PaperHeightOffset &&
        tip2paper_trans.y() > -kA4PaperHeightOffset &&
        tip2paper_trans.z() < kA4PaperDepthOffset) {
      float x = (tip2paper_trans.x() + kA4PaperWidthOffset) * 1000.0f *
                scaling_factor_;
      float y = (kA4PaperHeightOffset - tip2paper_trans.y()) * 1000.0f *
                scaling_factor_;
      cv::Point2i coordinate{int(x + 0.5f), int(y + 0.5f)};
      if (writing_) {
        int thickness = int(2.0f * scaling_factor_ + 0.5f);
        cv::line(image_, coordinate, previous_coordinate_, 0, thickness,
                 cv::LINE_4);
      }
      previous_coordinate_ = coordinate;
      writing_ = true;
    } else {
      writing_ = false;
    }

    // Display and save image
    cv::imshow(window_name_, image_);
    if (save_images_) {
      std::filesystem::path path{save_directory_ / (name_ + "_image_" +
                                                    std::to_string(iteration) +
                                                    "." + save_image_type_)};
      cv::imwrite(path.string(), image_);
    }
    return true;
  }

 private:
  // Parameters
  std::shared_ptr<m3t::Body> paper_body_ptr_{};
  std::shared_ptr<m3t::Body> stabilo_body_ptr_{};
  std::string window_name_;
  float scaling_factor_;
  std::filesystem::path save_directory_{};
  std::string save_image_type_ = "png";
  bool save_images_ = false;

  // Internal data
  bool writing_ = false;
  cv::Point2i previous_coordinate_;
  cv::Mat image_;
};

int main(int argc, char* argv[]) {
  const std::filesystem::path configfile_path{
      "../../data/pen_paper_demo/config.yaml"};

  // Generate tracker
  std::shared_ptr<m3t::Tracker> tracker_ptr;
  if (!GenerateConfiguredTracker(configfile_path, &tracker_ptr)) return -1;
  if (!tracker_ptr->SetUp()) return -1;

  // Search required bodies
  std::shared_ptr<m3t::Body> paper_body_ptr{};
  std::shared_ptr<m3t::Body> stabilo_body_ptr{};
  for (const auto& body_ptr : tracker_ptr->body_ptrs()) {
    if (body_ptr->name() == "paper") paper_body_ptr = body_ptr;
    if (body_ptr->name() == "stabilo") stabilo_body_ptr = body_ptr;
  }
  if (!paper_body_ptr || !stabilo_body_ptr) return -1;

  // Add publisher that updates drawing
  auto drawing_publisher_ptr{std::make_shared<DrawingPublisher>(
      DrawingPublisher("Drawing Publisher", paper_body_ptr, stabilo_body_ptr))};
  tracker_ptr->AddPublisher(drawing_publisher_ptr);

  // Setup and run tracker
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return 0;
  return 0;
}
