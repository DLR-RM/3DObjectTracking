// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_RBGT_RENDERER_GEOMETRY_H_
#define OBJECT_TRACKING_INCLUDE_RBGT_RENDERER_GEOMETRY_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <rbgt/body.h>
#include <rbgt/common.h>

#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace rbgt {

// Class that holds rendering data for all assigned bodies as well as the glfw
// context for the renderer
class RendererGeometry {
 private:
  // Count number of instances
  static int n_instances_;

 public:
  // Data Structs
  using RenderDataBody = struct RenderDataBody {
    std::shared_ptr<Body> body_ptr = nullptr;
    GLuint vao = 0;
    GLuint vbo = 0;
    unsigned int n_vertices = 0;
  };

  // Constructors and destructors
  RendererGeometry();
  ~RendererGeometry();

  // Main methods
  bool Init();  // creates glfw context
  bool AddBody(std::shared_ptr<Body> body_ptr);
  bool DeleteBody(const std::string &name);
  void ClearBodies();

  // Getters
  const std::vector<RenderDataBody> &render_data_bodies() const;
  GLFWwindow *window();
  bool initialized() const;

 private:
  // Helper methods
  static bool LoadMeshIntoVertices(std::shared_ptr<Body> body_ptr,
                                   std::vector<float> *vertices);

  // Variables
  std::vector<RenderDataBody> render_data_bodies_;
  GLFWwindow *window_ = nullptr;  // only used to hold a glfw context
  bool initialized_ = false;
};

}  // namespace rbgt

#endif  // OBJECT_TRACKING_INCLUDE_RBGT_RENDERER_GEOMETRY_H_
