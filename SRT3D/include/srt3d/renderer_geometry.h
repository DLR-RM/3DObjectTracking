// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#ifndef OBJECT_TRACKING_INCLUDE_SRT3D_RENDERER_GEOMETRY_H_
#define OBJECT_TRACKING_INCLUDE_SRT3D_RENDERER_GEOMETRY_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <srt3d/body.h>
#include <srt3d/common.h>

#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace srt3d {

// Class that holds rendering data for all assigned bodies as well as the glfw
// context for the renderer
class RendererGeometry {
 private:
  // Count the number of instances to manage the GLFW library
  static int n_instances_;

 public:
  // Data Structs
  using RenderDataBody = struct RenderDataBody {
    std::shared_ptr<Body> body_ptr = nullptr;
    GLuint vao = 0;
    GLuint vbo = 0;
    unsigned n_vertices = 0;
  };

  // Constructor, destructor, and setup method
  // Both the destructor and setup method have to be called from
  // the main thread to comply with GLFW thread safety requirements
  RendererGeometry(const std::string &name);
  ~RendererGeometry();  // deletes glfw context
  bool SetUp();         // creates glfw context

  // Main methods
  bool AddBody(std::shared_ptr<Body> body_ptr, bool verbose = true);
  bool DeleteBody(const std::string &name, bool verbose = true);
  void ClearBodies();

  // Handling of GLFW context
  void MakeContextCurrent();
  void DetachContext();

  // Getters
  const std::string &name() const;
  const std::vector<RenderDataBody> &render_data_bodies() const;
  bool set_up() const;

 private:
  // Helper methods
  static bool LoadMeshIntoVertices(const Body &body,
                                   std::vector<float> *vertices);
  static void CreateGLVertexObjects(const std::vector<float> &vertices,
                                    RenderDataBody *render_data_body);
  static void DeleteGLVertexObjects(RenderDataBody *render_data_body);

  // Variables
  std::string name_{};
  std::vector<RenderDataBody> render_data_bodies_;
  GLFWwindow *window_ = nullptr;  // only used to hold a glfw context
  std::mutex mutex_;
  bool initial_set_up_ = false;
  bool set_up_ = false;
};

}  // namespace srt3d

#endif  // OBJECT_TRACKING_INCLUDE_SRT3D_RENDERER_GEOMETRY_H_
