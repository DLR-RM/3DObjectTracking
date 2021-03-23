// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Manuel Stoiber, German Aerospace Center (DLR)

#include <rbgt/renderer_geometry.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader/tiny_obj_loader.h>

namespace rbgt {

int RendererGeometry::n_instances_ = 0;

RendererGeometry::RendererGeometry() { n_instances_++; }

RendererGeometry::~RendererGeometry() {
  n_instances_--;
  if (window_ != nullptr) {
    glfwMakeContextCurrent(window_);
    for (auto &render_data_body : render_data_bodies_) {
      glDeleteBuffers(1, &render_data_body.vbo);
      glDeleteVertexArrays(1, &render_data_body.vao);
    }
    glfwDestroyWindow(window_);
    window_ = nullptr;
    if (n_instances_ == 0) {
      glfwTerminate();
    }
  }
}

bool RendererGeometry::Init() {
  if (window_ == nullptr) {
    if (!glfwInit()) {
      std::cerr << "Failed to initialize GLFW" << std::endl;
      return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

    window_ = glfwCreateWindow(640, 480, "window", nullptr, nullptr);
    if (window_ == nullptr) {
      std::cerr << "Failed to create GLFW window" << std::endl;
      glfwTerminate();
      return false;
    }
    glfwMakeContextCurrent(window_);

    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
      std::cerr << "Failed to initialize GLEW" << std::endl;
      glfwDestroyWindow(window_);
      window_ = nullptr;
      glfwTerminate();
      return false;
    }
  }
  initialized_ = true;
  return true;
}

bool RendererGeometry::AddBody(std::shared_ptr<Body> body_ptr) {
  // Check if class is already initialized
  if (!initialized_) {
    if (!Init()) return false;
  }

  // Check if renderer geometry for body already exists
  for (auto &render_data_body : render_data_bodies_) {
    if (body_ptr->name() == render_data_body.body_ptr->name()) {
      std::cerr << "Body data \"" << body_ptr->name() << "\" already exists"
                << std::endl;
      return false;
    }
  }

  // Create data for body and assign parameters
  RenderDataBody render_data_body;
  render_data_body.body_ptr = std::move(body_ptr);

  std::vector<float> vertices;  // vertices are input to graphics pipeline
  if (!LoadMeshIntoVertices(render_data_body.body_ptr, &vertices)) return false;
  render_data_body.n_vertices = int(vertices.size()) / 6;

  // Create vertex array object and vertex buffer object
  glfwMakeContextCurrent(window_);
  glGenVertexArrays(1, &render_data_body.vao);
  glBindVertexArray(render_data_body.vao);

  glGenBuffers(1, &render_data_body.vbo);
  glBindBuffer(GL_ARRAY_BUFFER, render_data_body.vbo);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
               &vertices.front(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
  glfwMakeContextCurrent(nullptr);

  // Add body data
  render_data_bodies_.push_back(std::move(render_data_body));
  return true;
}

bool RendererGeometry::DeleteBody(const std::string &name) {
  // Search and delete body
  for (size_t i = 0; i < render_data_bodies_.size(); ++i) {
    if (name == render_data_bodies_[i].body_ptr->name()) {
      glfwMakeContextCurrent(window_);
      glDeleteBuffers(1, &render_data_bodies_[i].vbo);
      glDeleteVertexArrays(1, &render_data_bodies_[i].vao);
      glfwMakeContextCurrent(nullptr);
      render_data_bodies_.erase(render_data_bodies_.begin() + i);
      return true;
    }
  }
  std::cerr << "Body data \"" << name << "\" not found" << std::endl;
  return false;
}

void RendererGeometry::ClearBodies() {
  for (auto &render_data_body : render_data_bodies_) {
    glfwMakeContextCurrent(window_);
    glDeleteBuffers(1, &render_data_body.vbo);
    glDeleteVertexArrays(1, &render_data_body.vao);
    glfwMakeContextCurrent(nullptr);
  }
  render_data_bodies_.clear();
}

const std::vector<RendererGeometry::RenderDataBody>
    &RendererGeometry::render_data_bodies() const {
  return render_data_bodies_;
}

GLFWwindow *RendererGeometry::window() { return window_; }

bool RendererGeometry::initialized() const { return initialized_; }

bool RendererGeometry::LoadMeshIntoVertices(std::shared_ptr<Body> body_ptr,
                                            std::vector<float> *vertices) {
  tinyobj::attrib_t attributes;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warning;
  std::string error;

  if (!tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error,
                        body_ptr->geometry_path().string().c_str(), nullptr,
                        true, false)) {
    std::cerr << "TinyObjLoader failed to load data" << std::endl;
    return false;
  }
  if (!error.empty()) std::cerr << error << std::endl;

  vertices->clear();
  for (auto &shape : shapes) {
    int index_offset = 0;
    for (int f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
      if (shape.mesh.num_face_vertices[f] != 3) {
        std::cerr << "Mesh contains non triangle shapes" << std::endl;
        index_offset += shape.mesh.num_face_vertices[f];
        continue;
      }

      // Extract triangle points
      Eigen::Vector3f points[3];
      for (int v = 0; v < 3; ++v) {
        int idx = 3 * shape.mesh.indices[index_offset + v].vertex_index;
        if (body_ptr->geometry_counterclockwise()) {
          points[v][0] = float(attributes.vertices[idx + 0]);
          points[v][1] = float(attributes.vertices[idx + 1]);
          points[v][2] = float(attributes.vertices[idx + 2]);
          points[v] *= body_ptr->geometry_unit_in_meter();
        } else {
          points[2 - v][0] = float(attributes.vertices[idx + 0]);
          points[2 - v][1] = float(attributes.vertices[idx + 1]);
          points[2 - v][2] = float(attributes.vertices[idx + 2]);
          points[2 - v] *= body_ptr->geometry_unit_in_meter();
        }
      }

      // Calculate normal vector
      Eigen::Vector3f normal{
          (points[2] - points[1]).cross(points[0] - points[1]).normalized()};

      // Save data in vertices vector
      for (auto point : points) {
        vertices->insert(vertices->end(), point.data(), point.data() + 3);
        vertices->insert(vertices->end(), normal.data(), normal.data() + 3);
      }

      index_offset += 3;
    }
  }
  return true;
}

}  // namespace rbgt
