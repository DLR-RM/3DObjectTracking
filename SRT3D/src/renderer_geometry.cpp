// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/renderer_geometry.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader/tiny_obj_loader.h>

namespace srt3d {

int RendererGeometry::n_instances_ = 0;

RendererGeometry::RendererGeometry(const std::string &name) : name_{name} {}

RendererGeometry::~RendererGeometry() {
  if (initial_set_up_) {
    glfwMakeContextCurrent(window_);
    for (auto &render_data_body : render_data_bodies_) {
      DeleteGLVertexObjects(&render_data_body);
    }
    glfwMakeContextCurrent(0);
    glfwDestroyWindow(window_);
    window_ = nullptr;
    n_instances_--;
    if (n_instances_ == 0) glfwTerminate();
  }
}

bool RendererGeometry::SetUp() {
  // Set up GLFW
  if (!initial_set_up_) {
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
    glfwMakeContextCurrent(nullptr);

    n_instances_++;
    initial_set_up_ = true;
  }

  // Start with normal setup
  set_up_ = false;

  // Set up bodies
  glfwMakeContextCurrent(window_);
  for (auto &render_data_body : render_data_bodies_) {
    // Load vertices of mesh
    std::vector<float> vertices;
    if (!LoadMeshIntoVertices(*render_data_body.body_ptr.get(), &vertices))
      return false;
    render_data_body.n_vertices = unsigned(vertices.size()) / 6;

    // Create GL Vertex objects
    if (set_up_) DeleteGLVertexObjects(&render_data_body);
    CreateGLVertexObjects(vertices, &render_data_body);
  }
  glfwMakeContextCurrent(nullptr);

  set_up_ = true;
  return true;
}

bool RendererGeometry::AddBody(std::shared_ptr<Body> body_ptr, bool verbose) {
  const std::lock_guard<std::mutex> lock{mutex_};

  // Check if renderer geometry for body already exists
  for (auto &render_data_body : render_data_bodies_) {
    if (body_ptr->name() == render_data_body.body_ptr->name()) {
      if (verbose)
        std::cerr << "Body data " << body_ptr->name() << " already exists"
                  << std::endl;
      return false;
    }
  }

  // Create data for body and assign parameters
  RenderDataBody render_data_body;
  render_data_body.body_ptr = std::move(body_ptr);
  if (set_up_) {
    // Load vertices of mesh
    std::vector<float> vertices;
    if (!LoadMeshIntoVertices(*render_data_body.body_ptr.get(), &vertices))
      return false;
    render_data_body.n_vertices = unsigned(vertices.size()) / 6;

    // Create GL Vertex objects
    glfwMakeContextCurrent(window_);
    CreateGLVertexObjects(vertices, &render_data_body);
    glfwMakeContextCurrent(nullptr);
  }

  // Add body data
  render_data_bodies_.push_back(std::move(render_data_body));
  return true;
}

bool RendererGeometry::DeleteBody(const std::string &name, bool verbose) {
  const std::lock_guard<std::mutex> lock{mutex_};
  for (size_t i = 0; i < render_data_bodies_.size(); ++i) {
    if (name == render_data_bodies_[i].body_ptr->name()) {
      if (set_up_) {
        glfwMakeContextCurrent(window_);
        DeleteGLVertexObjects(&render_data_bodies_[i]);
        glfwMakeContextCurrent(nullptr);
      }
      render_data_bodies_.erase(begin(render_data_bodies_) + i);
      return true;
    }
  }
  if (verbose)
    std::cerr << "Body data \"" << name << "\" not found" << std::endl;
  return false;
}

void RendererGeometry::ClearBodies() {
  const std::lock_guard<std::mutex> lock{mutex_};
  if (set_up_) {
    glfwMakeContextCurrent(window_);
    for (auto &render_data_body : render_data_bodies_) {
      DeleteGLVertexObjects(&render_data_body);
    }
    glfwMakeContextCurrent(nullptr);
  }
  render_data_bodies_.clear();
}

void RendererGeometry::MakeContextCurrent() {
  mutex_.lock();
  glfwMakeContextCurrent(window_);
}

void RendererGeometry::DetachContext() {
  glfwMakeContextCurrent(nullptr);
  mutex_.unlock();
}

const std::string &RendererGeometry::name() const { return name_; }

const std::vector<RendererGeometry::RenderDataBody>
    &RendererGeometry::render_data_bodies() const {
  return render_data_bodies_;
}

bool RendererGeometry::set_up() const { return set_up_; }

bool RendererGeometry::LoadMeshIntoVertices(const Body &body,
                                            std::vector<float> *vertices) {
  tinyobj::attrib_t attributes;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warning;
  std::string error;

  if (!tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error,
                        body.geometry_path().string().c_str(), nullptr, true,
                        false)) {
    std::cerr << "TinyObjLoader failed to load data from "
              << body.geometry_path() << std::endl;
    return false;
  }
  if (!error.empty()) std::cerr << error << std::endl;

  vertices->clear();
  for (auto &shape : shapes) {
    size_t index_offset = 0;
    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
      if (shape.mesh.num_face_vertices[f] != 3) {
        std::cerr << "Mesh contains non triangle shapes" << std::endl;
        index_offset += shape.mesh.num_face_vertices[f];
        continue;
      }

      // Extract triangle points
      std::array<Eigen::Vector3f, 3> points;
      for (int v = 0; v < 3; ++v) {
        int idx = 3 * shape.mesh.indices[index_offset + v].vertex_index;
        if (body.geometry_counterclockwise()) {
          points[v](0) = float(attributes.vertices[idx + 0]);
          points[v](1) = float(attributes.vertices[idx + 1]);
          points[v](2) = float(attributes.vertices[idx + 2]);
          points[v] *= body.geometry_unit_in_meter();
        } else {
          points[2 - v](0) = float(attributes.vertices[idx + 0]);
          points[2 - v](1) = float(attributes.vertices[idx + 1]);
          points[2 - v](2) = float(attributes.vertices[idx + 2]);
          points[2 - v] *= body.geometry_unit_in_meter();
        }
      }

      // Calculate normal vector
      Eigen::Vector3f normal{
          (points[2] - points[1]).cross(points[0] - points[1]).normalized()};

      // Save data in vertices vector
      for (auto point : points) {
        vertices->insert(end(*vertices), point.data(), point.data() + 3);
        vertices->insert(end(*vertices), normal.data(), normal.data() + 3);
      }

      index_offset += 3;
    }
  }
  return true;
}

void RendererGeometry::CreateGLVertexObjects(const std::vector<float> &vertices,
                                             RenderDataBody *render_data_body) {
  glGenVertexArrays(1, &render_data_body->vao);
  glBindVertexArray(render_data_body->vao);

  glGenBuffers(1, &render_data_body->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, render_data_body->vbo);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
               &vertices.front(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void RendererGeometry::DeleteGLVertexObjects(RenderDataBody *render_data_body) {
  glDeleteBuffers(1, &render_data_body->vbo);
  glDeleteVertexArrays(1, &render_data_body->vao);
}

}  // namespace srt3d
