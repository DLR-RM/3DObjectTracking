// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/renderer_geometry.h>

namespace m3t {

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
  const std::lock_guard<std::mutex> lock{mutex_};
  set_up_ = false;

  // Check if all required objects are set up
  for (auto &body_ptr : body_ptrs_) {
    if (!body_ptr->set_up()) {
      std::cerr << "Body " << body_ptr->name() << " was not set up"
                << std::endl;
      return false;
    }
  }

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

  // Set up bodies
  glfwMakeContextCurrent(window_);
  for (auto &render_data_body : render_data_bodies_) {
    // Assemble vertex data
    std::vector<float> vertex_data;
    AssembleVertexData(*render_data_body.body_ptr, &vertex_data);
    render_data_body.n_vertices = unsigned(vertex_data.size()) / 6;

    // Create GL Vertex objects
    if (set_up_) DeleteGLVertexObjects(&render_data_body);
    CreateGLVertexObjects(vertex_data, &render_data_body);
  }
  glfwMakeContextCurrent(nullptr);

  set_up_ = true;
  return true;
}

bool RendererGeometry::AddBody(const std::shared_ptr<Body> &body_ptr) {
  const std::lock_guard<std::mutex> lock{mutex_};

  // Check if renderer geometry for body already exists
  for (auto &p : body_ptrs_) {
    if (body_ptr->name() == p->name()) {
      std::cerr << "Body data " << body_ptr->name() << " already exists"
                << std::endl;
      return false;
    }
  }

  // Create data for body and assign parameters
  RenderDataBody render_data_body;
  render_data_body.body_ptr = body_ptr.get();
  if (set_up_ && body_ptr->set_up()) {
    // Assemble vertex data
    std::vector<float> vertex_data;
    AssembleVertexData(*body_ptr.get(), &vertex_data);
    render_data_body.n_vertices = unsigned(vertex_data.size()) / 6;

    // Create GL Vertex objects
    glfwMakeContextCurrent(window_);
    CreateGLVertexObjects(vertex_data, &render_data_body);
    glfwMakeContextCurrent(nullptr);
  } else if (set_up_ && !body_ptr->set_up()) {
    set_up_ = false;
  }

  // Add body ptr and body data
  body_ptrs_.push_back(body_ptr);
  render_data_bodies_.push_back(std::move(render_data_body));
  return true;
}

bool RendererGeometry::DeleteBody(const std::string &name) {
  const std::lock_guard<std::mutex> lock{mutex_};
  for (size_t i = 0; i < body_ptrs_.size(); ++i) {
    if (name == body_ptrs_[i]->name()) {
      body_ptrs_.erase(begin(body_ptrs_) + i);
      if (set_up_) {
        glfwMakeContextCurrent(window_);
        DeleteGLVertexObjects(&render_data_bodies_[i]);
        glfwMakeContextCurrent(nullptr);
      }
      render_data_bodies_.erase(begin(render_data_bodies_) + i);
      return true;
    }
  }
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
  body_ptrs_.clear();
}

bool RendererGeometry::MakeContextCurrent() {
  mutex_.lock();
  if (!initial_set_up_) {
    std::cerr << "Set up renderer geometry " << name_ << " first" << std::endl;
    mutex_.unlock();
    return false;
  }
  glfwMakeContextCurrent(window_);
  return true;
}

bool RendererGeometry::DetachContext() {
  if (!initial_set_up_) {
    std::cerr << "Set up renderer geometry " << name_ << " first" << std::endl;
    return false;
  }
  glfwMakeContextCurrent(nullptr);
  mutex_.unlock();
  return true;
}

const std::string &RendererGeometry::name() const { return name_; }

const std::vector<std::shared_ptr<Body>> &RendererGeometry::body_ptrs() const {
  return body_ptrs_;
}

const std::vector<RendererGeometry::RenderDataBody>
    &RendererGeometry::render_data_bodies() const {
  return render_data_bodies_;
}

bool RendererGeometry::set_up() const { return set_up_; }

void RendererGeometry::AssembleVertexData(const Body &body,
                                          std::vector<float> *vertex_data) {
  for (const auto &triangle_indices : body.mesh_indices()) {
    std::array<Eigen::Vector3f, 3> points;
    for (int i = 0; i < 3; ++i)
      points[i] = body.vertices()[triangle_indices[i]];

    Eigen::Vector3f normal{
        (points[2] - points[1]).cross(points[0] - points[1]).normalized()};

    for (auto point : points) {
      vertex_data->insert(end(*vertex_data), point.data(), point.data() + 3);
      vertex_data->insert(end(*vertex_data), normal.data(), normal.data() + 3);
    }
  }
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

}  // namespace m3t
