// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <gtest/gtest.h>
#include <m3t/body.h>
#include <m3t/renderer_geometry.h>

#include "common_test.h"

class RendererGeometryTest : public testing::Test {
 protected:
  void SetUp() override {
    renderer_geometry_ptr_ =
        std::make_shared<m3t::RendererGeometry>("renderer_geometry");
    triangle_body_ptr_ = TriangleBodyPtr();
    schauma_body_ptr_ = SchaumaBodyPtr();
  }

  bool TestRendererDataSchauma(
      const m3t::RendererGeometry::RenderDataBody &renderer_data_schauma) {
    return renderer_data_schauma.body_ptr->name() == "schauma" &&
           renderer_data_schauma.vao != 0 && renderer_data_schauma.vao != 0 &&
           renderer_data_schauma.n_vertices == 62850;
  }

  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_ptr_;
  std::shared_ptr<m3t::Body> triangle_body_ptr_;
  std::shared_ptr<m3t::Body> schauma_body_ptr_;
};

TEST_F(RendererGeometryTest, SetUp) {
  ASSERT_TRUE(renderer_geometry_ptr_->SetUp());
  ASSERT_TRUE(renderer_geometry_ptr_->set_up());
}

TEST_F(RendererGeometryTest, AddDeleteBodiesBeforeSetUp) {
  ASSERT_TRUE(renderer_geometry_ptr_->AddBody(triangle_body_ptr_));
  ASSERT_TRUE(renderer_geometry_ptr_->AddBody(schauma_body_ptr_));
  ASSERT_TRUE(renderer_geometry_ptr_->DeleteBody("triangle"));
  ASSERT_TRUE(renderer_geometry_ptr_->SetUp());
  ASSERT_TRUE(
      TestRendererDataSchauma(renderer_geometry_ptr_->render_data_bodies()[0]));
  ASSERT_TRUE(renderer_geometry_ptr_->SetUp());
  ASSERT_TRUE(
      TestRendererDataSchauma(renderer_geometry_ptr_->render_data_bodies()[0]));
}

TEST_F(RendererGeometryTest, AddDeleteBodiesAfterSetUp) {
  ASSERT_TRUE(renderer_geometry_ptr_->SetUp());
  ASSERT_TRUE(renderer_geometry_ptr_->AddBody(triangle_body_ptr_));
  ASSERT_TRUE(renderer_geometry_ptr_->AddBody(schauma_body_ptr_));
  ASSERT_TRUE(renderer_geometry_ptr_->DeleteBody("triangle"));
  ASSERT_TRUE(
      TestRendererDataSchauma(renderer_geometry_ptr_->render_data_bodies()[0]));
  ASSERT_TRUE(renderer_geometry_ptr_->SetUp());
  ASSERT_TRUE(
      TestRendererDataSchauma(renderer_geometry_ptr_->render_data_bodies()[0]));
}

TEST_F(RendererGeometryTest, ClearBodiesBeforeSetUp) {
  renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
  renderer_geometry_ptr_->AddBody(schauma_body_ptr_);
  renderer_geometry_ptr_->ClearBodies();
  ASSERT_TRUE(renderer_geometry_ptr_->render_data_bodies().empty());
}

TEST_F(RendererGeometryTest, ClearBodiesAfterSetUp) {
  renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
  renderer_geometry_ptr_->AddBody(schauma_body_ptr_);
  renderer_geometry_ptr_->SetUp();
  renderer_geometry_ptr_->ClearBodies();
  ASSERT_TRUE(renderer_geometry_ptr_->render_data_bodies().empty());
}

TEST_F(RendererGeometryTest, AddTwiceAndDeleteNonExistent) {
  renderer_geometry_ptr_->AddBody(triangle_body_ptr_);
  ASSERT_FALSE(renderer_geometry_ptr_->AddBody(triangle_body_ptr_));
  ASSERT_FALSE(renderer_geometry_ptr_->DeleteBody("non_exitent"));
}

TEST_F(RendererGeometryTest, TestWithoutSetUpBody) {
  renderer_geometry_ptr_->AddBody(TriangleBodyPtrNoSetUp());
  ASSERT_FALSE(renderer_geometry_ptr_->SetUp());
}

TEST_F(RendererGeometryTest, TestWithoutSetUp) {
  ASSERT_FALSE(renderer_geometry_ptr_->MakeContextCurrent());
  ASSERT_FALSE(renderer_geometry_ptr_->DetachContext());
}

TEST_F(RendererGeometryTest, MakeContextCurrent) {
  renderer_geometry_ptr_->SetUp();
  ASSERT_TRUE(renderer_geometry_ptr_->MakeContextCurrent());
  ASSERT_TRUE(renderer_geometry_ptr_->DetachContext());
}
