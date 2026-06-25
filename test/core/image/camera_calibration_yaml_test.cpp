// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_calibration_yaml.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace
{

using bagwiz::core::image::parse_camera_calibration_yaml;

// A complete, well-formed camera_calibration YAML in the canonical form.
constexpr const char * kValidYaml = R"(image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.01, -0.02, 0.003, 0.004, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
)";

class CameraCalibrationYamlTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_camera_calibration_yaml_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path write_yaml(const std::string & contents)
  {
    const auto path = tmp_dir_ / "calib.yaml";
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(CameraCalibrationYamlTest, ParsesCanonicalFile)
{
  const auto path = write_yaml(kValidYaml);
  const auto result = parse_camera_calibration_yaml(path);
  ASSERT_TRUE(result.ok()) << result.error;

  const auto & c = *result.calibration;
  EXPECT_EQ(c.width, 640U);
  EXPECT_EQ(c.height, 480U);
  EXPECT_EQ(c.distortion_model, "plumb_bob");

  ASSERT_EQ(c.d.size(), 5U);
  EXPECT_DOUBLE_EQ(c.d[0], 0.01);
  EXPECT_DOUBLE_EQ(c.d[4], 0.0);

  EXPECT_DOUBLE_EQ(c.k[0], 500.0);
  EXPECT_DOUBLE_EQ(c.k[2], 320.0);
  EXPECT_DOUBLE_EQ(c.k[5], 240.0);
  EXPECT_DOUBLE_EQ(c.k[8], 1.0);

  EXPECT_DOUBLE_EQ(c.r[0], 1.0);
  EXPECT_DOUBLE_EQ(c.r[4], 1.0);
  EXPECT_DOUBLE_EQ(c.r[8], 1.0);

  EXPECT_DOUBLE_EQ(c.p[0], 500.0);
  EXPECT_DOUBLE_EQ(c.p[6], 240.0);
  EXPECT_DOUBLE_EQ(c.p[10], 1.0);
}

TEST_F(CameraCalibrationYamlTest, RejectsMissingImageDimensions)
{
  std::string yaml = kValidYaml;
  // Drop the image_width line.
  const auto pos = yaml.find("image_width: 640\n");
  ASSERT_NE(pos, std::string::npos);
  yaml.erase(pos, std::string("image_width: 640\n").size());

  const auto result = parse_camera_calibration_yaml(write_yaml(yaml));
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("image_width"), std::string::npos);
}

TEST_F(CameraCalibrationYamlTest, RejectsWrongCameraMatrixSize)
{
  // 8 values instead of 9.
  const char * yaml = R"(image_width: 640
image_height: 480
camera_matrix:
  rows: 2
  cols: 4
  data: [1, 2, 3, 4, 5, 6, 7, 8]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0, 0, 0, 0, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
)";
  const auto result = parse_camera_calibration_yaml(write_yaml(yaml));
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("camera_matrix"), std::string::npos);
}

TEST_F(CameraCalibrationYamlTest, RejectsEmptyDistortionCoefficients)
{
  const char * yaml = R"(image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 0
  cols: 0
  data: []
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
)";
  const auto result = parse_camera_calibration_yaml(write_yaml(yaml));
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("distortion_coefficients"), std::string::npos);
}

TEST_F(CameraCalibrationYamlTest, RejectsMissingFile)
{
  const auto result = parse_camera_calibration_yaml(tmp_dir_ / "does_not_exist.yaml");
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
