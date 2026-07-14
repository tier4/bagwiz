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

using bagwiz::core::image::emit_camera_calibration_yaml;
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

// --- camera_name -----------------------------------------------------------

TEST_F(CameraCalibrationYamlTest, ParsesCameraName)
{
  const auto result = parse_camera_calibration_yaml(write_yaml(kValidYaml));
  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_TRUE(result.calibration->camera_name.has_value());
  EXPECT_EQ(*result.calibration->camera_name, "narrow_stereo");
}

TEST_F(CameraCalibrationYamlTest, LeavesCameraNameUnsetWhenFileOmitsIt)
{
  const std::string key = "camera_name: narrow_stereo\n";
  std::string yaml = kValidYaml;
  const auto at = yaml.find(key);
  ASSERT_NE(at, std::string::npos);
  yaml.erase(at, key.size());

  const auto result = parse_camera_calibration_yaml(write_yaml(yaml));
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_FALSE(result.calibration->camera_name.has_value());
}

// --- emit ------------------------------------------------------------------

TEST_F(CameraCalibrationYamlTest, EmitIsAFixedPointOfParse)
{
  const auto original = parse_camera_calibration_yaml(write_yaml(kValidYaml));
  ASSERT_TRUE(original.ok()) << original.error;

  const std::string emitted = emit_camera_calibration_yaml(*original.calibration);
  const auto reparsed = parse_camera_calibration_yaml(write_yaml(emitted));
  ASSERT_TRUE(reparsed.ok()) << "emitted YAML did not parse back:\n"
                             << emitted << "\nerror: " << reparsed.error;

  const auto & a = *original.calibration;
  const auto & b = *reparsed.calibration;
  EXPECT_EQ(b.width, a.width);
  EXPECT_EQ(b.height, a.height);
  EXPECT_EQ(b.distortion_model, a.distortion_model);
  EXPECT_EQ(b.camera_name, a.camera_name);
  EXPECT_EQ(b.d, a.d);
  EXPECT_EQ(b.k, a.k);
  EXPECT_EQ(b.r, a.r);
  EXPECT_EQ(b.p, a.p);
}

// The whole reason camera_name is stored at all: a recompute-p round-trip must
// not silently delete a key the file's author set.
TEST_F(CameraCalibrationYamlTest, EmitPreservesCameraName)
{
  const auto parsed = parse_camera_calibration_yaml(write_yaml(kValidYaml));
  ASSERT_TRUE(parsed.ok()) << parsed.error;

  const std::string emitted = emit_camera_calibration_yaml(*parsed.calibration);

  EXPECT_NE(emitted.find("camera_name"), std::string::npos) << emitted;
  EXPECT_NE(emitted.find("narrow_stereo"), std::string::npos) << emitted;
}

TEST_F(CameraCalibrationYamlTest, EmitOmitsCameraNameWhenUnset)
{
  auto parsed = parse_camera_calibration_yaml(write_yaml(kValidYaml));
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  parsed.calibration->camera_name.reset();

  const std::string emitted = emit_camera_calibration_yaml(*parsed.calibration);

  EXPECT_EQ(emitted.find("camera_name"), std::string::npos) << emitted;
  // Still a complete, parseable document without it.
  EXPECT_TRUE(parse_camera_calibration_yaml(write_yaml(emitted)).ok());
}

// Values carrying ~12 significant digits (real distortion coefficients do) must
// survive the emitter's precision setting intact.
TEST_F(CameraCalibrationYamlTest, EmitRoundTripsHighPrecisionCoefficients)
{
  auto parsed = parse_camera_calibration_yaml(write_yaml(kValidYaml));
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  parsed.calibration->d = {
    -0.143768742681, 0.031336024404, -0.001296524890, -0.001500067534, -0.003719333094};

  const std::string emitted = emit_camera_calibration_yaml(*parsed.calibration);
  const auto reparsed = parse_camera_calibration_yaml(write_yaml(emitted));

  ASSERT_TRUE(reparsed.ok()) << reparsed.error;
  EXPECT_EQ(reparsed.calibration->d, parsed.calibration->d);
}

}  // namespace
