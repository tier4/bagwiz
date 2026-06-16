// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/color/color_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace bagwiz::core::color
{

namespace
{
// Lookup table for the Google Turbo colormap.
constexpr std::array<Rgb, 256> kTurboTable = {
  Rgb{48, 18, 59},    Rgb{50, 21, 67},    Rgb{51, 24, 74},   Rgb{52, 27, 81},   Rgb{53, 30, 88},
  Rgb{54, 33, 95},    Rgb{55, 36, 102},   Rgb{56, 39, 109},  Rgb{57, 42, 115},  Rgb{58, 45, 121},
  Rgb{59, 47, 128},   Rgb{60, 50, 134},   Rgb{61, 53, 139},  Rgb{62, 56, 145},  Rgb{63, 59, 151},
  Rgb{63, 62, 156},   Rgb{64, 64, 162},   Rgb{65, 67, 167},  Rgb{65, 70, 172},  Rgb{66, 73, 177},
  Rgb{66, 75, 181},   Rgb{67, 78, 186},   Rgb{68, 81, 191},  Rgb{68, 84, 195},  Rgb{68, 86, 199},
  Rgb{69, 89, 203},   Rgb{69, 92, 207},   Rgb{69, 94, 211},  Rgb{70, 97, 214},  Rgb{70, 100, 218},
  Rgb{70, 102, 221},  Rgb{70, 105, 224},  Rgb{70, 107, 227}, Rgb{71, 110, 230}, Rgb{71, 113, 233},
  Rgb{71, 115, 235},  Rgb{71, 118, 238},  Rgb{71, 120, 240}, Rgb{71, 123, 242}, Rgb{70, 125, 244},
  Rgb{70, 128, 246},  Rgb{70, 130, 248},  Rgb{70, 133, 250}, Rgb{70, 135, 251}, Rgb{69, 138, 252},
  Rgb{69, 140, 253},  Rgb{68, 143, 254},  Rgb{67, 145, 254}, Rgb{66, 148, 255}, Rgb{65, 150, 255},
  Rgb{64, 153, 255},  Rgb{62, 155, 254},  Rgb{61, 158, 254}, Rgb{59, 160, 253}, Rgb{58, 163, 252},
  Rgb{56, 165, 251},  Rgb{55, 168, 250},  Rgb{53, 171, 248}, Rgb{51, 173, 247}, Rgb{49, 175, 245},
  Rgb{47, 178, 244},  Rgb{46, 180, 242},  Rgb{44, 183, 240}, Rgb{42, 185, 238}, Rgb{40, 188, 235},
  Rgb{39, 190, 233},  Rgb{37, 192, 231},  Rgb{35, 195, 228}, Rgb{34, 197, 226}, Rgb{32, 199, 223},
  Rgb{31, 201, 221},  Rgb{30, 203, 218},  Rgb{28, 205, 216}, Rgb{27, 208, 213}, Rgb{26, 210, 210},
  Rgb{26, 212, 208},  Rgb{25, 213, 205},  Rgb{24, 215, 202}, Rgb{24, 217, 200}, Rgb{24, 219, 197},
  Rgb{24, 221, 194},  Rgb{24, 222, 192},  Rgb{24, 224, 189}, Rgb{25, 226, 187}, Rgb{25, 227, 185},
  Rgb{26, 228, 182},  Rgb{28, 230, 180},  Rgb{29, 231, 178}, Rgb{31, 233, 175}, Rgb{32, 234, 172},
  Rgb{34, 235, 170},  Rgb{37, 236, 167},  Rgb{39, 238, 164}, Rgb{42, 239, 161}, Rgb{44, 240, 158},
  Rgb{47, 241, 155},  Rgb{50, 242, 152},  Rgb{53, 243, 148}, Rgb{56, 244, 145}, Rgb{60, 245, 142},
  Rgb{63, 246, 138},  Rgb{67, 247, 135},  Rgb{70, 248, 132}, Rgb{74, 248, 128}, Rgb{78, 249, 125},
  Rgb{82, 250, 122},  Rgb{85, 250, 118},  Rgb{89, 251, 115}, Rgb{93, 252, 111}, Rgb{97, 252, 108},
  Rgb{101, 253, 105}, Rgb{105, 253, 102}, Rgb{109, 254, 98}, Rgb{113, 254, 95}, Rgb{117, 254, 92},
  Rgb{121, 254, 89},  Rgb{125, 255, 86},  Rgb{128, 255, 83}, Rgb{132, 255, 81}, Rgb{136, 255, 78},
  Rgb{139, 255, 75},  Rgb{143, 255, 73},  Rgb{146, 255, 71}, Rgb{150, 254, 68}, Rgb{153, 254, 66},
  Rgb{156, 254, 64},  Rgb{159, 253, 63},  Rgb{161, 253, 61}, Rgb{164, 252, 60}, Rgb{167, 252, 58},
  Rgb{169, 251, 57},  Rgb{172, 251, 56},  Rgb{175, 250, 55}, Rgb{177, 249, 54}, Rgb{180, 248, 54},
  Rgb{183, 247, 53},  Rgb{185, 246, 53},  Rgb{188, 245, 52}, Rgb{190, 244, 52}, Rgb{193, 243, 52},
  Rgb{195, 241, 52},  Rgb{198, 240, 52},  Rgb{200, 239, 52}, Rgb{203, 237, 52}, Rgb{205, 236, 52},
  Rgb{208, 234, 52},  Rgb{210, 233, 53},  Rgb{212, 231, 53}, Rgb{215, 229, 53}, Rgb{217, 228, 54},
  Rgb{219, 226, 54},  Rgb{221, 224, 55},  Rgb{223, 223, 55}, Rgb{225, 221, 55}, Rgb{227, 219, 56},
  Rgb{229, 217, 56},  Rgb{231, 215, 57},  Rgb{233, 213, 57}, Rgb{235, 211, 57}, Rgb{236, 209, 58},
  Rgb{238, 207, 58},  Rgb{239, 205, 58},  Rgb{241, 203, 58}, Rgb{242, 201, 58}, Rgb{244, 199, 58},
  Rgb{245, 197, 58},  Rgb{246, 195, 58},  Rgb{247, 193, 58}, Rgb{248, 190, 57}, Rgb{249, 188, 57},
  Rgb{250, 186, 57},  Rgb{251, 184, 56},  Rgb{251, 182, 55}, Rgb{252, 179, 54}, Rgb{252, 177, 54},
  Rgb{253, 174, 53},  Rgb{253, 172, 52},  Rgb{254, 169, 51}, Rgb{254, 167, 50}, Rgb{254, 164, 49},
  Rgb{254, 161, 48},  Rgb{254, 158, 47},  Rgb{254, 155, 45}, Rgb{254, 153, 44}, Rgb{254, 150, 43},
  Rgb{254, 147, 42},  Rgb{254, 144, 41},  Rgb{253, 141, 39}, Rgb{253, 138, 38}, Rgb{252, 135, 37},
  Rgb{252, 132, 35},  Rgb{251, 129, 34},  Rgb{251, 126, 33}, Rgb{250, 123, 31}, Rgb{249, 120, 30},
  Rgb{249, 117, 29},  Rgb{248, 114, 28},  Rgb{247, 111, 26}, Rgb{246, 108, 25}, Rgb{245, 105, 24},
  Rgb{244, 102, 23},  Rgb{243, 99, 21},   Rgb{242, 96, 20},  Rgb{241, 93, 19},  Rgb{240, 91, 18},
  Rgb{239, 88, 17},   Rgb{237, 85, 16},   Rgb{236, 83, 15},  Rgb{235, 80, 14},  Rgb{234, 78, 13},
  Rgb{232, 75, 12},   Rgb{231, 73, 12},   Rgb{229, 71, 11},  Rgb{228, 69, 10},  Rgb{226, 67, 10},
  Rgb{225, 65, 9},    Rgb{223, 63, 8},    Rgb{221, 61, 8},   Rgb{220, 59, 7},   Rgb{218, 57, 7},
  Rgb{216, 55, 6},    Rgb{214, 53, 6},    Rgb{212, 51, 5},   Rgb{210, 49, 5},   Rgb{208, 47, 5},
  Rgb{206, 45, 4},    Rgb{204, 43, 4},    Rgb{202, 42, 4},   Rgb{200, 40, 3},   Rgb{197, 38, 3},
  Rgb{195, 37, 3},    Rgb{193, 35, 2},    Rgb{190, 33, 2},   Rgb{188, 32, 2},   Rgb{185, 30, 2},
  Rgb{183, 29, 2},    Rgb{180, 27, 1},    Rgb{178, 26, 1},   Rgb{175, 24, 1},   Rgb{172, 23, 1},
  Rgb{169, 22, 1},    Rgb{167, 20, 1},    Rgb{164, 19, 1},   Rgb{161, 18, 1},   Rgb{158, 16, 1},
  Rgb{155, 15, 1},    Rgb{152, 14, 1},    Rgb{149, 13, 1},   Rgb{146, 11, 1},   Rgb{142, 10, 1},
  Rgb{139, 9, 2},     Rgb{136, 8, 2},     Rgb{133, 7, 2},    Rgb{129, 6, 2},    Rgb{126, 5, 2},
  Rgb{122, 4, 3},
};
// Lookup table for the matplotlib viridis colormap.
constexpr std::array<Rgb, 256> kViridisTable = {
  Rgb{68, 1, 84},    Rgb{68, 2, 86},    Rgb{69, 4, 87},    Rgb{69, 5, 89},    Rgb{70, 7, 90},
  Rgb{70, 8, 92},    Rgb{70, 10, 93},   Rgb{70, 11, 94},   Rgb{71, 13, 96},   Rgb{71, 14, 97},
  Rgb{71, 16, 99},   Rgb{71, 17, 100},  Rgb{71, 19, 101},  Rgb{72, 20, 103},  Rgb{72, 22, 104},
  Rgb{72, 23, 105},  Rgb{72, 24, 106},  Rgb{72, 26, 108},  Rgb{72, 27, 109},  Rgb{72, 28, 110},
  Rgb{72, 29, 111},  Rgb{72, 31, 112},  Rgb{72, 32, 113},  Rgb{72, 33, 115},  Rgb{72, 35, 116},
  Rgb{72, 36, 117},  Rgb{72, 37, 118},  Rgb{72, 38, 119},  Rgb{72, 40, 120},  Rgb{72, 41, 121},
  Rgb{71, 42, 122},  Rgb{71, 44, 122},  Rgb{71, 45, 123},  Rgb{71, 46, 124},  Rgb{71, 47, 125},
  Rgb{70, 48, 126},  Rgb{70, 50, 126},  Rgb{70, 51, 127},  Rgb{70, 52, 128},  Rgb{69, 53, 129},
  Rgb{69, 55, 129},  Rgb{69, 56, 130},  Rgb{68, 57, 131},  Rgb{68, 58, 131},  Rgb{68, 59, 132},
  Rgb{67, 61, 132},  Rgb{67, 62, 133},  Rgb{66, 63, 133},  Rgb{66, 64, 134},  Rgb{66, 65, 134},
  Rgb{65, 66, 135},  Rgb{65, 68, 135},  Rgb{64, 69, 136},  Rgb{64, 70, 136},  Rgb{63, 71, 136},
  Rgb{63, 72, 137},  Rgb{62, 73, 137},  Rgb{62, 74, 137},  Rgb{62, 76, 138},  Rgb{61, 77, 138},
  Rgb{61, 78, 138},  Rgb{60, 79, 138},  Rgb{60, 80, 139},  Rgb{59, 81, 139},  Rgb{59, 82, 139},
  Rgb{58, 83, 139},  Rgb{58, 84, 140},  Rgb{57, 85, 140},  Rgb{57, 86, 140},  Rgb{56, 88, 140},
  Rgb{56, 89, 140},  Rgb{55, 90, 140},  Rgb{55, 91, 141},  Rgb{54, 92, 141},  Rgb{54, 93, 141},
  Rgb{53, 94, 141},  Rgb{53, 95, 141},  Rgb{52, 96, 141},  Rgb{52, 97, 141},  Rgb{51, 98, 141},
  Rgb{51, 99, 141},  Rgb{50, 100, 142}, Rgb{50, 101, 142}, Rgb{49, 102, 142}, Rgb{49, 103, 142},
  Rgb{49, 104, 142}, Rgb{48, 105, 142}, Rgb{48, 106, 142}, Rgb{47, 107, 142}, Rgb{47, 108, 142},
  Rgb{46, 109, 142}, Rgb{46, 110, 142}, Rgb{46, 111, 142}, Rgb{45, 112, 142}, Rgb{45, 113, 142},
  Rgb{44, 113, 142}, Rgb{44, 114, 142}, Rgb{44, 115, 142}, Rgb{43, 116, 142}, Rgb{43, 117, 142},
  Rgb{42, 118, 142}, Rgb{42, 119, 142}, Rgb{42, 120, 142}, Rgb{41, 121, 142}, Rgb{41, 122, 142},
  Rgb{41, 123, 142}, Rgb{40, 124, 142}, Rgb{40, 125, 142}, Rgb{39, 126, 142}, Rgb{39, 127, 142},
  Rgb{39, 128, 142}, Rgb{38, 129, 142}, Rgb{38, 130, 142}, Rgb{38, 130, 142}, Rgb{37, 131, 142},
  Rgb{37, 132, 142}, Rgb{37, 133, 142}, Rgb{36, 134, 142}, Rgb{36, 135, 142}, Rgb{35, 136, 142},
  Rgb{35, 137, 142}, Rgb{35, 138, 141}, Rgb{34, 139, 141}, Rgb{34, 140, 141}, Rgb{34, 141, 141},
  Rgb{33, 142, 141}, Rgb{33, 143, 141}, Rgb{33, 144, 141}, Rgb{33, 145, 140}, Rgb{32, 146, 140},
  Rgb{32, 146, 140}, Rgb{32, 147, 140}, Rgb{31, 148, 140}, Rgb{31, 149, 139}, Rgb{31, 150, 139},
  Rgb{31, 151, 139}, Rgb{31, 152, 139}, Rgb{31, 153, 138}, Rgb{31, 154, 138}, Rgb{30, 155, 138},
  Rgb{30, 156, 137}, Rgb{30, 157, 137}, Rgb{31, 158, 137}, Rgb{31, 159, 136}, Rgb{31, 160, 136},
  Rgb{31, 161, 136}, Rgb{31, 161, 135}, Rgb{31, 162, 135}, Rgb{32, 163, 134}, Rgb{32, 164, 134},
  Rgb{33, 165, 133}, Rgb{33, 166, 133}, Rgb{34, 167, 133}, Rgb{34, 168, 132}, Rgb{35, 169, 131},
  Rgb{36, 170, 131}, Rgb{37, 171, 130}, Rgb{37, 172, 130}, Rgb{38, 173, 129}, Rgb{39, 173, 129},
  Rgb{40, 174, 128}, Rgb{41, 175, 127}, Rgb{42, 176, 127}, Rgb{44, 177, 126}, Rgb{45, 178, 125},
  Rgb{46, 179, 124}, Rgb{47, 180, 124}, Rgb{49, 181, 123}, Rgb{50, 182, 122}, Rgb{52, 182, 121},
  Rgb{53, 183, 121}, Rgb{55, 184, 120}, Rgb{56, 185, 119}, Rgb{58, 186, 118}, Rgb{59, 187, 117},
  Rgb{61, 188, 116}, Rgb{63, 188, 115}, Rgb{64, 189, 114}, Rgb{66, 190, 113}, Rgb{68, 191, 112},
  Rgb{70, 192, 111}, Rgb{72, 193, 110}, Rgb{74, 193, 109}, Rgb{76, 194, 108}, Rgb{78, 195, 107},
  Rgb{80, 196, 106}, Rgb{82, 197, 105}, Rgb{84, 197, 104}, Rgb{86, 198, 103}, Rgb{88, 199, 101},
  Rgb{90, 200, 100}, Rgb{92, 200, 99},  Rgb{94, 201, 98},  Rgb{96, 202, 96},  Rgb{99, 203, 95},
  Rgb{101, 203, 94}, Rgb{103, 204, 92}, Rgb{105, 205, 91}, Rgb{108, 205, 90}, Rgb{110, 206, 88},
  Rgb{112, 207, 87}, Rgb{115, 208, 86}, Rgb{117, 208, 84}, Rgb{119, 209, 83}, Rgb{122, 209, 81},
  Rgb{124, 210, 80}, Rgb{127, 211, 78}, Rgb{129, 211, 77}, Rgb{132, 212, 75}, Rgb{134, 213, 73},
  Rgb{137, 213, 72}, Rgb{139, 214, 70}, Rgb{142, 214, 69}, Rgb{144, 215, 67}, Rgb{147, 215, 65},
  Rgb{149, 216, 64}, Rgb{152, 216, 62}, Rgb{155, 217, 60}, Rgb{157, 217, 59}, Rgb{160, 218, 57},
  Rgb{162, 218, 55}, Rgb{165, 219, 54}, Rgb{168, 219, 52}, Rgb{170, 220, 50}, Rgb{173, 220, 48},
  Rgb{176, 221, 47}, Rgb{178, 221, 45}, Rgb{181, 222, 43}, Rgb{184, 222, 41}, Rgb{186, 222, 40},
  Rgb{189, 223, 38}, Rgb{192, 223, 37}, Rgb{194, 223, 35}, Rgb{197, 224, 33}, Rgb{200, 224, 32},
  Rgb{202, 225, 31}, Rgb{205, 225, 29}, Rgb{208, 225, 28}, Rgb{210, 226, 27}, Rgb{213, 226, 26},
  Rgb{216, 226, 25}, Rgb{218, 227, 25}, Rgb{221, 227, 24}, Rgb{223, 227, 24}, Rgb{226, 228, 24},
  Rgb{229, 228, 25}, Rgb{231, 228, 25}, Rgb{234, 229, 26}, Rgb{236, 229, 27}, Rgb{239, 229, 28},
  Rgb{241, 229, 29}, Rgb{244, 230, 30}, Rgb{246, 230, 32}, Rgb{248, 230, 33}, Rgb{251, 231, 35},
  Rgb{253, 231, 37},
};

Rgb jet_rgb(std::size_t index)
{
  const float t = static_cast<float>(index) / 255.0f;
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;

  if (t < 0.125f) {
    r = 0.0f;
    g = 0.0f;
    b = 0.5f + 4.0f * t;
  } else if (t < 0.375f) {
    r = 0.0f;
    g = 4.0f * t - 0.5f;
    b = 1.0f;
  } else if (t < 0.625f) {
    r = 4.0f * t - 1.5f;
    g = 1.0f;
    b = 2.5f - 4.0f * t;
  } else if (t < 0.875f) {
    r = 1.0f;
    g = 3.5f - 4.0f * t;
    b = 0.0f;
  } else {
    r = 4.5f - 4.0f * t;
    g = 0.0f;
    b = 0.0f;
  }

  const auto to_u8 = [](float v) {
    return static_cast<std::uint8_t>(std::clamp(v * 255.0f + 0.5f, 0.0f, 255.0f));
  };
  return Rgb{to_u8(r), to_u8(g), to_u8(b)};
}

Rgb hsv_to_rgb(float h, float s, float v)
{
  const float c = v * s;
  const float x = c * (1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f));
  const float m = v - c;

  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  switch (static_cast<int>(h * 6.0f) % 6) {
    case 0:
      r = c;
      g = x;
      break;
    case 1:
      r = x;
      g = c;
      break;
    case 2:
      g = c;
      b = x;
      break;
    case 3:
      g = x;
      b = c;
      break;
    case 4:
      r = x;
      b = c;
      break;
    case 5:
      r = c;
      b = x;
      break;
  }

  const auto to_u8 = [m](float component) {
    return static_cast<std::uint8_t>(std::clamp((component + m) * 255.0f + 0.5f, 0.0f, 255.0f));
  };
  return Rgb{to_u8(r), to_u8(g), to_u8(b)};
}
}  // namespace

ColorMap make_color_map(ColorMapName name)
{
  ColorMap map;

  switch (name) {
    case ColorMapName::kJet:
      for (std::size_t i = 0; i < map.table.size(); ++i) {
        map.table[i] = jet_rgb(i);
      }
      break;
    case ColorMapName::kTurbo:
      map.table = kTurboTable;
      break;
    case ColorMapName::kViridis:
      map.table = kViridisTable;
      break;
    case ColorMapName::kGrayscale:
      for (std::size_t i = 0; i < map.table.size(); ++i) {
        const auto v = static_cast<std::uint8_t>(i);
        map.table[i] = Rgb{v, v, v};
      }
      break;
    case ColorMapName::kRainbow:
      for (std::size_t i = 0; i < map.table.size(); ++i) {
        map.table[i] = hsv_to_rgb(static_cast<float>(i) / 255.0f, 1.0f, 1.0f);
      }
      break;
    default:
      break;
  }

  return map;
}

Rgb apply(const ColorMap & map, std::uint8_t index)
{
  return map.table[index];
}

std::uint8_t normalize(float value, float min, float max)
{
  if (max <= min) {
    return 0;
  }
  const float t = (value - min) / (max - min);
  return static_cast<std::uint8_t>(std::clamp(t * 255.0f, 0.0f, 255.0f));
}

}  // namespace bagwiz::core::color
