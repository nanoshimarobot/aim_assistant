#pragma once
#include "vector2.hpp"
#include <array>
#include <cmath>
#include <iostream>

using namespace abu2023::cpp_general;

namespace abu2023 {
class field_info {
public:
  const double UNIT = 0.001f;               //[mm]->[m]
  const double MAX_FIELD_DIST = 11.9;       //[m]
  const double CENTRE = MAX_FIELD_DIST / 2; // 中心
  // ポール
  const double TYPE1_D = 100 * UNIT;  // Type1ポールの直径
  const double TYPE2_D = 100 * UNIT;  // Type2ポールの直径
  const double TYPE3_D = 150 * UNIT;  // Type3ポールの直径
  const double TYPE1_H = 800 * UNIT;  // Type1ポールの高さ
  const double TYPE2_H = 800 * UNIT;  // Type2ポールの高さ
  const double TYPE3_H = 1500 * UNIT; // Type3ポールの高さ
  const std::array<std::tuple<Vector2, int>, 11> POLE{{
      // 形式 {x,y,type}
      // 赤ゾーン type1
      {Vector2(2.75, 2.75), 1},
      {Vector2(CENTRE, 2.75), 1},
      {Vector2(9.15, 2.75), 1},
      // 青ゾーン type1
      {Vector2(2.75, 9.15), 1},
      {Vector2(CENTRE, 9.15), 1},
      {Vector2(9.15, 9.15), 1},
      // type2
      {Vector2(4.65, 4.65), 2},
      {Vector2(7.25, 4.65), 2},
      {Vector2(4.65, 7.25), 2},
      {Vector2(7.25, 7.25), 2},
      // type3
      {Vector2(CENTRE, CENTRE), 3},
  }};
};
} // namespace abu2023