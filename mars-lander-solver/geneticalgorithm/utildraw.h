#pragma once

#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <raylib.h>
#include <vector>

void
drawRocket(float x, float y, int angle, float power, const Color& color, float scale);
// inline void drawRocketSimple(float x,
//                              float y,
//                              int angle,
//                              float power,
//                              const Color& color,
//                              float scale);

void
_drawRocket(float x,
            float y,
            int angle,
            float power,
            const Color& color,
            float scale,
            const std::vector<glm::vec<3, float>>& vec,
            const std::vector<size_t>& order);

void
drawRocketSimple(float x,
                 float y,
                 int angle,
                 float power,
                 const Color& color,
                 float scale);

template <class T>
T
bounceMinMax(const T& val, const T& minV, const T& maxV)
{
  return val - 2 * std::min(val - minV, 0) - 2 * std::max(val - maxV, 0);
}