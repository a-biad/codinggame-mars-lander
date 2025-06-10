#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <raylib.h>
#include <vector>

inline void
drawRocket(float x, float y, int angle, float power, const Color& color, float scale);
inline void drawRocketSimple(float x,
                      float y,
                      int angle,
                      float power,
                      const Color& color,
                      float scale);

inline void _drawRocket(float x,
                 float y,
                 int angle,
                 float power,
                 const Color& color,
                 float scale,
                 const std::vector<glm::vec<3, float>>& vec,
                 const std::vector<size_t>& order)
{
  float an{ float(angle) / 180.f * 3.14159265f };

  glm::mat<3, 3, float> scaleM{ scale, 0.f,   0.f, //
                                0.f,   scale, 0.f, //
                                0.f,   0.f,   1.f };
  glm::mat<3, 3, float> rotateM{ cos(an),  sin(an), 0.f, //
                                 -sin(an), cos(an), 0.f, //
                                 0.f,      0.f,     1.f };

  glm::mat<3, 3, float> transM{ 1.f, 0.f, 0.f, //
                                0.f, 1.f, 0.f, //
                                x,   y,   1.f };

  size_t op{ *(order.end() - 1) };
  for(auto o : order) {
    // auto p1 = transM * rotateM * scaleM * vec[op];
    // auto p2 = transM * rotateM * scaleM * vec[o];
    glm::vec<3, float> p1 = transM * scaleM * rotateM * vec[op];
    glm::vec<3, float> p2 = transM * scaleM * rotateM * vec[o];

    // static int numCall{ 0 };
    // numCall++;
    // if(numCall == 100) {
    //   exit(0);
    // }
    // std::cerr << x << " " << y << " " << p1.x << " " << p1.y << " " << p2.x << " " <<
    // p2.y
    //           << "\n";

    DrawLine(p1.x, p1.y, p2.x, p2.y, color);
    op = o;
  }
}

inline void drawRocket(float x, float y, int angle, float power, const Color& color, float scale)
{
  static const std::vector<glm::vec<3, float>> vec{
    { 0.f, 1.2f, 1.f },
    { -1.f, -1.f, 1.f },
    { +1.f, -1.f, 1.f },
    { -.25f, -1.f, 1.f },
    { -.25f, -1.f - power / 2.f, 1.f },
    { +.25f, -1.f - power / 2.f, 1.f },
    { +.25f, -1.f, 1.f },
  };

  static const std::vector<size_t> order{ 1, 3, 4, 5, 6, 2, 0 };
  _drawRocket(x, y, angle, power, color, scale, vec, order);
}

inline void
drawRocketSimple(float x, float y, int angle, float power, const Color& color, float scale)
{
  static const std::vector<glm::vec<3, float>> vec{
    { 0.f, 0.f, 1.f },
    { 0.f, -0.1f - 0.2f * power, 1.f },
  };
  static const std::vector<size_t> order{ 0, 1 };
  _drawRocket(x, y, angle, power, color, scale, vec, order);
}

template <class T>
T bounceMinMax(const T& val, const T& minV, const T& maxV)
{
  return val - 2 * std::min(val - minV, 0) - 2 * std::max(val - maxV, 0);
}
