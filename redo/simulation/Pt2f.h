#pragma once

struct Pt2f
{
  float x;
  float y;
  float dist() const;
  float dist(const Pt2f&) const;
  float dist2() const;
  float dist2(const Pt2f&) const;
  float cross(const Pt2f& that) const;
  float dot(const Pt2f& that) const;

  /**
   * Minimimum distance to segment [v,w]
   */
  Pt2f segmentProjection(const Pt2f& v, const Pt2f& w) const;
};

inline Pt2f
operator+(const Pt2f& a, const Pt2f& b) { return { a.x + b.x, a.y + b.y }; }

inline Pt2f
operator-(const Pt2f& a, const Pt2f& b) { return { a.x - b.x, a.y - b.y }; }

inline Pt2f
operator-(const Pt2f& a) { return { -a.x, -a.y }; }

inline Pt2f
operator*(const Pt2f& a, float b) { return { a.x * b, a.y * b }; }

inline Pt2f
operator*(float a, const Pt2f& b) { return b * a; }

inline Pt2f
operator/(const Pt2f& a, float b) { return { a.x / b, a.y / b }; }

inline Pt2f&
operator+=(Pt2f& a, const Pt2f& b)
{
  a.x += b.x;
  a.y += b.y;
  return a;
};