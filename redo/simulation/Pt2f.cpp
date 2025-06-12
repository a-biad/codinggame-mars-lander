#include <cmath>

#include "Pt2f.h"



Float
Pt2f::dist() const { return std::sqrt(this->x * this->x + this->y * this->y); }


Float
Pt2f::dist(const Pt2f& that) const
{
  return Pt2f{ this->x - that.x, this->y - that.y }.dist();
}

Float
Pt2f::dist2() const { return this->x * this->x + this->y * this->y; }


Float
Pt2f::dist2(const Pt2f& that) const
{
  return Pt2f{ this->x - that.x, this->y - that.y }.dist2();
}

Float
Pt2f::cross(const Pt2f& that) const { return this->x * that.y - this->y * that.x; }

Float
Pt2f::dot(const Pt2f& that) const { return this->x * that.x + this->y * that.y; }

Pt2f
Pt2f::segmentProjection(const Pt2f& v, const Pt2f& w) const
{
  // https://stackoverflow.com/a/1501725

  const Pt2f& p{ *this };
  // Return minimum distance between line segment vw and point p
  const Float l2{ v.dist2(w) };
  if(l2 == 0.0)
    return v; // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  const Float t = std::max(0._f, std::min(1._f, (p - v).dot(w - v) / l2));
  const Pt2f projection = v + t * (w - v); // Projection falls on the segment
  return projection;
}