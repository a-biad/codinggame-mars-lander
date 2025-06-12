#include <iostream>
#include <cmath>


#include "Land.h"
#include "Constants.h"


using namespace simulation;


Land::Land() : pts_{ Pt2f{ 0._f, 0._f }, //
                     Pt2f{ BOUNDARIES.x, 0._f }, //
                     Pt2f{ BOUNDARIES }, //
                     Pt2f{ 0._f, BOUNDARIES.y }, //
                     Pt2f{ 0._f, 0._f } },
               idxLTarget_(0)
{
  //
}

void
Land::fetchFromIstream(std::istream& is)
{
  auto& land{ this->pts_ };
  land.clear();

  size_t surface_n; // the number of points used to draw the surface of Mars.
  is >> surface_n;
  is.ignore();
  land.resize(surface_n + 3);

  for(size_t i = 0; i < surface_n; i++) {
    is >> land[i].x >> land[i].y;
    is.ignore();
  }
  land[surface_n] = { BOUNDARIES.x, BOUNDARIES.y };
  land[surface_n + 1] = { 0, BOUNDARIES.y };
  land[surface_n + 2] = land[0];

  findTarget();
}

void
Land::findTarget()
{
  const auto& land{ pts_ };
  for(size_t i = 1; i < land.size(); i++) {
    if(land[i].x - land[i - 1].x >= 1000
       && land[i - 1].y == land[i].y) {
      idxLTarget_ = i - 1;
      return;
    }
  }
  throw std::runtime_error("Land target not found");
}

Collision
Land::checkCollision(const Pt2f& p1, const Pt2f& p2) const
{
  const auto& land = this->pts_;
  Collision ret;
  ret.hasCollided = false;

  const Pt2f r{ p2 - p1 };
  for(size_t i{ 1 }; i < land.size(); i++) {
    const Pt2f& l1{ land[i - 1] };
    const Pt2f& l2{ land[i] };
    const Pt2f s{ l2 - l1 };
    Float denum{ r.cross(s) };
    Float uNum{ (l1 - p1).cross(r) };
    Float tNum{ (l1 - p1).cross(s) };
    if(denum != 0) {
      Float t, u;
      if(0 <= (t = tNum / denum) && t <= 1 //
         && 0 <= (u = uNum / denum) && u <= 1) {
        ret.hasCollided = true;
        ret.intersection = p1 + t * r;
        ret.idxLIntersection = i - 1;
        return ret;
      }
      else {
        // Not parallel and no intersection
      }
    }
    else {
      // parallel
    }
  }
  return ret;
}

float
Land::projectedDistance(const Collision& c) const
{
  if(c.idxLIntersection.value() == this->idxLTarget_)
    return 0.f;

  const auto& land{ pts_ };
  float ret{ 0.f };

  int dir; // Direction of the target
  size_t idxLi;
  size_t idxLiend;
  if(c.idxLIntersection.value() < this->idxLTarget_) {
    dir = 1;
    idxLi = c.idxLIntersection.value() + 1;
    idxLiend = this->idxLTarget_;
    ret += c.intersection->dist(land[c.idxLIntersection.value() + 1]);
  }
  else {
    dir = -1;
    idxLi = c.idxLIntersection.value() - 1;
    idxLiend = this->idxLTarget_;
    ret += c.intersection->dist(land[c.idxLIntersection.value()]);
  }
  for(; idxLi != idxLiend; idxLi += dir) { ret += land[idxLi].dist(land[idxLi + 1]); }

  return ret;
}


float
Land::projectedDistance(const Pt2f& p) const
{
  const auto& land = this->pts_;
  float ans{ std::numeric_limits<float>::infinity() };

  Collision coll;
  coll.hasCollided = true;

  for(size_t idxL{ 0 }; idxL < land.size() - 1; idxL++) {
    const Pt2f& l1{ land[idxL] };
    const Pt2f& l2{ land[idxL + 1] };
    const Pt2f proj{ p.segmentProjection(l1, l2) };
    const Float d{ p.dist2(proj) };
    if(d < ans) {
      ans = d;
      coll.idxLIntersection = idxL;
      coll.intersection = proj;
    }
  }

  return std::sqrt(ans) + projectedDistance(coll);
}