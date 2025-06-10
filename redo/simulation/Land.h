#pragma once

#include <vector>
#include <istream>

#include "Collision.h"
#include "Pt2f.h"


namespace simulation
{
  class Land
  {
    std::vector<Pt2f> pts_;
    size_t idxLTarget_;

  public:
    Land();

    // optionnal TODO: Add constructor Land(vector<Pt2f>)

    Land(std::istream& is) { fetchFromIstream(is); };

    void fetchFromIstream(std::istream&);

    void findTarget();

    std::vector<Pt2f>&
    pts() { return pts_; }

    Collision checkCollision(const Pt2f& p1, const Pt2f& p2) const;

    float projectedDistance(const Collision& c) const;

    float projectedDistance(const Pt2f& p) const;

    Pt2f
    targetL() const { return pts_[idxLTarget_]; };

    Pt2f
    targetR() const { return pts_[idxLTarget_ + 1]; };

    size_t
    idxLTarget() const { return idxLTarget_; };

    auto const&
    points() const { return pts_; }
  };
}