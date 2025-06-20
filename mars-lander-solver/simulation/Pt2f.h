#pragma once

#include "Float.h"

struct Pt2f
{
    Float x;
    Float y;
    Float dist() const;
    Float dist(const Pt2f&) const;
    Float dist2() const;
    Float dist2(const Pt2f&) const;
    Float cross(const Pt2f& that) const;
    Float dot(const Pt2f& that) const;

    /**
     * Minimimum distance to segment [v,w]
     */
    Pt2f segmentProjection(const Pt2f& v, const Pt2f& w) const;
};

inline Pt2f
operator+(const Pt2f& a, const Pt2f& b) { return {a.x + b.x, a.y + b.y}; }

inline Pt2f
operator-(const Pt2f& a, const Pt2f& b) { return {a.x - b.x, a.y - b.y}; }

inline Pt2f
operator-(const Pt2f& a) { return {-a.x, -a.y}; }

inline Pt2f
operator*(const Pt2f& a, Float b) { return {a.x * b, a.y * b}; }

inline Pt2f
operator*(Float a, const Pt2f& b) { return b * a; }

inline Pt2f
operator/(const Pt2f& a, Float b) { return {a.x / b, a.y / b}; }

inline Pt2f&
operator+=(Pt2f& a, const Pt2f& b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
};
