#pragma once

#include <vector>
#include "Vect3.h"

namespace DeWall
{
    struct Edge
    {
        short a;
        short b;
        short oppositePoint;
        Edge(const int _a, const int _b, const int _opposite = -1) : a(_a), b(_b), oppositePoint(_opposite) {}

        bool operator==(const Edge &) const;
        bool operator!=(const Edge &) const;
    };

    inline bool Edge::operator==(const Edge &_b) const { return a == _b.a && b == _b.b || a == _b.b && b == _b.a; }
    inline bool Edge::operator!=(const Edge &_b) const { return !(*this == _b); }

    struct Triangle
    {
		int a;
		int b;
		int c;

		Triangle(int _a, int _b, int _c) :a(_a), b(_b), c(_c) {}

		Edge AB() const { return Edge(a, b, c); }
		Edge BC() const { return Edge(b, c, a); }
		Edge CA() const { return Edge(c, a, b); }
    };

    typedef std::vector<Vect3<float>> PointSet;


    std::vector<Triangle> Triangulate(const PointSet &_vertices);
}
