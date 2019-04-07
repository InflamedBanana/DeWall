#pragma once

#include <vector>
#include "Vect3.h"

namespace Delaunay
{
    struct Edge
    {
        short a;
        short b;
        short oppositePoint;
        Edge(const int _a, const int _b, const int _opposite = -1) : a(_a), b(_b), oppositePoint(_opposite) {}

       /* friend bool operator==(const Edge &_a, const Edge &_b);
        friend bool operator!=(const Edge &_a, const Edge &_b);*/

        bool operator==(const Edge &_b) { return a == _b.a && b == _b.b || a == _b.b && b == _b.a; }
        bool operator!=(const Edge &_b) { return !(*this == _b); }
    };

   /* bool operator==(const Edge &_a, const Edge &_b) { return _a.a == _b.a && _a.b == _b.b; }
    bool operator!=(const Edge &_a, const Edge &_b) { return !(_a == _b); }*/

    struct Plane
    {
        Vect3<float> normal;
        Vect3<float> position;
        Plane(const Vect3<float> &_normal, const Vect3<float> &_position) : normal(_normal), position(_position) {}
    };

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