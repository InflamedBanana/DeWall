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
    };

    struct Plane
    {
        Vect3<float> normal;
        Vect3<float> position;
        Plane(const Vect3<float> &_normal, const Vect3<float> &_position) : normal(_normal), position(_position) {}
    };

    struct Triangle
    {
        Edge* a;
        Edge* b;
        Edge* c;
        Triangle(Edge *_a, Edge *_b, Edge *_c) : a(_a), b(_b), c(_c) {}
    };

    typedef std::vector<Vect3<float>> PointSet;


    std::vector<Triangle> Triangulate(const PointSet &_vertices, std::vector<Edge> *_edges = nullptr, std::vector<Vect3<float>> *_convexHull = nullptr);
}