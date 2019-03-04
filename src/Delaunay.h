#pragma once

#include <vector>
#include "Vect3.h"

namespace Delaunay
{
    struct Edge
    {
        Vect3<float> a;
        Vect3<float> b;
        Edge(const Vect3<float> &_a, const Vect3<float> &_b) : a(_a), b(_b) {}
    };

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
        Triangle(const int _a, const int _b, const int _c) : a(_a), b(_b), c(_c) {}
    };

    typedef std::vector<Vect3<float>> PointSet;


    std::vector<Triangle> Triangulate(const PointSet &_vertices, std::vector<Edge> *_edges = nullptr);
}