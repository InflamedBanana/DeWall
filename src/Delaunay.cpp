#include "Delaunay.h"
#include <iostream>
#include <utility>
#include "Math3D.h"
#include <algorithm>
#include <stack>

using namespace std;

#define SQRT(x) (x*x)

namespace Delaunay
{
    const PointSet *globalPointSet = nullptr;

    void PartitionPointSet(const std::vector<Vect3<float>> &pointSet, const Plane &wall, std::vector<Vect3<float>> &p1, std::vector<Vect3<float>> &p2)
    {
        for (auto &point : pointSet)
        {
            if (Dot(point - wall.position, wall.position + wall.normal) >= .0f)
                p2.push_back(point);                
            else
                p1.push_back(point);
        }
    }

    Plane GetDividingPlane(const vector<Vect3<float>> &pointSet)
    {
        float lowestX(99999), biggestX(-99999), lowestZ(99999), biggestZ(-99999);

        for (const auto &point : pointSet)
        {
            if (point.x < lowestX) lowestX = point.x;
            else if (point.x > biggestX) biggestX = point.x;
            if (point.z < lowestZ) lowestZ = point.z;
            else if (point.z > biggestZ) biggestZ = point.z;
        }

        if (biggestX - lowestX >= biggestZ - lowestZ)
            return Plane(Vect3<float>(1.0f, .0f, .0f), Vect3<float>((biggestX + lowestX) / 2.f, .0f, .0f));
        else
            return Plane(Vect3<float>(.0f, .0f, 1.0f), Vect3<float>(.0f, .0f, (biggestZ + lowestZ) / 2.f));
    }


    float DistanceToPlane(const Vect3<float> &_point, const Plane &_plane)
    {
        return abs(Dot((_point - _plane.position), _plane.normal));
    }

    float GetCircumCircleRadius(const float _distAB, const float _distBC, const float _distAC)
    {
        return ((_distAB * _distBC * _distAC) / sqrt(((_distAB + _distBC + _distAC)*(_distBC + _distAC - _distAB)*(_distAC + _distAB - _distBC)*(_distAB + _distBC - _distAC))));
    }

    Vect3<float> GetCircumCenter(const Vect3<float> &_A, const Vect3<float> &_B, const Vect3<float> &_C)
    {
        float a = (_B - _C).GetSqrMagnitude();
        float b = (_A - _C).GetSqrMagnitude();
        float c = (_B - _A).GetSqrMagnitude();

        return (a * (b + c - a) * _A + b * (c + a - b) * _B + c * (a + b - c) * _C) / (a * (b + c - a) + b * (c + a - b) + c * (a + b - c));
    }

    Triangle MakeFirstSimplex(const std::vector<Vect3<float>> &pointSet, const Plane &wall)
    {
        int a = -1;
        {
            float distToPlane = .0f;

            for (int i = 0; i < pointSet.size(); ++i)
            {
                float dist = DistanceToPlane(pointSet[i], wall);
                if (dist < distToPlane || a == -1)
                {
                    a = i;
                    distToPlane = dist;
                }
            }
        }

        int b = -1;
        float distAB = .0f;

        for (int i = 0; i < pointSet.size(); ++i)
        {
            Vect3<float> ray = pointSet[i] - pointSet[a];

            if (!Math3D::LinePlaneIntersection(pointSet[a], ray, wall.position, wall.normal) || i == a)
                continue;

            float dist = ray.GetSqrMagnitude();
            if (dist < distAB || b == -1)
            {
                b = i;
                distAB = dist;
            }
        }

        int c = -1;
        float circumcircleRadius = .0f;

        for (int i = 0; i < pointSet.size(); ++i)
        {
            if (i == a || i == b)
                continue;

            float radius = GetCircumCircleRadius(sqrt(distAB), (pointSet[i] - pointSet[b]).GetMagnitude(), (pointSet[i] - pointSet[a]).GetMagnitude());
            if (radius < circumcircleRadius || c == -1)
            {
                c = i;
                circumcircleRadius = radius;
            }
        }
        
        a = static_cast<int>(find(globalPointSet->begin(), globalPointSet->end(), pointSet[a]) - globalPointSet->begin());
        b = static_cast<int>(find(globalPointSet->begin(), globalPointSet->end(), pointSet[b]) - globalPointSet->begin());
        c = static_cast<int>(find(globalPointSet->begin(), globalPointSet->end(), pointSet[c]) - globalPointSet->begin());

        return Triangle(new Edge(a, b, c), new Edge(b, c, a), new Edge(c, a, b));
    }

    //> 0 if counterclockwise, < 0 if clockwise & == 0 is collinear
    int CounterClockWise(const Vect3<float> &_nextToTopStack, const Vect3<float> &_topStack, const Vect3<float> _point)
    {
        auto cross = (_topStack.x - _nextToTopStack.x)*(_point.z - _nextToTopStack.z) - (_topStack.z - _nextToTopStack.z)*(_point.x - _nextToTopStack.x);
        return cross > 0 ? 1 : (cross < 0 ? -1 : 0);
    }

    vector<Vect3<float>> GetConvexHull(vector<Vect3<float>> _pointSet)
    {
        auto lowestZCoord = min_element(_pointSet.begin(), _pointSet.end(), [](const Vect3<float>& a, const Vect3<float>& b) { return (a.z < b.z || (a.z == b.z && a.x < b.x)); });
        auto p = *lowestZCoord;
        _pointSet.erase(lowestZCoord);

        sort(_pointSet.begin(), _pointSet.end(), [&p](const Vect3<float>& a, const Vect3<float>& b)
        {
            Vect3<float> xAxis(1.0f, .0f, .0f);
            auto vectA = a - p;
            auto vectB = b - p;
            float cosA = Dot(vectA, xAxis) / vectA.GetMagnitude();
            float cosB = Dot(vectB, xAxis) / vectB.GetMagnitude();
            return cosA > cosB;
        });
        //Very bad.
        for (int i = _pointSet.size() - 1; i > 0; --i)
        {
            Vect3<float> xAxis(1.0f, .0f, .0f);
            auto vectA = _pointSet[i] - p;
            auto vectB = _pointSet[i - 1] - p;
            float cosA = Dot(vectA, xAxis) / vectA.GetMagnitude();
            float cosB = Dot(vectB, xAxis) / vectB.GetMagnitude();

            if (cosA == cosB)
            {
                if (vectA.GetMagnitude() > vectB.GetMagnitude())
                    _pointSet.erase(_pointSet.begin() + i - 1);
                else
                    _pointSet.erase(_pointSet.begin() + i);
            }
        }

        vector<Vect3<float>> convexHull = vector<Vect3<float>>();
        convexHull.push_back(p);

        for (const auto &point : _pointSet)
        {
            while (convexHull.size() > 1 && CounterClockWise(convexHull[convexHull.size() - 2], convexHull.back(), point) < 0)
                convexHull.pop_back();

            convexHull.push_back(point);
        }

        return convexHull;
    }

    void MakeSimplex(const Edge &_edge, const PointSet &_pointSet, Triangle *&_triangle, const vector<Vect3<float>> &_convexHull)
    {
        auto &a = (*globalPointSet)[_edge.a];
        auto &b = (*globalPointSet)[_edge.b];
        auto &opposite = (*globalPointSet)[_edge.oppositePoint];

        if (find(_convexHull.begin(), _convexHull.end(), a) != _convexHull.end() &&
            find(_convexHull.begin(), _convexHull.end(), b) != _convexHull.end())
        {
            _triangle = nullptr;
            return;
        }

        Vect3<float> edgeCenter = (a + b) / 2.0f;
        Vect3<float> planeNormal = GetCircumCenter(a, b, opposite) - edgeCenter;
        Plane halfPlane(planeNormal, edgeCenter);

        int cIndex = -1;
        float circumcircleRadius = .0f;
        
        bool cSide = Dot(opposite - halfPlane.position, halfPlane.normal) >= .0f;
        float distAB = (b - a).GetMagnitude();
        
        for (int i = 0; i < _pointSet.size(); ++i)
        {
            bool halfPlanePos = Dot(_pointSet[i] - halfPlane.position, halfPlane.normal) >= .0f;

            if (_pointSet[i] == a || _pointSet[i] == b || opposite == _pointSet[i] ||
                cSide && halfPlanePos || !cSide && !halfPlanePos ) // check if point is in HalfSpace.
                continue;

            float radius = 
                GetCircumCircleRadius(distAB,
                (_pointSet[i] - b).GetMagnitude(), 
                (_pointSet[i] - a).GetMagnitude());

            if (radius < circumcircleRadius || cIndex == -1)
            {
                cIndex = i;
                circumcircleRadius = radius;
            }
        }
        
        if (cIndex == -1)
        {
            _triangle = nullptr;
            return;
        }
        
        auto cIterator = find(globalPointSet->begin(), globalPointSet->end(), _pointSet[cIndex]);
        cIndex = static_cast<int>(cIterator - globalPointSet->begin());

        _triangle = new Triangle(const_cast<Edge*>(&_edge), new Edge(cIndex, _edge.a, _edge.b), new Edge(_edge.b, cIndex, _edge.a));
    }

    void AddEdgeToAFLs(const Edge &_edge, const Plane &_wall, const PointSet &_p1, vector<Edge> &_aflw, vector<Edge> &_afl1, vector<Edge> &_afl2)
    {
        auto &a = (*globalPointSet)[_edge.a];
        auto &b = (*globalPointSet)[_edge.b];
        if (Math3D::LinePlaneIntersection(a, b - a, _wall.position, _wall.normal))
        {
            _aflw.push_back(_edge);
        }
        else if (any_of(_p1.begin(), _p1.end(), [&_edge, &a](const Vect3<float> &vertice) {return vertice == a; }))
        {
            _afl1.push_back(_edge);
        }
        else
            _afl2.push_back(_edge);
    }

    std::vector<Triangle> Triangulate(const PointSet &_pointSet, std::vector<Edge> *_afl, std::vector<Vect3<float>> &_convexHull)
    {
        vector<Edge> aflw, afl1, afl2;
        vector<Triangle> triangles;
        PointSet p1, p2;

        Plane wall = GetDividingPlane(_pointSet);

        PartitionPointSet(_pointSet, wall, p1, p2);

        if (_afl == nullptr)
        {
            _afl = new vector<Edge>();
            Triangle t = MakeFirstSimplex(_pointSet, wall);
            _afl->push_back(*t.a);
            _afl->push_back(*t.b);
            _afl->push_back(*t.c);
            triangles.push_back(t);
        }
        for (const auto& edge : *_afl)
        {
            AddEdgeToAFLs(edge, wall, p1, aflw, afl1, afl2);
        }
        
        while (aflw.size() > 0)
        {
            auto edge = new Edge(aflw.back());
            aflw.pop_back();
            
            Triangle *triangle = nullptr;
            MakeSimplex(*edge, _pointSet, triangle, _convexHull);

            if (triangle != nullptr)
            {
                triangles.push_back(*triangle);

                AddEdgeToAFLs(*triangle->b , wall, p1, aflw, afl1, afl2);
                AddEdgeToAFLs(*triangle->c, wall, p1, aflw, afl1, afl2);
            }
        }
        //return triangles;

        if (afl1.size() > 0)
        {
            auto afl1Triangles = Triangulate(p1, &afl1, _convexHull);
            triangles.insert(triangles.end(), afl1Triangles.begin(), afl1Triangles.end());
        }
      
        if (afl2.size() > 0)
        {
            auto afl2Triangles = Triangulate(p2, &afl2, _convexHull);
            triangles.insert(triangles.end(), afl2Triangles.begin(), afl2Triangles.end());
        }

        return triangles;
    }

    std::vector<Triangle> Triangulate(const PointSet &_pointSet)
    {
        auto convexHull = GetConvexHull(_pointSet);

        globalPointSet = &_pointSet;
        auto triangles = Triangulate(_pointSet, nullptr, convexHull);

        globalPointSet = nullptr;
        return triangles;
    }

} // namespace Delaunay