#include "Delaunay.h"
#include <iostream>
#include <utility>
#include "Math3D.h"
#include <algorithm>

using namespace std;

namespace Delaunay
{
    //typedef std::vector<Vect3<float>> PointSet;

    namespace
    {
        void PartitionPointSet(const std::vector<Vect3<float>> &pointSet, const Plane &wall, std::vector<Vect3<float>> &p1, std::vector<Vect3<float>> &p2)
        {
            for (auto &point : pointSet)
            {
                cout << "point : " << point << " - " << wall.position << " = " << (point - wall.position) << endl;
                if (Dot(point - wall.position, wall.position + wall.normal) >= .0f)
                {
                    p2.push_back(point);
                }
                else
                    p1.push_back(point);
            }
        }

        /* Plane GetDividingPlane(const vector<Vect3<float>> &pointSet)
         {*/
         //Create a line
         //Projects all other points on line ( ignore y )
         //order them
         //get center point
         //return plane 
     //}


        float DistanceToPlane(const Vect3<float> &_point, const Plane &_plane)
        {
            return abs(Dot((_point - _plane.position), _plane.normal));
        }

        float GetCircumCircleRadius(const float _distAB, const float _distBC, const float _distAC)
        {
            return ((_distAB * _distBC * _distAC) / sqrt(((_distAB + _distBC + _distAC)*(_distBC + _distAC - _distAB)*(_distAC + _distAB - _distBC)*(_distAB + _distBC - _distAC))));
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

                if (!Math3D::RayPlaneIntersection(pointSet[a], ray, wall.position, wall.normal) || i == a)
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

            return Triangle(a, b, c);
        }
    }

    std::vector<Triangle> Triangulate(const PointSet &_vertices, std::vector<Edge> *_afl)
    {
        vector<Edge> aflw, afl1, afl2;

        PointSet p1, p2;
        Plane wall(Vect3<float>(1.f, .0f, 0.f), Vect3<float>(.0f, .0f, .0f)); // calculate wall

        vector<Triangle> triangles;
        //new list /\

        PartitionPointSet(_vertices, wall, p1, p2);

        if (_afl == nullptr)
        {
            Triangle t = MakeFirstSimplex(_vertices, wall);
            _afl->push_back(Edge(_vertices[t.a], _vertices[t.b]));
            _afl->push_back(Edge(_vertices[t.b], _vertices[t.c]));
            _afl->push_back(Edge(_vertices[t.c], _vertices[t.a]));
            triangles.push_back(t);
        }

        for (const auto& edge : *_afl)
        {
            if (Math3D::RayPlaneIntersection(edge.a, edge.b - edge.a, wall.position, wall.normal))
            {
                aflw.push_back(edge);
            }
            else if (any_of(p1.begin(), p1.end(), [edge](const Vect3<float> &vertice) {return vertice == edge.a; }))
            {
                afl1.push_back(edge);
            }
            else
                afl2.push_back(edge);
        }

        while (aflw.size() > 0)
        {
            //get last edge
            //MakeSimplex(edge, _vertices)
        }



        //return List/\

        return vector<Triangle>();
    }

} // namespace Delaunay