#include "DeWall.h"
#include "Math3D.h"
#include <algorithm>
#include <unordered_set>
#include <numeric>

using namespace std;
using namespace Math3D;

namespace DeWall
{
    const PointSet *globalPointSet = nullptr;
    const unordered_set<int> *convexHull = nullptr;

    void PartitionPointSet(const std::vector<Vect3<float>> &pointSet, const Plane &wall, std::vector<Vect3<float>> &p1, std::vector<Vect3<float>> &p2)
    {
        for (auto &point : pointSet)
        {
            if (Dot(point - wall.position, wall.normal) >= .0f)
                p2.push_back(point);                
            else
                p1.push_back(point);
        }
    }

    Plane GetDividingPlane(const vector<Vect3<float>> &pointSet, const Vect3<float> &_minBounds,const Vect3<float> &_maxBounds)
    {
        auto position = Vect3<float>((_maxBounds.x + _minBounds.x) / 2.f, .0f, (_maxBounds.z + _minBounds.z) / 2.f);

        if (_maxBounds.x - _minBounds.x >= _maxBounds.z - _minBounds.z)
            return Plane(Vect3<float>(1.0f, .0f, .0f), position);
        else
            return Plane(Vect3<float>(.0f, .0f, 1.0f), position);
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

        return Triangle(a, b, c);
    }

    //> 0 if counterclockwise, < 0 if clockwise & == 0 is collinear
    int CounterClockWise(const Vect3<float> &_nextToTopStack, const Vect3<float> &_topStack, const Vect3<float> _point)
    {
        auto cross = (_topStack.x - _nextToTopStack.x)*(_point.z - _nextToTopStack.z) - (_topStack.z - _nextToTopStack.z)*(_point.x - _nextToTopStack.x);
        return cross > 0 ? 1 : (cross < 0 ? -1 : 0);
    }

	//Algo is right, implementation is bad.
    unordered_set<int> GetConvexHull()
    {
		const PointSet &pointSet = *globalPointSet;

		vector<int> indexes(pointSet.size());
		std::iota(indexes.begin(), indexes.end(), 0);

        auto lowestZCoord = min_element(pointSet.begin(), pointSet.end(), [](const Vect3<float>& a, const Vect3<float>& b) { return (a.z < b.z || (a.z == b.z && a.x < b.x)); });
        auto p = *lowestZCoord;
		int pIndex = static_cast<int>(lowestZCoord - pointSet.begin());
		indexes.erase(indexes.begin() + pIndex);

		sort(indexes.begin(), indexes.end(), [&p, &pointSet](const int& a, const int& b)
		{
			Vect3<float> xAxis(1.0f, .0f, .0f);
			auto vectA = pointSet[a] - p;
			auto vectB = pointSet[b] - p;
			float cosA = Dot(vectA, xAxis) / vectA.GetMagnitude();
			float cosB = Dot(vectB, xAxis) / vectB.GetMagnitude();
			return cosA > cosB;
		});
        //Very bad.
		for (int i = indexes.size() - 1; i > 0; --i)
		{
			Vect3<float> xAxis(1.0f, .0f, .0f);
			auto vectA = pointSet[indexes[i]] - p;
			auto vectB = pointSet[indexes[i-1]] - p;
			float cosA = Dot(vectA, xAxis) / vectA.GetMagnitude();
			float cosB = Dot(vectB, xAxis) / vectB.GetMagnitude();

			if (cosA == cosB)
			{
				if (vectA.GetMagnitude() > vectB.GetMagnitude())
					indexes.erase(indexes.begin() + i - 1);
				else
					indexes.erase(indexes.begin() + i);
			}
		}

        vector<int> convexHull = vector<int>();
        convexHull.push_back(pIndex);

		for(const auto &i : indexes)
        {
            while (convexHull.size() > 1 && CounterClockWise(pointSet[convexHull[convexHull.size() - 2]], pointSet[convexHull.back()], pointSet[i]) < 0)
                convexHull.pop_back();

            convexHull.push_back(i);
        }

        return unordered_set<int>(convexHull.begin(), convexHull.end());
    }

    void MakeSimplex(const Edge &_edge, const PointSet &_pointSet, Triangle *&_triangle)
    {
        auto &a = (*globalPointSet)[_edge.a];
        auto &b = (*globalPointSet)[_edge.b];
        auto &opposite = (*globalPointSet)[_edge.oppositePoint];

		if(convexHull->find(_edge.a) != convexHull->end() && convexHull->find(_edge.b) != convexHull->end())
        {
            _triangle = nullptr;
            return;
        }
			
        float distAB = (b - a).GetMagnitude();

        auto abDir = (b - a) / distAB;
        auto acDir = (opposite - a);

        float doot = Dot(abDir, acDir);
        Vect3<float> bisectionPos = (a + (abDir * doot));

        Plane halfPlane(opposite - bisectionPos, bisectionPos);

        int cIndex = -1;
        float circumcircleRadius = .0f;

        for (int i = 0; i < _pointSet.size(); ++i)
        {
            if (_pointSet[i] == a || _pointSet[i] == b || opposite == _pointSet[i] ||
                Dot(_pointSet[i] - halfPlane.position, halfPlane.normal) >= .0f)
            {  
                continue;
            }

            float distBC = (_pointSet[i] - b).GetMagnitude();
            float distAC = (_pointSet[i] - a).GetMagnitude();

            float p = (distAB + distBC + distAC) / 2.f;
            if (p * (p - distAB) * (p - distBC) * (p - distAC) <= .0000001f )//Area
                continue;
			
           auto center = GetCircumcircleCenter<float>(a, b, _pointSet[i]);

            float radius = (center - a).GetMagnitude();

            if (Dot(center - halfPlane.position, halfPlane.normal) >= .0f)
            {
                radius *= -1;
            }

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

        _triangle = new Triangle(_edge.a, _edge.b, cIndex);
    }

    void UpdateAFL(const Edge &_edge, vector<Edge> &_afl)
    {
        auto it = find(_afl.begin(), _afl.end(), _edge);
        if (it != _afl.end())
            _afl.erase(it);
        else
            _afl.push_back(_edge);
    }

    void AddEdgeToAFLs(const Edge &_edge, const Plane &_wall, const PointSet &_p1, vector<Edge> &_aflw, vector<Edge> &_afl1, vector<Edge> &_afl2)
    {
        auto &a = (*globalPointSet)[_edge.a];
        auto &b = (*globalPointSet)[_edge.b];
        if (Math3D::LinePlaneIntersection(a, b - a, _wall.position, _wall.normal))
            UpdateAFL(_edge, _aflw);
        else if (any_of(_p1.begin(), _p1.end(), [&_edge, &a](const Vect3<float> &vertice) {return vertice == a; }))
            UpdateAFL(_edge, _afl1);
        else
            UpdateAFL(_edge, _afl2);
    }

    int e = 0;

    std::vector<Triangle> Triangulate(const PointSet &_pointSet, std::vector<Edge> *_afl,const Vect3<float> &_minBounds, const Vect3<float> &_maxBounds)
    {
        vector<Edge> aflw, afl1, afl2;
        vector<Triangle> triangles;
        PointSet p1, p2;

        Plane wall = GetDividingPlane(_pointSet, _minBounds, _maxBounds);

        PartitionPointSet(_pointSet, wall, p1, p2);

        if (_afl == nullptr)
        {
            _afl = new vector<Edge>();
            Triangle t = MakeFirstSimplex(_pointSet, wall);
            _afl->push_back(t.AB());
            _afl->push_back(t.BC());
            _afl->push_back(t.CA());
            triangles.push_back(t);
        }

        for (const auto& edge : *_afl)
			AddEdgeToAFLs(edge, wall, p1, aflw, afl1, afl2);
        
        while (aflw.size() > 0)
        {
            auto edge = new Edge(aflw.back());
            
            Triangle *triangle = nullptr;
            MakeSimplex(*edge, _pointSet, triangle);

            if (triangle != nullptr)
            {
                triangles.push_back(*triangle);
               
                AddEdgeToAFLs(triangle->AB(), wall, p1, aflw, afl1, afl2);
                AddEdgeToAFLs(triangle->BC() , wall, p1, aflw, afl1, afl2);
                AddEdgeToAFLs(triangle->CA(), wall, p1, aflw, afl1, afl2);
            }
            else
                aflw.pop_back();
        }

        if (afl1.size() > 0)
        {
            auto p1MinBounds = Vect3<float>(_minBounds);
            auto p1MaxBounds = Vect3<float>(_maxBounds);

            if (p1.size() > 0)
            {
                if (wall.normal.x > 0)
                {
                    p1MaxBounds.x = wall.position.x;
                }
                else if (wall.normal.z > 0)
                {
                    p1MaxBounds.z = wall.position.z;
                }
            }

            auto afl1Triangles = Triangulate(p1, &afl1, p1MinBounds, p1MaxBounds);
            triangles.insert(triangles.end(), afl1Triangles.begin(), afl1Triangles.end());
        }

        if (afl2.size() > 0)
        {
            auto p2MinBounds = Vect3<float>(_minBounds);
            auto p2MaxBounds = Vect3<float>(_maxBounds);

            if (p2.size() > 0)
            {
                if (wall.normal.x > 0)
                {
                    p2MinBounds.x = wall.position.x;
                }
                else if (wall.normal.z > 0)
                {
                    p2MinBounds.z = wall.position.z;
                }
            }

            auto afl2Triangles = Triangulate(p2, &afl2, p2MinBounds, p2MaxBounds);
            triangles.insert(triangles.end(), afl2Triangles.begin(), afl2Triangles.end());
        }

        return triangles;
    }


    std::vector<Triangle> Triangulate(const PointSet &_pointSet)
    {
        globalPointSet = &_pointSet;

        auto hull = GetConvexHull();
        convexHull = &hull;

		auto maxIntValue = (int)((~((unsigned int)0)) >> 1);

		Vect3<float> minBounds(maxIntValue, maxIntValue, maxIntValue),
			maxBounds(-maxIntValue, -maxIntValue, -maxIntValue);

		for(const auto& point : *convexHull)
		{
			if (_pointSet[point].x < minBounds.x) minBounds.x = _pointSet[point].x;
			else if (_pointSet[point].x > maxBounds.x) maxBounds.x = _pointSet[point].x;
			if (_pointSet[point].y < minBounds.y) minBounds.y = _pointSet[point].y;
			else if (_pointSet[point].y > maxBounds.y) maxBounds.y = _pointSet[point].y;
			if (_pointSet[point].z < minBounds.z) minBounds.z = _pointSet[point].z;
			else if (_pointSet[point].z > maxBounds.z) maxBounds.z = _pointSet[point].z;
		}
        auto triangles = Triangulate(_pointSet, nullptr, minBounds, maxBounds);

        globalPointSet = nullptr;
        convexHull = nullptr;

        return triangles;
    }

} // namespace Delaunay