#pragma once

#include "Vect3.h"
#include <iostream>
#include <cmath>
using namespace std;
namespace Math3D
{
    //0 = no intersection
    //1 = intersection at _intersectionPoint
    //2 = parallel to plane
    template<typename T>
    int RayPlaneIntersection(const Vect3<T> &_rayStart, const Vect3<T> &_rayDir, const Vect3<T> &_planePoint, const Vect3<T> &_planeNorm, Vect3<T> &_intersectionPoint)
    {
        float D = Dot(_planeNorm, _rayDir);
        float N = -Dot(_planeNorm, _rayStart - _planePoint);

        if (fabs(D) < .0000001f)
        {
            if (N == 0)
                return 2;
            else
                return 0;
        }

        float sI = N / D;
        if (sI < 0)
            return 0;

        _intersectionPoint = _rayStart + _rayDir * sI;

        return 1;
    }

    template<typename T>
    bool LinePlaneIntersection(const Vect3<T> &_rayStart, const Vect3<T> &_rayDir, const Vect3<T> &_planePoint, const Vect3<T> &_planeNorm)
    {
        float D = Dot(_planeNorm, _rayDir);
        float N = -Dot(_planeNorm, _rayStart - _planePoint);

        if (fabs(D) < .000001f)
            return false;

        float sI = N / D;
        if (sI < 0 || sI > 1)
            return false;

        return true;
    }
}
  