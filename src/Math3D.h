#pragma once

#ifndef DEF_MATH3D
#define DEF_MATH3D

#include "Vect3.h"
#include <cmath>

namespace Math3D
{
    struct Plane
    {
        Vect3<float> normal;
        Vect3<float> position;
        Plane(const Vect3<float> &_normal, const Vect3<float> &_position) : normal(_normal), position(_position) {}
    };

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

    float DistanceToPlane(const Vect3<float> &_point, const Plane &_plane)
    {
        return abs(Dot((_point - _plane.position), _plane.normal));
    }

    float GetCircumCircleRadius(const float _distAB, const float _distBC, const float _distAC)
    {
        return ((_distAB * _distBC * _distAC) / sqrt(((_distAB + _distBC + _distAC)*(_distBC + _distAC - _distAB)*(_distAC + _distAB - _distBC)*(_distAB + _distBC - _distAC))));
    }

    Vect3<float> GetCircumcircleCenter(const Vect3<float> &_A, const Vect3<float> &_B, const Vect3<float> &_C)
    {
        float a = (_B - _C).GetSqrMagnitude();
        float b = (_A - _C).GetSqrMagnitude();
        float c = (_B - _A).GetSqrMagnitude();

        return (a * (b + c - a) * _A + b * (c + a - b) * _B + c * (a + b - c) * _C) / (a * (b + c - a) + b * (c + a - b) + c * (a + b - c));
    }
}

#endif // !DEF_MATH3D