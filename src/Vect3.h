#pragma once

#ifndef DEF_VECT3
#define DEF_VECT3

#include <iostream>
#include <cmath>

template <typename T> struct Vect3 {
    T x;
    T y;
    T z;

    virtual ~Vect3() {}
    Vect3() = default;
    Vect3(T _value) : x(_value), y(_value), z(_value) {}
    Vect3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

    Vect3(const Vect3 &_other) : x(_other.x), y(_other.y), z(_other.z) {}
    Vect3 &operator=(const Vect3 &_rhs) {
        if (&_rhs == this)
            return *this;
        x = _rhs.x;
        y = _rhs.y;
        z = _rhs.z;
    }

    Vect3(Vect3 &&_rhs) noexcept : x(_rhs.x), y(_rhs.y), z(_rhs.z) {
        _rhs.x = _rhs.y = _rhs.z = .0f;
    }

    Vect3 &operator=(Vect3 &&_rhs) noexcept {
        if (&_rhs == this)
            return *this;
        x = _rhs.x;
        y = _rhs.y;
        z = _rhs.z;
        _rhs.x = _rhs.y = _rhs.z = .0f;
        return *this;
    }

    float const GetSqrMagnitude() { return x*x + y*y + z*z; }
    float const GetMagnitude() { return static_cast<float>(std::sqrt(x*x + y * y + z * z)); }

    /*bool operator==(const Vect3 &_rhs) {
        return x == _rhs.x && y == _rhs.y && z == _rhs.z;
    }
    bool operator!=(const Vect3 &_rhs) { return !(*this == _rhs); }*/

    template <typename T>
    friend bool operator==(const Vect3<T> &_a, const Vect3<T> &_b);
    template <typename T>
    friend bool operator!=(const Vect3<T> &_a, const Vect3<T> &_b);

   /* Vect3 operator+(const Vect3 &_rhs) {
        return Vect3(x + _rhs.x, y + _rhs.y, z + _rhs.z);
    }
    Vect3 operator-(const Vect3 &_rhs) {
        return Vect3(x - _rhs.x, y - _rhs.y, z - _rhs.z);
    }*/
    
    template <typename T>
    friend Vect3<T> operator+(const Vect3<T> &_v, const Vect3<T> &_v2);
    template <typename T>
    friend Vect3<T> operator-(const Vect3<T> &_v, const Vect3<T> &_v2);

    template <typename T>
    friend Vect3<T> operator/(const Vect3<T> &_v, const float &_a);
    template <typename T>
    friend Vect3<T> operator/(const float &_a, const Vect3<T> &_v);

    template <typename T>
    friend Vect3<T> operator/(const Vect3<T> &_v, const int &_a);
    template <typename T>
    friend Vect3<T> operator/(const int &_a, const Vect3<T> &_v);

    template <typename T>
    friend Vect3<T> operator*(const Vect3<T> &_v, const float &_a);
    template <typename T>
    friend Vect3<T> operator*(const float &_a, const Vect3<T> &_v);

    template <typename T>
    friend Vect3<T> operator*(const Vect3<T> &_v, const int &_a);
    template <typename T>
    friend Vect3<T> operator*(const int &_a, const Vect3<T> &_v);

    Vect3 &operator+=(const Vect3 &_rhs) {
        x += _rhs.x;
        y += _rhs.y;
        z += _rhs.z;
        return *this;
    }
    Vect3 &operator-=(const Vect3 &_rhs) {
        x -= _rhs.x;
        y -= _rhs.y;
        z -= _rhs.z;
        return *this;
    }
    Vect3 &operator/=(const int _value) {
        x /= _value;
        y /= _value;
        z /= _value;
        return *this;
    }
    Vect3 &operator/=(const float _value) {
        x /= _value;
        y /= _value;
        z /= _value;
        return *this;
    }
    Vect3 &operator/=(const short _value) {
        x /= _value;
        y /= _value;
        z /= _value;
        return *this;
    }
    Vect3 &operator/=(const long _value) {
        x /= _value;
        y /= _value;
        z /= _value;
        return *this;
    }
    Vect3 &operator*=(const int _value) {
        x *= _value;
        y *= _value;
        z *= _value;
        return *this;
    }
    Vect3 &operator*=(const float _value) {
        x *= _value;
        y *= _value;
        z *= _value;
        return *this;
    }
    Vect3 &operator*=(const short _value) {
        x *= _value;
        y *= _value;
        z *= _value;
        return *this;
    }
    Vect3 &operator*=(const long _value) {
        x *= _value;
        y *= _value;
        z *= _value;
        return *this;
    }

    template <typename U>
    friend std::ostream &operator<<(std::ostream &stream, const Vect3<U> &v3);
};

template <typename T>
std::ostream &operator<<(std::ostream &stream, const Vect3<T> &v3) {
    return (stream << "( " << v3.x << ", " << v3.y << ", " << v3.z << " )");
}

template <typename T>
Vect3<T> operator/(const Vect3<T> &_v, const int &_a)
{
    return Vect3<T>(_v.x / _a, _v.y / _a, _v.z / _a);
}
template <typename T>
Vect3<T> operator/(const int &_a, const Vect3<T> &_v) { return _v / _a; }

template <typename T>
Vect3<T> operator/(const Vect3<T> &_v, const float &_a)
{
    return Vect3<T>(_v.x / _a, _v.y / _a, _v.z / _a);
}
template <typename T>
Vect3<T> operator/(const float &_a, const Vect3<T> &_v) { return _v / _a; }


template <typename T>
Vect3<T> operator*(const Vect3<T> & _v, const float & _a)
{
    return Vect3<T>(_v.x * _a, _v.y * _a, _v.z * _a);
}
template <typename T>
Vect3<T> operator*(const float & _a, const Vect3<T> & _v) { return _v * _a; }

template <typename T>
Vect3<T> operator*(const Vect3<T> & _v, const int & _a)
{
    return Vect3<T>(_v.x * _a, _v.y * _a, _v.z * _a);
}
template <typename T>
Vect3<T> operator*(const int & _a, const Vect3<T> & _v) { return _v * _a; }

template <typename T>
Vect3<T> operator+(const Vect3<T> &_v, const Vect3<T> &_v2)
{
    return Vect3<T>(_v.x + _v2.x, _v.y + _v2.y, _v.z + _v2.z);
}

template <typename T>
Vect3<T> operator-(const Vect3<T> &_v, const Vect3<T> &_v2)
{
    return Vect3<T>(_v.x - _v2.x, _v.y - _v2.y, _v.z - _v2.z);
}

template <typename T>
bool operator==(const Vect3<T> &_a, const Vect3<T> &_b)
{
    return _a.x == _b.x && _a.y == _b.y && _a.z == _b.z;
}
template <typename T>
bool operator!=(const Vect3<T> &_a, const Vect3<T> &_b)
{
    return !(_a == _b);
}


template <typename T>
T Dot(const Vect3<T> &a, const Vect3<T> &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

#endif // DEF_VECT3
