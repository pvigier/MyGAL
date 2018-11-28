/* MyGAL
 * Copyright (C) 2018 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// STL
#include <ostream>
#include <cmath>

// Declarations

template<typename T>
class Vector2;
template<typename T>
Vector2<T> operator-(Vector2<T> lhs, const Vector2<T>& rhs);

// Implementations

template<typename T>
class Vector2
{
public:
    double x;
    double y;

    Vector2<T>(double x = 0.0, double y = 0.0) : x(x), y(y)
    {

    }

    // Unary operators

    Vector2<T> operator-() const
    {
        return Vector2<T>(-x, -y);
    }

    Vector2<T>& operator+=(const Vector2<T>& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2<T>& operator-=(const Vector2<T>& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2<T>& operator*=(double t)
    {
        x *= t;
        y *= t;
        return *this; 
    }
    
    // Other operations
    
    Vector2<T> getOrthogonal() const
    {
        return Vector2<T>(-y, x);
    }

    double dot(const Vector2<T>& other) const
    {
        return x * other.x + y * other.y;
    }

    double getNorm() const
    {
        return std::sqrt(x * x + y * y);
    }

    double getDistance(const Vector2<T>& other) const
    {
        return (*this - other).getNorm();
    }

    double getDet(const Vector2<T>& other) const
    {
        return x * other.y - y * other.x;
    }
};

// Binary operators

template<typename T>
Vector2<T> operator+(Vector2<T> lhs, const Vector2<T>& rhs)
{
    lhs += rhs;
    return lhs;
}

template<typename T>
Vector2<T> operator-(Vector2<T> lhs, const Vector2<T>& rhs)
{
    lhs -= rhs;
    return lhs;
}

template<typename T>
Vector2<T> operator*(double t, Vector2<T> vec)
{
    vec *= t;
    return vec;
}

template<typename T>
Vector2<T> operator*(Vector2<T> vec, double t)
{
    return t * vec;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector2<T>& vec)
{
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}

