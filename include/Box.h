/* FortuneAlgorithm
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
#include <array>
#include <limits>
// My includes
#include "Vector2.h"

template<typename T>
class Box
{
public:
    // Be careful, y-axis is oriented to the top like in math
    enum class Side : int {LEFT, BOTTOM, RIGHT, TOP};

    struct Intersection
    {
        Side side;
        Vector2<T> point;
    };

    T left;
    T bottom;
    T right;
    T top;

    bool contains(const Vector2<T>& point) const
    {
        return point.x >= left - EPSILON && point.x <= right + EPSILON &&
            point.y >= bottom  - EPSILON && point.y <= top + EPSILON;
    }

    // Useful for Fortune's algorithm
    Intersection getFirstIntersection(const Vector2<T>& origin, const Vector2<T>& direction) const
    {
        // origin must be in the box
        auto intersection = Intersection{};
        auto t = std::numeric_limits<T>::infinity();
        if (direction.x > static_cast<T>(0.0))
        {
            t = (right - origin.x) / direction.x;
            intersection.side = Side::RIGHT;
            intersection.point = origin + t * direction;
        }
        else if (direction.x < static_cast<T>(0.0))
        {
            t = (left - origin.x) / direction.x;
            intersection.side = Side::LEFT;
            intersection.point = origin + t * direction;
        }
        if (direction.y > static_cast<T>(0.0))
        {
            auto newT = (top - origin.y) / direction.y;
            if (newT < t)
            {
                intersection.side = Side::TOP;
                intersection.point = origin + newT * direction;
            }
        }
        else if (direction.y < static_cast<T>(0.0))
        {
            auto newT = (bottom - origin.y) / direction.y;
            if (newT < t)
            {
                intersection.side = Side::BOTTOM;
                intersection.point = origin + newT * direction;
            }
        }
        return intersection;
    }

    // Useful for diagram intersection
    int getIntersections(const Vector2<T>& origin, const Vector2<T>& destination, std::array<Intersection, 2>& intersections) const
    {
        // WARNING: If the intersection is a corner, both intersections are equals
        auto direction = destination - origin;
        auto t = std::array<T, 2>();
        auto i = std::size_t(0); // index of the current intersection
        // Left
        if (origin.x < left - EPSILON || destination.x < left - EPSILON)
        {   
            t[i] = (left - origin.x) / direction.x;
            if (t[i] > EPSILON && t[i] < static_cast<T>(1.0) - EPSILON)  
            {
                intersections[i].side = Side::LEFT;
                intersections[i].point = origin + t[i] * direction;
                if (intersections[i].point.y >= bottom  - EPSILON && intersections[i].point.y <= top + EPSILON)
                    ++i;
            }
        }
        // Right
        if (origin.x > right + EPSILON || destination.x > right + EPSILON)
        {   
            t[i] = (right - origin.x) / direction.x;
            if (t[i] > EPSILON && t[i] < static_cast<T>(1.0) - EPSILON)  
            {
                intersections[i].side = Side::RIGHT;
                intersections[i].point = origin + t[i] * direction;
                if (intersections[i].point.y >= bottom - EPSILON && intersections[i].point.y <= top + EPSILON)
                    ++i;
            }
        }
        // Bottom
        if (origin.y < bottom - EPSILON || destination.y < bottom - EPSILON)
        {   
            t[i] = (bottom - origin.y) / direction.y;
            if (i < 2 && t[i] > EPSILON && t[i] < static_cast<T>(1.0) - EPSILON)  
            {
                intersections[i].side = Side::BOTTOM;
                intersections[i].point = origin + t[i] * direction;
                if (intersections[i].point.x >= left  - EPSILON && intersections[i].point.x <= right + EPSILON)
                    ++i;
            }
        }
        // Top
        if (origin.y > top + EPSILON || destination.y > top + EPSILON)
        {   
            t[i] = (top - origin.y) / direction.y;
            if (i < 2 && t[i] > EPSILON && t[i] < static_cast<T>(1.0) - EPSILON)  
            {
                intersections[i].side = Side::TOP;
                intersections[i].point = origin + t[i] * direction;
                if (intersections[i].point.x >= left - EPSILON && intersections[i].point.x <= right + EPSILON)
                    ++i;
            }
        }
        // Sort the intersections from the nearest to the farthest
        if (i == 2 && t[0] > t[1])
            std::swap(intersections[0], intersections[1]);
        return i;
    }

private:
    static constexpr T EPSILON = std::numeric_limits<T>::epsilon();
};

