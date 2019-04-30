#pragma once

#include <limits>

namespace mygal
{

template<typename T>
static constexpr T EPSILON = std::numeric_limits<T>::epsilon();

// Almost predicates are easier to satisfy than the normal ones

template<typename T>
bool almostLower(T lhs, T rhs)
{
    return lhs <= rhs + EPSILON<T>;
}

template<typename T>
bool almostGreater(T lhs, T rhs)
{
    return lhs >= rhs - EPSILON<T>;
}

template<typename T>
bool almostEqual(T lhs, T rhs)
{
    return almostLower(lhs, rhs) && almostGreater(lhs, rhs);
}

template<typename T>
bool almostZero(T x)
{
    return almostEqual(x, static_cast<T>(0));
}

template<typename T>
bool almostBetween(T x, T a, T b)
{
    return almostGreater(x, a) && almostLower(x, b);
}

// Strictly predicates are harder to satisfy than the normal ones

template<typename T>
bool strictlyLower(T lhs, T rhs)
{
    return lhs < rhs - EPSILON<T>;
}

template<typename T>
bool strictlyGreater(T lhs, T rhs)
{
    return lhs > rhs + EPSILON<T>;
}

template<typename T>
bool strictlyBetween(T x, T a, T b)
{
    return strictlyGreater(x, a) && strictlyLower(x, b);
}

}