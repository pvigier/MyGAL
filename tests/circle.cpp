/* MyGAL
 * Copyright (C) 2019 Pierre Vigier
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

#include <cmath>
#include <algorithm>
#include "common.h"

using namespace mygal;

using Float = double;

static constexpr Float PI = 3.14159265358979323846264338327950288419716939937510;

template<typename T>
std::vector<Vector2<T>> getCircle(std::size_t n)
{
    auto points = std::vector<Vector2<T>>(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        auto angle = 2.0 * PI * static_cast<T>(i) / static_cast<T>(n);
        points[i] = Vector2<T>((std::cos(angle) + 1.0) / 2.0, (1.0 + std::sin(angle)) / 2.0);
    }
    return points;
}

bool isTriangulationCorrect(const Triangulation& triangulation)
{
    auto n = int(triangulation.getNbVertices());
    for (auto i = 0; i < n; ++i)
    {
        const auto& neighbors = triangulation.getNeighbors(i);
        // Check that the adjacent points are neighbors
        auto it = std::find(std::begin(neighbors), std::end(neighbors), (i - 1 + n) % n);
        if (it == std::end(neighbors))
            return false;
        it = std::find(std::begin(neighbors), std::end(neighbors), (i + 1) % n);
        if (it == std::end(neighbors))
            return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    assert(argc == 2);
    auto nbPoints = std::atoi(argv[1]);

    auto points = getCircle<Float>(nbPoints);
    auto diagram = generateDiagram(points);
    auto triangulation = diagram.computeTriangulation();
    assert(isTriangulationCorrect(triangulation));

    return 0;
}