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

#include <algorithm>
#include "common.h"

using namespace mygal;

using Float = double;

template<typename T>
std::vector<Vector2<T>> getGrid(std::size_t m, std::size_t n)
{
    auto points = std::vector<Vector2<T>>(m * n);
    auto dx = static_cast<T>(1.0) / (m + 1);
    auto dy = static_cast<T>(1.0) / (n + 1);
    for (auto i = std::size_t(0); i < n; ++i)
    {
        for (auto j = std::size_t(0); j < m; ++j)
            points[i * m + j] = Vector2<T>((j + 1) * dx, (i + 1) * dy);
    }
    return points;
}

bool isTriangulationCorrect(const Triangulation& triangulation, std::size_t n, std::size_t m)
{
    static auto deltas = std::array<std::array<int, 2>, 4>{{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}};
    for (auto i = 0; i < n; ++i)
    {
        for (auto j = 0; j < m; ++j)
        {
            const auto& neighbors = triangulation.getNeighbors(i * m + j);
            // Check that the adjacent cells are neighbors
            for (const auto& delta : deltas)
            {
                auto ni = i + delta[0];
                auto nj = j + delta[1];
                if (ni >= 0 && ni < n && nj >= 0 && nj < m)
                {
                    auto it = std::find(std::begin(neighbors), std::end(neighbors), ni * m + nj);
                    if (it == std::end(neighbors))
                        return false;
                }
            }
            // Check that neighbors are either adjacent or diagonal cells
            for (const auto& neighbor : neighbors)
            {
                auto ni = int(neighbor / m);
                auto nj = int(neighbor % m);
                if (std::abs(ni - i) + std::abs(nj - j) > 2)
                    return false;
            }
        }
    }
    return true;
}

int main(int argc, char* argv[])
{
    assert(argc == 3);
    auto m = std::atoi(argv[1]);
    auto n = std::atoi(argv[2]);

    auto points = getGrid<Float>(m, n);
    auto diagram = generateDiagram(points);
    auto triangulation = diagram.computeTriangulation();
    assert(isTriangulationCorrect(triangulation, m, n));

    return 0;
}