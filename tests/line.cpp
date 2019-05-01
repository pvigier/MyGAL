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

#include "common.h"

using namespace mygal;

using Float = double;

template<typename T>
std::vector<Vector2<T>> getAlignedPoints(std::size_t n)
{
    auto points = std::vector<Vector2<T>>(n);
    auto dx = static_cast<T>(1.0) / (n + 1);
    for (auto i = std::size_t(0); i < n; ++i)
        points[i] = Vector2<T>((i + 1) * dx, 0.5);
    return points;
}

bool isTriangulationCorrect(const Triangulation& triangulation)
{
    for (auto i = std::size_t(0); i < triangulation.getNbVertices(); ++i)
    {
        const auto& neighbors = triangulation.getNeighbors(i);
        if ((i == 0 || i == triangulation.getNbVertices() - 1))
        {
            if (neighbors.size() != 1)
                return false;
        } 
        else
        {
            if (neighbors.size() != 2)
                return false;
        }
        for (auto neighbor : neighbors)
        {
            if (neighbor != (i - 1) && neighbor != (i + 1))
                return false;
        }
    }
    return true;
}

int main(int argc, char* argv[])
{
    assert(argc == 2);
    auto nbPoints = std::atoi(argv[1]);

    auto points = getAlignedPoints<Float>(nbPoints);
    auto diagram = generateDiagram(points);
    auto triangulation = diagram.computeTriangulation();
    assert(isTriangulationCorrect(triangulation));

    return 0;
}