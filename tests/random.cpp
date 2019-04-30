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

#include <random>
#include "common.h"

using namespace mygal;

using Float = double;

template<typename T>
std::vector<Vector2<T>> getRandomPoints(int nbPoints)
{
    auto seed = 0;
    auto generator = std::default_random_engine(seed);
    auto distribution = std::uniform_real_distribution<T>(0.0, 1.0);

    auto points = std::vector<Vector2<T>>(nbPoints);
    for (auto i = 0; i < nbPoints; ++i)
        points[i] = Vector2<T>(distribution(generator), distribution(generator));

    return points;
}

int main(int argc, char* argv[])
{
    assert(argc == 2);
    auto nbPoints = std::atoi(argv[1]);

    auto points = getRandomPoints<Float>(nbPoints);
    auto diagram = generateDiagram(points);

    return 0;
}