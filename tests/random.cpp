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