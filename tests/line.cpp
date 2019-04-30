#include "common.h"

using namespace mygal;

using Float = double;

template<typename T>
std::vector<Vector2<T>> getAlignedPoints(std::size_t n)
{
    auto points = std::vector<Vector2<T>>(n);
    auto dx = static_cast<T>(1.0) / (n + 1);
    for (std::size_t i = 0; i < n; ++i)
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