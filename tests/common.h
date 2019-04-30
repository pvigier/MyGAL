#include <cassert>
#include <MyGAL/FortuneAlgorithm.h>

// Tests

template<typename T>
bool isBounded(mygal::Diagram<T>& diagram)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto face = diagram.getFace(i);
        auto start = face->outerComponent;
        auto halfEdge = start;
        do
        {
            halfEdge = halfEdge->next;
            if (halfEdge == nullptr)
            {
                std::cerr << "Face " << i << " is not bounded.\n";
                return false;
            }
        } while (halfEdge != start);
    }
    return true;
}

template<typename T>
bool isIntersectionValid(mygal::Diagram<T>& diagram, const mygal::Box<T>& box)
{
    for (const auto& vertex : diagram.getVertices())
    {
        if (!box.contains(vertex.point))
            return false;
    }
    return true;
}

// Diagram

template<typename T>
mygal::Diagram<T> generateDiagram(const std::vector<mygal::Vector2<T>>& points)
{
    // Construct diagram
    auto algorithm = mygal::FortuneAlgorithm<T>(points);
    algorithm.construct();

    // Bound the diagram
    algorithm.bound(mygal::Box<T>{-0.05, -0.05, 1.05, 1.05});
    auto diagram = algorithm.getDiagram();
    assert(isBounded(diagram));

    // Intersect the diagram with a box
    auto box = mygal::Box<T>{0.0, 0.0, 1.0, 1.0};
    bool success = diagram.intersect(box);
    assert(success);
    assert(isIntersectionValid(diagram, box));

    return diagram;
}