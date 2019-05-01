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

// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
// SFML
#include <SFML/Graphics.hpp>
// MyGAL
#include <MyGAL/FortuneAlgorithm.h>

using namespace mygal;

using Float = double;

constexpr Float WindowWidth = 600.0f;
constexpr Float WindowHeight = 600.0f;
constexpr Float PointRadius = 0.005f;
constexpr Float Offset = 1.0f;

// Points generation

template<typename T>
std::vector<Vector2<T>> generatePoints(int nbPoints)
{
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::cout << "seed: " << seed << '\n';
    auto generator = std::default_random_engine(seed);
    auto distribution = std::uniform_real_distribution<T>(0.0, 1.0);

    auto points = std::vector<Vector2<T>>(nbPoints);
    for (auto i = 0; i < nbPoints; ++i)
        points[i] = Vector2<T>(distribution(generator), distribution(generator));

    return points;
}

// Rendering

template<typename T>
void drawPoint(sf::RenderWindow& window, Vector2<T> point, sf::Color color)
{
    auto shape = sf::CircleShape(PointRadius);
    shape.setPosition(sf::Vector2f(point.x - PointRadius, 1.0 - point.y - PointRadius));
    shape.setFillColor(color);
    window.draw(shape);
}

template<typename T>
void drawEdge(sf::RenderWindow& window, Vector2<T> origin, Vector2<T> destination, sf::Color color)
{
    auto line = std::array<sf::Vertex, 2>
    {
        sf::Vertex(sf::Vector2f(origin.x, 1.0 - origin.y), color),
        sf::Vertex(sf::Vector2f(destination.x, 1.0 - destination.y), color)
    };
    window.draw(line.data(), 2, sf::Lines);
}

template<typename T>
void drawPoints(sf::RenderWindow& window, Diagram<T>& diagram)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
        drawPoint(window, diagram.getSite(i)->point, sf::Color(100, 250, 50));
}

template<typename T>
void drawDiagram(sf::RenderWindow& window, Diagram<T>& diagram)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto site = diagram.getSite(i);
        auto center = site->point;
        auto face = site->face;
        auto halfEdge = face->outerComponent;
        if (halfEdge == nullptr)
            continue;
        while (halfEdge->prev != nullptr)
        {
            halfEdge = halfEdge->prev;
            if (halfEdge == face->outerComponent)
                break;
        }
        auto start = halfEdge;
        while (halfEdge != nullptr)
        {
            if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
            {
                auto origin = (halfEdge->origin->point - center) * Offset + center;
                auto destination = (halfEdge->destination->point - center) * Offset + center;
                drawEdge(window, origin, destination, sf::Color::Red);
            }
            halfEdge = halfEdge->next;
            if (halfEdge == start)
                break;
        }
    }
}

template<typename T>
void drawTriangulation(sf::RenderWindow& window, Diagram<T>& diagram, Triangulation& triangulation)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto origin = diagram.getSite(i)->point;
        for (const auto& j : triangulation.getNeighbors(i))
        {
            auto destination = diagram.getSite(j)->point;
            drawEdge(window, origin, destination, sf::Color::Green);
        }
    }
}

// Generating the diagram

template<typename T>
Diagram<T> generateDiagram(const std::vector<Vector2<T>>& points)
{
    // Construct diagram
    auto algorithm = FortuneAlgorithm<T>(points);
    auto start = std::chrono::steady_clock::now();
    algorithm.construct();
    auto duration = std::chrono::steady_clock::now() - start;
    std::cout << "construction: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';

    // Bound the diagram
    start = std::chrono::steady_clock::now();
    algorithm.bound(Box<T>{-0.05, -0.05, 1.05, 1.05}); // Take the bounding box slightly bigger than the intersection box
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "bounding: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';
    auto diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    start = std::chrono::steady_clock::now();
    diagram.intersect(Box<T>{0.0, 0.0, 1.0, 1.0});
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "intersection: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';

    return diagram;
}

int main()
{
    auto nbPoints = 100;
    auto diagram = generateDiagram(generatePoints<Float>(nbPoints));
    auto triangulation = diagram.computeTriangulation();

    // Display the diagram
    auto settings = sf::ContextSettings();
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WindowWidth, WindowHeight), "MyGAL", sf::Style::Default, settings); // Can use auto only in C++17
    window.setView(sf::View(sf::FloatRect(-0.1f, -0.1f, 1.2f, 1.2f)));
    window.setFramerateLimit(60);
    auto showTriangulation = false;

    while (window.isOpen())
    {
        auto event = sf::Event();
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::KeyReleased)
            {
                if (event.key.code == sf::Keyboard::Key::N)
                {
                    diagram = generateDiagram(generatePoints<Float>(nbPoints));
                    triangulation = diagram.computeTriangulation();
                }
                else if (event.key.code == sf::Keyboard::Key::R)
                {
                    diagram = generateDiagram(diagram.computeLloydRelaxation());
                    triangulation = diagram.computeTriangulation();
                }
                else if (event.key.code == sf::Keyboard::Key::T)
                    showTriangulation = !showTriangulation;
            }
        }

        window.clear(sf::Color::Black);

        if (!showTriangulation)
            drawDiagram(window, diagram);
        drawPoints(window, diagram);
        if (showTriangulation)
            drawTriangulation(window, diagram, triangulation);

        window.display();
    }

    return 0;
}
