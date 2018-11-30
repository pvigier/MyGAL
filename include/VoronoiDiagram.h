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
#include <vector>
#include <list>
#include <unordered_set>
// My includes
#include "Box.h"

namespace mygal
{

template<typename T>
class FortuneAlgorithm;

template<typename T>
class VoronoiDiagram
{
public:
    struct HalfEdge;
    struct Face;

    struct Site
    {
        std::size_t index;
        Vector2<T> point;
        Face* face;
    };

    struct Vertex
    {
        Vector2<T> point;

    private:
        friend VoronoiDiagram<T>;
        typename std::list<Vertex>::iterator it;
    };

    struct HalfEdge
    {
        Vertex* origin = nullptr;
        Vertex* destination = nullptr;
        HalfEdge* twin = nullptr;
        Face* incidentFace;
        HalfEdge* prev = nullptr;
        HalfEdge* next = nullptr;

    private:
        friend VoronoiDiagram;
        typename std::list<HalfEdge>::iterator it;
    };

    struct Face
    {
        Site* site;
        HalfEdge* outerComponent;
    };

    VoronoiDiagram(const std::vector<Vector2<T>>& points)
    {
        mSites.reserve(points.size());
        mFaces.reserve(points.size());
        for(std::size_t i = 0; i < points.size(); ++i)
        {
            mSites.push_back(VoronoiDiagram::Site{i, points[i], nullptr});
            mFaces.push_back(VoronoiDiagram::Face{&mSites.back(), nullptr});
            mSites.back().face = &mFaces.back();
        }
    }

    // Remove copy operations
    VoronoiDiagram(const VoronoiDiagram&) = delete;

    VoronoiDiagram& operator=(const VoronoiDiagram&) = delete;

    // Move operations
    VoronoiDiagram(VoronoiDiagram&&) = default;

    VoronoiDiagram& operator=(VoronoiDiagram&&) = default;

    // Accessors

    Site* getSite(std::size_t i)
    {
        return &mSites[i];
    }

    std::size_t getNbSites() const
    {
        return mSites.size();
    }

    Face* getFace(std::size_t i)
    {
        return &mFaces[i];
    }

    const std::list<Vertex>& getVertices() const
    {
        return mVertices;
    }

    const std::list<HalfEdge>& getHalfEdges() const
    {
        return mHalfEdges;
    }

    // Intersection with a box

    bool intersect(Box<T> box)
    {
        auto error = false;
        auto processedHalfEdges = std::unordered_set<HalfEdge*>();
        auto verticesToRemove = std::unordered_set<Vertex*>();
        for (const auto& site : mSites)
        {
            auto halfEdge = site.face->outerComponent;
            auto inside = box.contains(halfEdge->origin->point);
            auto outerComponentDirty = !inside;
            auto incomingHalfEdge = static_cast<HalfEdge*>(nullptr); // First half edge coming in the box
            auto outgoingHalfEdge = static_cast<HalfEdge*>(nullptr); // Last half edge going out the box
            auto incomingSide = typename Box<T>::Side{};
            auto outgoingSide = typename Box<T>::Side{};
            do
            {
                auto intersections = std::array<typename Box<T>::Intersection, 2>{};
                auto nbIntersections = box.getIntersections(halfEdge->origin->point, halfEdge->destination->point, intersections);
                auto nextInside = box.contains(halfEdge->destination->point);
                auto nextHalfEdge = halfEdge->next;
                // The two points are outside the box 
                if (!inside && !nextInside)
                {
                    // The edge is outside the box
                    if (nbIntersections == 0)
                    {
                        verticesToRemove.emplace(halfEdge->origin);
                        removeHalfEdge(halfEdge);
                    }
                    // The edge crosses twice the frontiers of the box
                    else if (nbIntersections == 2)
                    {
                        verticesToRemove.emplace(halfEdge->origin);
                        if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                        {
                            halfEdge->origin = halfEdge->twin->destination;
                            halfEdge->destination = halfEdge->twin->origin;
                        }
                        else
                        {
                            halfEdge->origin = createVertex(intersections[0].point);
                            halfEdge->destination = createVertex(intersections[1].point);
                        }
                        if (outgoingHalfEdge != nullptr)
                            link(box, outgoingHalfEdge, outgoingSide, halfEdge, intersections[0].side);
                        if (incomingHalfEdge == nullptr)
                        {
                           incomingHalfEdge = halfEdge;
                           incomingSide = intersections[0].side;
                        }
                        outgoingHalfEdge = halfEdge;
                        outgoingSide = intersections[1].side;
                        processedHalfEdges.emplace(halfEdge);
                    }
                    else
                        error = true;
                }
                // The edge is going outside the box
                else if (inside && !nextInside)
                {
                    if (nbIntersections == 1)
                    {
                        if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                            halfEdge->destination = halfEdge->twin->origin;
                        else
                            halfEdge->destination = createVertex(intersections[0].point);
                        outgoingHalfEdge = halfEdge;
                        outgoingSide = intersections[0].side;
                        processedHalfEdges.emplace(halfEdge);
                    }
                    else
                        error = true;
                }
                // The edge is coming inside the box
                else if (!inside && nextInside)
                {
                    if (nbIntersections == 1)
                    {
                        verticesToRemove.emplace(halfEdge->origin);
                        if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                            halfEdge->origin = halfEdge->twin->destination;
                        else
                            halfEdge->origin = createVertex(intersections[0].point);
                        if (outgoingHalfEdge != nullptr)
                            link(box, outgoingHalfEdge, outgoingSide, halfEdge, intersections[0].side);
                        if (incomingHalfEdge == nullptr)
                        {
                           incomingHalfEdge = halfEdge;
                           incomingSide = intersections[0].side;
                        }
                        processedHalfEdges.emplace(halfEdge);
                    }
                    else
                        error = true;
                }
                halfEdge = nextHalfEdge;
                // Update inside
                inside = nextInside;
            } while (halfEdge != site.face->outerComponent);
            // Link the last and the first half edges inside the box
            if (outerComponentDirty && incomingHalfEdge != nullptr)
                link(box, outgoingHalfEdge, outgoingSide, incomingHalfEdge, incomingSide);
            // Set outer component
            if (outerComponentDirty)
                site.face->outerComponent = incomingHalfEdge;
        }
        // Remove vertices
        for (auto& vertex : verticesToRemove)
            removeVertex(vertex);
        // Return the status
        return !error;
    }

private:
    std::vector<Site> mSites;
    std::vector<Face> mFaces;
    std::list<Vertex> mVertices;
    std::list<HalfEdge> mHalfEdges;

    // Diagram construction

    template<typename>
    friend class FortuneAlgorithm;

    Vertex* createVertex(Vector2<T> point)
    {
        mVertices.emplace_back();
        mVertices.back().point = point;
        mVertices.back().it = std::prev(mVertices.end());
        return &mVertices.back();
    }

    Vertex* createCorner(Box<T> box, typename Box<T>::Side side)
    {
        switch (side)
        {
            case Box<T>::Side::LEFT:
                return createVertex(Vector2<T>(box.left, box.top));
            case Box<T>::Side::BOTTOM:
                return createVertex(Vector2<T>(box.left, box.bottom));
            case Box<T>::Side::RIGHT:
                return createVertex(Vector2<T>(box.right, box.bottom));
            case Box<T>::Side::TOP:
                return createVertex(Vector2<T>(box.right, box.top));
            default:
                return nullptr;
        }
    }

    HalfEdge* createHalfEdge(Face* face)
    {
        mHalfEdges.emplace_back();
        mHalfEdges.back().incidentFace = face;
        mHalfEdges.back().it = std::prev(mHalfEdges.end());
        if(face->outerComponent == nullptr)
            face->outerComponent = &mHalfEdges.back();
        return &mHalfEdges.back();
    }

    // Intersection with a box

    void link(Box<T> box, HalfEdge* start, typename Box<T>::Side startSide, HalfEdge* end, typename Box<T>::Side endSide)
    {
        auto halfEdge = start;
        auto side = static_cast<int>(startSide);
        while (side != static_cast<int>(endSide))
        {
            side = (side + 1) % 4;
            halfEdge->next = createHalfEdge(start->incidentFace);
            halfEdge->next->prev = halfEdge;
            halfEdge->next->origin = halfEdge->destination;
            halfEdge->next->destination = createCorner(box, static_cast<typename Box<T>::Side>(side));
            halfEdge = halfEdge->next;
        }
        halfEdge->next = createHalfEdge(start->incidentFace);
        halfEdge->next->prev = halfEdge;
        end->prev = halfEdge->next;
        halfEdge->next->next = end;
        halfEdge->next->origin = halfEdge->destination;
        halfEdge->next->destination = end->origin;
    }

    void removeVertex(Vertex* vertex)
    {
        mVertices.erase(vertex->it);
    }

    void removeHalfEdge(HalfEdge* halfEdge)
    {
        mHalfEdges.erase(halfEdge->it);
    }
};

}
