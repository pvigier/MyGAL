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

// My includes
#include "VoronoiDiagram.h"

template<typename T>
class Event;

template<typename T>
struct Arc
{
    enum class Color{RED, BLACK};

    // Hierarchy
    Arc<T>* parent;
    Arc<T>* left;
    Arc<T>* right;
    // Diagram
    typename VoronoiDiagram<T>::Site* site;
    typename VoronoiDiagram<T>::HalfEdge* leftHalfEdge;
    typename VoronoiDiagram<T>::HalfEdge* rightHalfEdge;
    Event<T>* event;
    // Optimizations
    Arc<T>* prev;
    Arc<T>* next;
    // Only for balancing
    Color color;
};

