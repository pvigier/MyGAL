# MyGAL

MyGAL is a computational geometry algorithms library.

## Features

For the moment, the library is essentially based on [my implementation of Fortune's algorithm](https://github.com/pvigier/FortuneAlgorithm). It includes:

* [Voronoi diagram](https://en.wikipedia.org/wiki/Voronoi_diagram)
* [Lloyd's relaxation](https://en.wikipedia.org/wiki/Lloyd%27s_algorithm)
* [Delaunay triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation)

MyGAL is:

* easy to use
* easy to install (header-only)
* fast
* well documented
* multi-precision

## Getting started

To get started, you just have to add the `include` folder of MyGAL to the include directories of your project. Then, you just have to add this header to start using MyGAL:

```cpp
#include <MyGAL/FortuneAlgorithm.h>
```

To use the Fortune's algorithm to generate a Voronoi diagram, do the following: 

```cpp
auto points = std::vector<Vector2<double>>{{0.354, 0.678}, {0.632, 0.189}, {0.842, 942}}; // Generate some points
auto algorithm = FortuneAlgorithm<double>(points); // Initialize an instance of Fortune's algorithm
algorithm.construct(); // Construct the diagram
```

The template parameter corresponds to the floating point type used in computations. `double` is the recommended one.

The output diagram is unbounded but most of the time you will want a bounded one. To do that, we will compute the intersection between the diagram and a box. In MyGAL, we do that in two steps:

```cpp
algorithm.bound(Box<double>{-0.05, -0.05, 1.05, 1.05}); // Bound the diagram
auto diagram = algorithm.getDiagram(); // Get the constructed diagram
diagram.intersect(Box<double>{0.0, 0.0, 1.0, 1.0}); // Compute the intersection between the diagram and a box
```

Firstly, we bound the diagram then we compute the intersection. The two steps are due to technical details, you can read [this article](https://pvigier.github.io/2018/11/18/fortune-algorithm-details.html) if you want to know more. It is advised to use a box slightly bigger for the bounded step than the one for the intersection step. Otherwise you might face numerical issues.

Then, you can obtain a Delaunay triangulation:

```cpp
auto triangulation = diagram.computeTriangulation();
```

Or you can apply Lloyd's algorithm:

```cpp
auto relaxedPoints = diagram.computeLloydRelaxation()
```

## Example

If you want to build the example, you can use the cmake file present in the `example` folder. You will need [SFML](https://www.sfml-dev.org/).

The controls of the example are:

* `N`: to generate new random points
* `R`: to apply the Lloyd's algorithm
* `T`: to show the Delaunay triangulation

## Known issues

* If several points are aligned horizontally (same y-coordinate), the diagram may be incorrect.

## Documentation

The documentation is available online [here](https://pvigier.github.io/docs/mygal/).

If you want to build the documentation, you have to have [Doxygen](http://www.doxygen.nl/) installed. Then you just have to execute the `doxygen` command in the `doc` folder.

To know more about the implementation you can read some [articles](https://pvigier.github.io/tag/geometry) on my blog.

## License

Distributed under the [GNU Lesser GENERAL PUBLIC LICENSE version 3](https://www.gnu.org/licenses/lgpl-3.0.en.html)

