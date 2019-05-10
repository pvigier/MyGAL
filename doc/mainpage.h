/**
 * \mainpage
 *
 * \section welcome Welcome
 *
 * Welcome to the official MyGAL documentation. Here you will find a detailed
 * view of all the MyGAL classes.
 *
 * \section example Example
 *
 * Here is a short example, to show you how simple it is to use MyGAL:
 *
 * \code
 *
 * #include <MyGAL/FortuneAlgorithm.h>
 *
 * using namespace mygal;
 *
 * int main()
 * {
 *     // Generate some points
 *     auto points = std::vector<Vector2<double>>
 *     {
 *         {0.354, 0.678},
 *         {0.632, 0.189},
 *         {0.842, 0.942}
 *     };
 *
 *     // Initialize an instance of Fortune's algorithm
 *     auto algorithm = FortuneAlgorithm<double>(points);
 *     // Construct the diagram
 *     algorithm.construct();
 *     // Bound the diagram
 *     algorithm.bound(Box<double>{-0.05, -0.05, 1.05, 1.05});
 *     // Get the constructed diagram
 *     auto diagram = algorithm.getDiagram();
 *     // Compute the intersection between the diagram and a box
 *     diagram.intersect(Box<double>{0.0, 0.0, 1.0, 1.0});
 *
 *     return 0;
 * }
 *
 * \endcode
 */
