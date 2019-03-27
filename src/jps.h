// Copyright (c) 2018 joydeepb@cs.umass.edu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cstdint>
#include <unordered_map>
#include <unordered_set>

#include "eigen3/Eigen/Dense"
#include "CImg.h"
#include "imagemap.h"
#ifndef JPS_H
#define JPS_H

namespace astarplanner {

struct NodeHash;

typedef Eigen::Vector2i Node;
typedef ImageMap Map;
// typedef ObstacleMap Map;
typedef std::vector<Node> Path;
typedef std::unordered_map<Node, Node, NodeHash> NodeMap;

// Hashing function to optimize unordered maps with Nodes as keys.
struct NodeHash {
  NodeHash(size_t stride) : kStride(stride) {}
  size_t operator()(const Node& n) const {
    return (n.x() + n.y() * kStride);
  }
  const size_t kStride;
};

// Struct to keep track of node priorities in A*, with tie-breaking to
// prefer expansion of nodes farther along the path.
struct AStarPriority {
  AStarPriority() {}

  AStarPriority(float g, float h) : g(g), h(h) {}

  // Returns true iff this has lower priority than the other.
  // Note that higher cost => lower priority.
  bool operator<(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has lower priority if its total cost is higher.
    if (f > f_other + kEpsilon) return true;
    if (f < f_other - kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has lower priority if its g-value is lower.
    return (g < other.g);
  }

  // Returns true iff this has higher priority than the other.
  // Note that lower cost => higher priority.
  bool operator>(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has higher priority if its total cost is lower.
    if (f < f_other - kEpsilon) return true;
    if (f > f_other + kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has higher priority if its g-value is higher.
    return (g > other.g);
  }

  // Epsilon for float comparisons.
  static constexpr float kEpsilon = 1e-3;
  // Cost to go: cost from start to this node.
  float g;
  // Heuristic: estimated cost from this node to the goal.
  float h;
};

class AStarPlanner {
  static constexpr int kMaxNeighbors = 8;
 public:
  explicit AStarPlanner(int max_map_width) :
      kStride(max_map_width),
      kMinLookupSize(max_map_width),
      hash_(kStride),
      parent_map_(kMinLookupSize, hash_),
      g_values_(kMinLookupSize, hash_),
      closed_set_(kMinLookupSize, hash_) {}

  int GetJPSNeighbors(const Map& map,
                      const Node& v,
                      const Node& goal,
                      Node neighbors[kMaxNeighbors]);

  int GetImmediateNeighbors(const Map& map,
                            const Node& v,
                            const Node& goal,
                            Node neighbors[kMaxNeighbors]);

  int GetNeighbors(const Map& map,
                 const Node& v,
                 const Node& goal,
                 Node neighbors[kMaxNeighbors]);

  void Visualize(const Map& map,
                 const Node& start,
                 const Node& goal,
                 const Node& current,
                 const Node neighbors[kMaxNeighbors],
                 const int num_neighbors,
                 const NodeMap& parent_map);

  template<const int kAxis, const int kDir>
  bool AxisAlignedJump(const Map& map,
                       const Node& goal,
                       const int max_steps,
                       Node* current_ptr);

  template<const int kDiagonalID>
  bool DiagonalJump(const Map& map,
                    const Node& goal,
                    const int max_x_steps,
                    const int max_y_steps,
                    Node* current_ptr);

  // Plan a path from start to goal using A*. Returns true iff a path is found.
  bool Plan(const Map& map,
            const Node& start,
            const Node& goal,
            Path* path);

  // Stride length of the map, used for hashing.
  const int kStride;
  // Minimum number of entries to allocate memory for, in unordered maps.
  const size_t kMinLookupSize;
  // Node hashing functor.
  NodeHash hash_;
  // Parent map to store naviagtion policy.
  NodeMap parent_map_;
  // G-values of nodes in the open and closed list.
  std::unordered_map<Node, float, NodeHash> g_values_;
  // Closed set (nodes with optimal costs).
  std::unordered_set<Node, NodeHash> closed_set_;
};

// Returns true if the position (n.x() + dir[0], n.y() + dir[1]) is unoccupied.
// Does not perform any sanity checking of the coordinates. Caller must ensure
// the resulting point is on the map.
// bool Occupied(const Map& map, const Node& n, const int dir[2]);


// Performs a jump in an axis-aligned direction. Returns true iff a
// valid jump point was found. If a valid jump point is found, its coordinates
// are returned in the current node.
// kAxis = 0 => X-axis.
// kAxis = 1 => Y-axis.
// kDir = 1 => increasing direction.
template<const int kAxis, const int kDir>
bool AStarPlanner::AxisAlignedJump(const Map& map,
                                   const Node& goal,
                                   const int max_steps,
                                   Node* current_ptr) {
  static_assert(kAxis == 0 || kAxis == 1,
      "ERROR: INVALID AXIS TYPE PASSED TO AStarPlanner::AxisAlignedJump");
  static_assert(kDir == -1 || kDir == 1,
      "ERROR: INVALID DIRECTION PASSED TO AStarPlanner::AxisAlignedJump");
  // Lookup table of neighbors to check, and their potential resulting forced
  // neighbors.
  constexpr int kChecksLookup[2][2][2][2] = {
      {
          {{0, -1}, {kDir, -1}},
          {{0, 1}, {kDir, 1}}
      }, {
          {{-1, 0}, {-1, kDir}},
          {{1, 0}, {1, kDir}}
      }
  };

  // Coordinates of the natural neighbor:
  // When kAxis is 0 (X-axis), it will be (1,0).
  // When kAxis is 1 (Y-axis), it will be (0,1).
  constexpr int kNaturalNeighbor[2] = {
      kDir * (1 - kAxis),
      kDir * kAxis};

  Node& current = *current_ptr;
  int& coordinate = current[kAxis];
  // Loop to increase up to the max.
  for (int i = 0; i <= max_steps; ++i) {
    // If the goal is reached, it is a jump-point.
    if (current == goal) return true;
    // If the natural neighbor is occupied, then this jump is not useful.
    const Node natural_neighbor =
        current + Node(kNaturalNeighbor[0], kNaturalNeighbor[1]);
    if (map.Occupied(natural_neighbor)) return false;
    // Iterate over all the neighbors to be checked.
    for (auto check : kChecksLookup[kAxis]) {
      const Node n1(check[0][0], check[0][1]);
      const Node n2(check[1][0], check[1][1]);
      if (map.Occupied(current + n1) &&
          !map.Occupied(current + n2)) {
        return true;
      }
    }
    if(i < max_steps) coordinate += kDir;
  }
  // Traversed the maximum number of steps, no jump point found.
  return false;
}

// Performs a jump in a diagonal direction. Returns true iff a
// valid jump point was found. If a valid jump point is found, its coordinates
// are returned in the current node.
// kDiagonalID = 0 => (1,-1)
// kDiagonalID = 1 => (-1,-1)
// kDiagonalID = 2 => (-1,1)
// kDiagonalID = 3 => (1,1)
template<const int kDiagonalID>
bool AStarPlanner::DiagonalJump(const Map& map,
                                const Node& goal,
                                const int max_x_steps,
                                const int max_y_steps,
                                Node* current_ptr) {
  static_assert(
      kDiagonalID == 0 ||
      kDiagonalID == 1 ||
      kDiagonalID == 2 ||
      kDiagonalID == 3,
      "ERROR: INVALID DIAGONAL ID PASSED TO AStarPlanner::DiagonalJump");
  static const bool kDebug = false;
  // Lookup table of axes and directions of search for each diagonal type.
  constexpr int kAxesDirsLookup[4][2] = {
      {  // kDiagonalID = 0 (1, -1)
          1,  // +X direction
          -1,  // -Y direction
      },
      {  // kDiagonalID = 1 (-1, -1)
          -1,  // -X direction
          -1,  // -Y direction
      },
      {  // kDiagonalID = 2 (-1, 1)
          -1,  // -X direction
          1,  // +Y direction
      },
      {  // kDiagonalID = 3 (1, 1)
          1,  // +X direction
          1,  // +Y direction
      },
  };
  constexpr int kStepLookup[4][2] = {
      {1, -1},
      {-1, -1},
      {-1, 1},
      {1, 1},
  };
  Node& current = *current_ptr;
  const Node step(kStepLookup[kDiagonalID][0], kStepLookup[kDiagonalID][1]);
  const int max_steps = std::min(max_x_steps, max_y_steps);
  if (kDebug) {
    printf("Diagonal %d JPS start: %d,%d max=%d,%d\n",
           kDiagonalID, current.x(), current.y(), max_x_steps, max_y_steps);
  }
  for (int i = 0; i <= max_steps; ++i) {
    // Check if we've reached the goal.
    if (current == goal) return true;
    if (map.Occupied(current)) return false;
    Node n = current;
    if (kDebug) printf("%d, %d\n", current.x(), current.y());
    // Check if the x-axis aligned horizontal is clear.
    if (AxisAlignedJump<0, kAxesDirsLookup[kDiagonalID][0]>(
        map, goal, max_x_steps - i, &n)) {
      return true;
    }
    n = current;
    // Check if the y-axis aligned horizontal is clear.
    if (AxisAlignedJump<1, kAxesDirsLookup[kDiagonalID][1]>(
        map, goal, max_y_steps - i, &n)) {
      return true;
    }
    if (i < max_steps) current += step;
  }
  // Traversed the maximum number of steps, no jump point found.
  return false;
}


}  // namespace astarplanner

#endif  // JPS_H
