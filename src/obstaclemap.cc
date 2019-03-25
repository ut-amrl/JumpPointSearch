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

#include <unordered_set>
#include "obstaclemap.h"

namespace astarplanner {

// Construct an empty map of width w and height h.
ObstacleMap::ObstacleMap(int w, int h) :
    w_(w),
    h_(h),
    hash_(w_),
    obstacles_(w_ + h_, hash_) {}

// Load a map from an image.
ObstacleMap::ObstacleMap(const char* filename) :
    map_image_(filename),
    w_(map_image_.width()),
    h_(map_image_.height()),
    hash_(w_),
    obstacles_(w_ + h_, hash_) {}

// Returns true iff the specified node is occupied.
bool ObstacleMap::Occupied(const Node& n) const {
  return (obstacles_.count(n) > 0);
}

// Returns true iff the node at coordinates (x, y) is occupied.
bool ObstacleMap::Occupied(const int x, const int y) const {
  return (Occupied(Node(x, y)));
}

// Returns true iff the specified node is within the map dimensions.
bool ObstacleMap::ValidNode(const Node& n) const {
  if (n.x() < 0 || n.y() < 0 ||
      n.x() >= w_ || n.y() >= h_) {
    return false;
  }
  return true;
}

int ObstacleMap::width() const {
  return w_;
}

int ObstacleMap::height() const {
  return h_;
}

void ObstacleMap::DrawCircle(const Node& n, int r) {
  const int r_sq = r * r;
  for (int x = n.x() - r; x <= n.x() + r; ++x) {
    for (int y = n.y() - r; y <= n.y() + r; ++y) {
      if ((Node(x, y) - n).squaredNorm() <= r_sq) {
        obstacles_.insert(Node(x, y));
      }
    }
  }
}

void ObstacleMap::Set(int x, int y) {
}

}  // namespace astarplanner
