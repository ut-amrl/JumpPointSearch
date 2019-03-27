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

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "jps.h"
#include "simple_queue.h"
#include "util/timer.h"

using Eigen::Vector2i;
using std::make_pair;
using std::unordered_map;
using std::unordered_set;
using std::vector;

DEFINE_bool(nojps, false, "Disable JPS");
DEFINE_bool(nogui, false, "Disable Visualization");

namespace {
static const uint8_t kBlue[] = {0, 0, 255};
static const uint8_t kRed[] = {255, 0, 0};
static const uint8_t kGreen[] = {0, 255, 0};
static const uint8_t kYellow[] = {255, 255, 0};
static const uint8_t kGrey[] = {128, 128, 128};

}  // namespace

namespace astarplanner {

int AStarPlanner::GetJPSNeighbors(const Map& map,
                    const Node& v,
                    const Node& goal,
                    Node neighbors[kMaxNeighbors]) {
  static const bool kVisualizeJumps = true;
  int num_neighbors = 0;
  const int x_fwd_steps = map.width() - v.x() - 3;
  const int x_rev_steps = v.x() - 2;
  const int y_fwd_steps = map.height() - v.y() - 3;
  const int y_rev_steps = v.y() - 2;
  auto AddNeighbor = [&](const Node& n){
    // printf("Adding neighbor %d,%d\n", n.x(), n.y());
    neighbors[num_neighbors] = n;
    ++num_neighbors;
  };
  auto VisualizeJump = [&](const Node& n) {
    acc_viz_image_.draw_line(v.x(), v.y(), n.x(), n.y(), kGrey);
  };
  {
    // +x jump search.
    Node n = v + Node(1, 0);
    if (AxisAlignedJump<0, 1>(map, goal, x_fwd_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // -x jump search.
    Node n = v + Node(-1, 0);
    if (AxisAlignedJump<0, -1>(map, goal, x_rev_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // +y jump search.
    Node n = v + Node(0, 1);
    if (AxisAlignedJump<1, 1>(map, goal, y_fwd_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // -y jump search.
    Node n = v + Node(0, -1);
    if (AxisAlignedJump<1, -1>(map, goal, y_rev_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // diagonal 0 search.
    Node n = v + Node(1, -1);
    if (DiagonalJump<0>(map, goal, x_fwd_steps, y_rev_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // diagonal 1 search.
    Node n = v + Node(-1, -1);
    if (DiagonalJump<1>(map, goal, x_rev_steps, y_rev_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // diagonal 2 search.
    Node n = v + Node(-1, 1);
    if (DiagonalJump<2>(map, goal, x_rev_steps, y_fwd_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  {
    // diagonal 3 search.
    Node n = v + Node(1, 1);
    if (DiagonalJump<3>(map, goal, x_fwd_steps, y_fwd_steps, &n)) {
      AddNeighbor(n);
    }
    if (!FLAGS_nogui && kVisualizeJumps) VisualizeJump(n);
  }
  return num_neighbors;
}


int AStarPlanner::GetImmediateNeighbors(const Map& map,
                          const Node& v,
                          const Node& goal,
                          Node neighbors[kMaxNeighbors]) {
  static const Node kNeighbors[] = {
      Node(1, 0),
      Node(1, -1),
      Node(0, -1),
      Node(-1, -1),
      Node(-1, 0),
      Node(-1, 1),
      Node(0, 1),
      Node(1, 1),
  };
  int num_neighbors = 0;
  for (const Node& n : kNeighbors) {
    const Node v2 = v + n;
    if (map.ValidNode(v2) && !map.Occupied(v2)) {
      neighbors[num_neighbors] = v2;
      num_neighbors++;
    }
  }
  return num_neighbors;
}

int AStarPlanner::GetNeighbors(const Map& map,
                               const Node& v,
                               const Node& goal,
                               Node neighbors[kMaxNeighbors]) {
  if (FLAGS_nojps) {
    return GetImmediateNeighbors(map, v, goal, neighbors);
  }
  return GetJPSNeighbors(map, v, goal, neighbors);
}


float Heuristic(const Node& v, const Node& goal) {
  const float dx = abs(v.x() - goal.x());
  const float dy = abs(v.y() - goal.y());
  static const float k = sqrt(2.0) - 2.0;
  return (dx + dy + k * fmin(dx, dy));
}


float Heuristic(const Node& v, const Node& prev, const Node& goal) {
  return Heuristic(v, goal);
}

float Dist(const Node& v1, const Node& v2) {
  return ((v1 - v2).cast<float>().norm());
}

// Returns the path to goal, in reverse order.
void AStarPlanner::GetPath(const NodeMap& parent_map,
                           const Node& goal,
                           Path* path_ptr) {
  Path& path = *path_ptr;
  path.clear();
  Node v = goal;
  path.push_back(v);
  while (parent_map.find(v) != parent_map.end()) {
    v = parent_map.at(v);
    path.push_back(v);
  }
}

void AStarPlanner::InitVisualization(const Map& map) {
  display_ = new cimg_library::CImgDisplay(viz_image_, "A* with JPS");
  viz_image_ = cimg_library::CImg<uint8_t>(
      map.width(), map.height(), 1, 3, 0);
  // Draw map
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.Occupied(x, y)) {
        viz_image_(x, y, 0, 0) = 255;
        viz_image_(x, y, 0, 1) = 255;
        viz_image_(x, y, 0, 2) = 255;
      }
    }
  }
  acc_viz_image_ = viz_image_;
}

void AStarPlanner::Visualize(const Map& map,
                             const Node& start,
                             const Node& goal,
                             const Node& current,
                             const NodeMap& parent_map) {
  static double t_last_visualized = 0;
  static const double kMinVizInterval = 0.02;
  if (t_last_visualized > GetMonotonicTime() - kMinVizInterval) return;

  for (const auto& p : parent_map) {
    acc_viz_image_.draw_line(
      p.first.x(), p.first.y(), p.second.x(), p.second.y(), kBlue);
  }
  acc_viz_image_.draw_point(current.x(), current.y(), kYellow);

  viz_image_ = acc_viz_image_;

  viz_image_.draw_circle(current.x(), current.y(), 2, kYellow);
  viz_image_.draw_circle(start.x(), start.y(), 2, kRed);
  viz_image_.draw_circle(goal.x(), goal.y(), 2, kGreen);

  display_->display(viz_image_);
  // if (display_->is_key()) exit(0);
  t_last_visualized = GetMonotonicTime();
}

void AStarPlanner::DrawPath(const Path& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    viz_image_.draw_line(
        path[i].x(),
        path[i].y(),
        path[i + 1].x(),
        path[i + 1].y(),
        kGreen);
  }
  viz_image_.draw_circle(
      path[0].x(),
      path[0].y(),
      3,
      kGreen);
  viz_image_.draw_circle(
      path.back().x(),
      path.back().y(),
      3,
      kRed);
}

bool AStarPlanner::Plan(const Map& map,
                        const Node& start,
                        const Node& goal,
                        Path* path) {
  static const bool kDebug = false;
  if (!map.ValidNode(start)) {
    printf("Invalid start location.\n");
    return false;
  }
  if (!map.ValidNode(goal)) {
    printf("Invalid goal location.\n");
    return false;
  }
  if (map.Occupied(start) || map.Occupied(goal)) {
    printf("No path possible. Start unreachable:%d goal unreachable:%d\n",
           1 - static_cast<int>(map.Occupied(start)),
           1 - static_cast<int>(map.Occupied(goal)));
    return false;
  }
  if (!FLAGS_nogui) InitVisualization(map);
  // Initialize parent map.
  parent_map_.clear();
  // Clear all G values.
  g_values_.clear();
  // Initialize an empty priority queue.
  SimpleQueue<Node, AStarPriority> queue;
  // Add start to priority queue.
  queue.Push(start, AStarPriority(0, Heuristic(start, goal)));
  g_values_[start] = 0;
  // Clear the closed set.
  closed_set_.clear();
  // Allocate a neighbor list, for speed.
  Node neighbors[kMaxNeighbors];

  // ==============================================================
  // Priority queue.
  // ==============================================================
  // Adding a node to the priority queue, with g-value and h-value:
  // queue.Push(node, AStarPriority(g, h));

  // Checking if the queue is empty:
  // if (queue.Empty()) { ... }

  // Get the current node with the highest priority (smallest f = g + h).
  // Node current_node = queue.Pop();

  // ==============================================================
  // Sets.
  // ==============================================================
  // Insert a node into the closed set:
  // closed_set_.insert(node);

  // Check if a node exists in the closed set:
  // if (closed_set_.find(node) == closed_set_.end()) {
  //   // node was not found in the closed set.
  // } else {
  //   // node was found in the closed set.
  // }

  // ==============================================================
  // Parent map and g values are stored using a hash table.
  // ==============================================================
  // If a node does not exist in the hash table, setting its parent or
  // g-value will initialize a new entry in the hash table:
  // g_values_[node] = g;
  // parent_map_[node] = parent_node;

  // If a node already exists in the hash table, you can update its value by
  // setting it to a new value, which looks identical to its creation:
  // g_values_[node] = g;
  // parent_map_[node] = parent_node;

  // Checking if a node exists in the hash table or not:
  // if (g_values_.find(node) == g_values_.end()) {
  //   // Does not exist.
  // } else {
  //   // Exists.
  // }

  // ==============================================================
  // Visualization
  // ==============================================================
  // Visualize the current state of the plan search. Useful to call for every
  // iteration, to visualize the progress of the search.
  // Visualize(map, start, goal, current, parent_map_);

  const double t_start = GetMonotonicTime();


  // While priority queue is non-empty:
  while (!queue.Empty()) {
    // Get the node with the highest priority.
    const auto current = queue.Pop();
    closed_set_.insert(current);
    if (kDebug) {
      printf("\nCurrent = %d,%d (%f)\n",
             current.x(),
             current.y(),
             g_values_[current]);
    }
    // Get all neighbors.
    const int num_neighbors =
        GetNeighbors(map, current, goal, neighbors);
    // Visualization for debugging.
    if (!FLAGS_nogui) {
      Visualize(map, start, goal, current, parent_map_);
    }
    // For all neighbors:
    for (int i = 0; i < num_neighbors; ++i) {
      const Node& next = neighbors[i];
      // If not on closed list:
      if (closed_set_.find(next) == closed_set_.end()) {
        // Compute tentative g value.
        const float g = g_values_[current] + Dist(next, current);
        // Compute heuristic for the next node.
        const float h = Heuristic(next, current, goal);
        if (kDebug) printf("Next: %d,%d (%f,%f)\n", next.x(), next.y(), g, h);
        if (!queue.Exists(next)) {
          // This node does not exist on the queue, add to parent map and queue.
          parent_map_[next] = current;
          g_values_[next] = g;
          queue.Push(next, AStarPriority(g, h));
        } else {
          // The next node is already on the open set, check for relaxation.
          if (g_values_[next] > g) {
            if (kDebug) {
              printf("Update %6d,%6d : %10.3f -> %10.3f\n",
                     next.x(), next.y(),
                     g_values_[next], g);
            }
            // Update parent map.
            parent_map_[next] = current;
            // Update priority and g-value.
            g_values_[next] = g;
            queue.Push(next, AStarPriority(g, h));
          }
        }
      }
    }
    // If goal has a parent:
    if (parent_map_.find(goal) != parent_map_.end()) {
      const double t_end = GetMonotonicTime();
      printf("Path found in %f seconds.\n", t_end - t_start);
      // We're done. Extract path, and return.
      GetPath(parent_map_, goal, path);
      if (!FLAGS_nogui) {
        DrawPath(*path);
        display_->display(viz_image_);
        while (!display_->is_closed() && !display_->is_key()) display_->wait();
      }
      return true;
    }
  }
  const double t_end = GetMonotonicTime();
  printf("No path found, took %f seconds.\n", t_end - t_start);
  if (!FLAGS_nogui) {
    display_->display(viz_image_);
    while (!display_->is_closed() && !display_->is_key()) display_->wait();
  }
  // Priority queue is exhausted, but path not found. No path exists.
  return false;
}

}  // namespace astarplanner
