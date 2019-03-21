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

#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "jps.h"
#include "util/random.h"
#include "util/timer.h"

using astarplanner::Map;
using astarplanner::Node;
using astarplanner::Path;
using astarplanner::AStarPlanner;

static const float kLength = 12000;
static const float kWidth = 9000;
static const float kDiscretization = 77;
static const float kRobotObstRadius = 2.0 * (90 + 10);
static const int kNumRobots = 16;  // Used to simulate 15 robots + ball.

static const int kLengthInt = std::ceil(kLength / kDiscretization);
static const int kWidthInt = std::ceil(kWidth / kDiscretization);
static const int kRobotObstRadiusInt =
    std::ceil(kRobotObstRadius / kDiscretization);

minutebotrandom::Random rand_;

CumulativeFunctionTimer* timer = new CumulativeFunctionTimer("A*");

CumulativeFunctionTimer map_timer("Map Rebuild");
bool kVisualize = false;
bool use_jps_ = true;

void Plan(const Map& map, const Node& start, const Node& goal) {
  CumulativeFunctionTimer::Invocation invoke(timer);
  AStarPlanner planner(map.width(), kVisualize);
  Path path;
  planner.Plan(map, start, goal, use_jps_, &path);
}

void SoccerTest(AStarPlanner* planner) {
  Map map(0, 0);
  auto RandomNode = [&]() {
    return Node(
      rand_.RandomInt(1, kLengthInt - 2), rand_.RandomInt(1, kWidthInt - 2));
  };

  {
    map = Map(kLengthInt, kWidthInt);
    CumulativeFunctionTimer::Invocation invoke(&map_timer);
    for (int i = 0; i < kNumRobots; ++i) {
      const Node r = RandomNode();
      map.DrawCircle(r, kRobotObstRadiusInt);
    }
  }
  {
    const Node start = RandomNode();
    const Node goal = RandomNode();
    CumulativeFunctionTimer::Invocation invoke(timer);
    Path path;
    planner->Plan(map, start, goal, use_jps_, &path);
  }
}

void RunBenchmark() {
  if (true) {
    printf("Running soccer benchmark\n");
    kVisualize = false;
    const int N = 10000;
    timer = new CumulativeFunctionTimer("A* with JPS");
    use_jps_ = true;
    AStarPlanner planner(kLengthInt, kVisualize);
    for (int i = 0; i < N; ++i) {
      // printf("%d\n", i);
      SoccerTest(&planner);
      if(kVisualize) Sleep(0.5);
    }
    delete timer;
    timer = new CumulativeFunctionTimer("A* without JPS");
    use_jps_ = false;
    for (int i = 0; i < N; ++i) {
      // printf("%d\n", i);
      SoccerTest(&planner);
      if(kVisualize) Sleep(0.5);
    }
    delete timer;
  }
  if (false) {
    printf("Running benchmark...\n");
    static const int kRepeat = 1;
    Map map("map4.png");
    Node start(20, 20);
    Node goal(300, 300);
    AStarPlanner planner(map.width(), false);
    Path path;
    CumulativeFunctionTimer timer("A*");
    for (int i = 0; i < kRepeat; ++i) {
      CumulativeFunctionTimer::Invocation invoke(&timer);
      planner.Plan(map, start, goal, false, &path);
    }
  }
}

int main(int num_args, char* args[]) {
  if (num_args > 1) {
    printf("Planning on map image '%s'\n", args[1]);
    kVisualize = true;
    Map map(args[1]);
    Node start(15, 15);
    Node goal(map.width() - 10, map.height() / 2);
    if (num_args > 3) {
      goal.x() = atoi(args[2]);
      goal.y() = atoi(args[3]);
    }
    if (num_args > 5) {
      start.x() = atoi(args[4]);
      start.y() = atoi(args[5]);
    }
    printf("Start: %d,%d Goal: %d,%d\n",
           start.x(), start.y(),
           goal.x(), goal.y());
    Plan(map, start, goal);
  } else  {
    RunBenchmark();
  }
  return 0;
}
