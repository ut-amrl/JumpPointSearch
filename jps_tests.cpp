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

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "jps.h"

using astarplanner::Map;
using astarplanner::Node;
using astarplanner::Path;
using astarplanner::AStarPlanner;
using astarplanner::AxisAlignedJump;
using astarplanner::DiagonalJump;

// Map notation for tests: "." = free, "*" = occupied, S = start, E = end.

TEST(JumpPoint, NoJumps) {
  const Node goal(10, 10);
  {
    // Empty map, jump in +x direction.
    Map map(5, 5, 1, 1, 0);
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // Obstacle in the way, jump in +x direction.
    Map map(5, 5, 1, 1, 0);
    map(3, 2) = 1;
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // Obstacle in the way, jump in +x direction.
    Map map(5, 5, 1, 1, 0);
    map(4, 2) = 1;
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // Empty map, jump in +y direction.
    Map map(5, 5, 1, 1, 0);
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // Obstacle in the way, jump in +y direction.
    Map map(5, 5, 1, 1, 0);
    map(2, 3) = 1;
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // Obstacle in the way, jump in +y direction.
    Map map(5, 5, 1, 1, 0);
    map(2, 4) = 1;
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_FALSE(result);
  }
}


TEST(JumpPoint, JumpForward) {
  const Node goal(10, 10);
  // X-axis
  {
    // .....
    // *....
    // .S.E.
    // .....
    // .....
    Map map(5, 5, 1, 1, 0);
    map(0, 1) = 1;
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 1);
    EXPECT_EQ(n.y(), 2);
  }
  {
    Map map(5, 5, 1, 1, 0);
    map(1, 1) = 1;
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 1);
    EXPECT_EQ(n.y(), 2);
  }
  {
    Map map(5, 5, 1, 1, 0);
    map(0, 3) = 1;
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 1);
    EXPECT_EQ(n.y(), 2);
  }
  {
    Map map(5, 5, 1, 1, 0);
    map(1, 3) = 1;
    Node n(1,2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 1);
    EXPECT_EQ(n.y(), 2);
  }
  {
    // .....
    // ...*.
    // .S.E.
    // .....
    // .....
    Map map(5, 5, 1, 1, 0);
    map(3, 1) = 1;
    Node n(1, 2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 3);
    EXPECT_EQ(n.y(), 2);
  }

  // Y-axis
  {
    Map map(5, 5, 1, 1, 0);
    map(1, 0) = 1;
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 2);
    EXPECT_EQ(n.y(), 1);
  }
  {
    Map map(5, 5, 1, 1, 0);
    map(3, 0) = 1;
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 2);
    EXPECT_EQ(n.y(), 1);
  }
  {
    Map map(5, 5, 1, 1, 0);
    map(1, 1) = 1;
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 2);
    EXPECT_EQ(n.y(), 1);
  }
  {
    Map map(5, 5, 1, 1, 0);
    map(3, 1) = 1;
    Node n(2,1);
    const bool result = AxisAlignedJump<1, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 2);
    EXPECT_EQ(n.y(), 1);
  }
}

TEST(JumpPoint, JumpBackward) {
  const Node goal(10, 10);
  // X-axis
  {
    // .....
    // .*...
    // .E.S.
    // .....
    // .....
    Map map(5, 5, 1, 1, 0);
    map(1, 1) = 1;
    Node n(3, 2);
    const bool result = AxisAlignedJump<0, -1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 1);
    EXPECT_EQ(n.y(), 2);
  }

  // Y-axis
  {
    // .....
    // ..E*.
    // .....
    // ..S..
    // .....
    Map map(5, 5, 1, 1, 0);
    map(3, 1) = 1;
    Node n(2, 3);
    const bool result = AxisAlignedJump<1, -1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 2);
    EXPECT_EQ(n.y(), 1);
  }
}

TEST(JumpPoint, DiagonalNoJump) {
  const Node goal(10, 10);
  const Map map(5, 5, 1, 1, 0);
  {
    // +x,-y diagonal
    Node n(1, 3);
    const bool result = DiagonalJump<0>(map, goal, 2, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // -x,-y diagonal
    Node n(3, 3);
    const bool result = DiagonalJump<1>(map, goal, 2, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // -x,+y diagonal
    Node n(3, 1);
    const bool result = DiagonalJump<2>(map, goal, 2, 2, &n);
    EXPECT_FALSE(result);
  }
  {
    // +x,+y diagonal
    Node n(1, 1);
    const bool result = DiagonalJump<3>(map, goal, 2, 2, &n);
    EXPECT_FALSE(result);
  }
}


TEST(JumpPoint, DiagonalJump) {
  const Node goal(10, 10);
  // X-axis
  {
    // .....
    // ..E*.
    // .S...
    // .....
    // .....
    Map map(5, 5, 1, 1, 0);
    map(3, 1) = 1;
    Node n(1, 2);
    const bool result = DiagonalJump<0>(map, goal, 2, 1, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 1);
    EXPECT_EQ(n.y(), 2);
  }
}

TEST(JumpPoint, GoalJump) {
  const Node goal(3, 2);
  {
    // .....
    // .....
    // .S.E.
    // .....
    // .....
    Map map(5, 5, 1, 1, 0);
    Node n(1, 2);
    const bool result = AxisAlignedJump<0, 1>(map, goal, 2, &n);
    EXPECT_TRUE(result);
    EXPECT_EQ(n.x(), 3);
    EXPECT_EQ(n.y(), 2);
  }
}
int main(int argc, char** argv) {
  printf("WARNING: TODO: Check goal during jumps\n");
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
