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
#include <deque>

#ifndef MUTABLE_QUEUE
#define MUTABLE_QUEUE

using std::deque;
using std::pair;

template<class Value, class Priority, class HashFunction>
class MutableQueue {
  public:
  // Insert a new value, with the specified priority.
  void Push(const Value& v, const Priority& p);
  // Retreive the vaue with the highest priority.
  Value Pop();
  // Returns true iff the priority queue is empty.
  bool Empty();
  // Returns true iff the provided value is already on the queue.
  bool Exists(const Value& v);

  private:
  // Needs two internal data structures: 
  // 1. A sorted list, sorted by priority, with associated pointers to values.
  //    This is required for O(1) Pop(), and O(log(n)) Push(). 
  // 2. An unordered map of the values, with associated indi.
  //    This is required for ~O(1) Exists() and 
  deque<pair<Value*, Priority> > values_;
};

#endif  // MUTABLE_QUEUE
