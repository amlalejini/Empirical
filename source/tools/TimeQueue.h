/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2020
 *
 *  @file  TimeQueue.h
 *  @brief A priority queue for timings, always marching forward.
 *  @note Status: ALPHA
 *
 */


#ifndef EMP_TIME_QUEUE_H
#define EMP_TIME_QUEUE_H

#include <limits>
#include <string>
#include <sstream>

#include "../base/assert.h"
#include "../base/vector.h"
#include "vector_utils.h"

namespace emp {

  /// A TimeQueue is used to track when "items" are ready.  Insert() items with the 
  /// time they should be triggered.  Must be at least min_wait in the future.

  template <typename T=size_t>
  class TimeQueue {
  private:
    struct ItemInfo {
      T item;
      double timing;
  
      bool operator<(const ItemInfo & _in) const {
        if (timing == _in.timing) return (item < _in.item);
        return (timing < _in.timing);
      }
    };

    double cur_time = 0.0;               ///< What time are we up to?
    double min_wait = 1.0;               ///< Minimum amount of time for the next event.
    emp::vector<ItemInfo> item_queue;    ///< Sorted events to be triggered.
    emp::vector<ItemInfo> item_buffer;   ///< Unsorted events out of current range.
    size_t pos = 0;                      ///< What position are we up to in the item_queue?

    // Helper function to move more items into the queue.
    bool RefillQueue() {
      emp_assert(pos == item_queue.size());       // Make sure we've used up current queue.
      if (item_buffer.size() == 0) return false;  // No items left!

      item_queue.resize(0);  // Clear out old queue.
      pos = 0;

      // Scan the buffer to determine the earliest time.
      const double first_time = FindMin(item_buffer).timing;

      // Move all items that will trigger in the next "min_wait" timesteps to item_queue.
      double last_time = first_time + min_wait;
      size_t keep_count = 0;
      for (size_t i = 0; i < item_buffer.size(); i++) {
        if (item_buffer[i].timing <= last_time) item_queue.push_back(item_buffer[i]);
        else item_buffer[keep_count++] = item_buffer[i];
      }
      item_buffer.resize(keep_count);

      // Sort the item queue so that it's ready to go.
      emp::Sort(item_queue);

      return true;
    }

    // If the minimum wait changes we need to make sure that the item_queue is still accurate.
    void UpdateMinimum(double new_min) {
      min_wait = new_min;
      double time_limit = cur_time + min_wait;
      size_t shift_pos = 0;
      for (size_t i=pos; i < item_queue.size(); i++) {
        // If the next entry is no longer in range, move it back to buffer and set shift as needed.
        if (item_queue[i].timing > time_limit) item_buffer.push_back(item_queue[i]);

        // Otherwise shift it into position.
        else item_queue[shift_pos++] = item_queue[i];
      }

      pos = 0;
      item_queue.resize(shift_pos);
    }

  public:
    TimeQueue(double _min_wait=std::numeric_limits<double>::max()/2) : min_wait(_min_wait) {
      emp_assert(min_wait > 0.0);
    }

    double GetTime() const { return cur_time; }
    double GetMinWait() const { return min_wait; }
    size_t GetSize() const { return item_queue.size() + item_buffer.size() - pos; }

    void SetTime(double _time) { cur_time = _time; }

    /// Empty the TimeQueue.
    void Clear() {
      item_queue.resize(0);
      item_buffer.resize(0);
      pos = 0;
    }

    /// Empty the TimeQueue and start over at time zero.
    void Reset() {
      Clear();
      cur_time = 0.0;
    }
    /// Add a new item to the TimeQueue.
    void Insert(T in_item, double trigger_time) {
      emp_assert(trigger_time > cur_time, trigger_time, cur_time); // Triggers must be in the future
      if (cur_time + min_wait > trigger_time) UpdateMinimum(trigger_time - cur_time);
      item_buffer.emplace_back( ItemInfo{ in_item, trigger_time } );
    }

    /// Grab the next item from the TimeQueue, but don't remove it.
    T Front() {
      emp_assert(pos <= item_queue.size());        // Pos should never be more than one past end.
      if (pos == item_queue.size()) RefillQueue(); // Move over from buffer if needed.
      emp_assert(item_queue.size() > 0);           // Must have an element to return it!
      return item_queue[pos].item;
    }

    /// Remove and return the next item from the TimeQueue.
    T Next() {
      emp_assert(pos <= item_queue.size(), pos, item_queue.size(),
                 "Pos should never be more than one past end.");
      if (pos == item_queue.size()) RefillQueue();                  // Refill from buffer if needed.
      emp_assert(item_queue.size() > pos, item_queue.size(), pos);  // Must have an element to return!
      cur_time = item_queue[pos].timing;
      return item_queue[pos++].item;
    }

    /// Remove the next item from TimeQueue, setting argument to it; return whether items remain.
    bool Next(T & out_item) {
      emp_assert(pos <= item_queue.size());        // Pos should never be more than one past end.
      if (pos == item_queue.size()) RefillQueue(); // Move over from buffer if needed.
      if (item_queue.size() == 0) return false;    // Return false if no items left.
      cur_time = item_queue[pos].timing;
      out_item = item_queue[pos++].item;
      return true;
    }

    std::string AsString() {
      std::stringstream ss;
      emp::Sort(item_buffer);
      ss << GetSize() << " entries:";
      for (size_t i = pos; i < item_queue.size(); i++) {
        auto x = item_queue[i];
        ss << " " << x.item << "(" << x.timing << ")";
      }
      for (auto x : item_buffer) ss << " " << x.item << "(" << x.timing << ")";
      return ss.str();
    }

  };

}

#endif
