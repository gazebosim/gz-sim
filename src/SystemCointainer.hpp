/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_SIM_SYSTEM_CONTAINER_HH_
#define GZ_SIM_SYSTEM_CONTAINER_HH_

#include <vector>
#include <cstddef>
#include <functional>

namespace gz
{
  namespace sim
  {
    //////////////////////////////////////////////////
    /// This container implements a simple masked vector.
    /// Using a masked vector for systems ensures that
    /// when a system is deleted, its memory location stays
    /// the same. This is important because there are a number
    /// of places where we refer to systems by raw pointer
    /// values.
    ///
    /// Operation times:
    /// push - O(1)
    /// removeIf - O(n)
    /// iterate - O(n) where n is the max number of systems
    ///  -> TODO(arjoc): It should be possible to use a cache
    ///  to make each iterator increment have O(1) time.
    ///  For now worst case iterator increment is O(n).
    ///  That being said for most cases the increment
    ///  should be O(1). Eitherways, run does not iterate
    ///  through systems but rather ISystem* interfaces.
    ///  The only two places we iterate over this is during
    ///  reset and TotalByEntity in the tests.
    template<typename T>
    class SystemContainer
    {
      //////////////////////////////////////////
      /// Push an item onto the container
      public: void Push(T internal)
      {
        if (freeSpots.size() == 0)
        {
          this->systems.push_back(internal);
          this->occupied.push_back(true);
          return;
        }

        auto freeIdx = freeSpots.back();
        freeSpots.pop_back();
        this->systems[freeIdx] = internal;
        this->occupied[freeIdx] = true;
      }

      //////////////////////////////////////////
      /// Clear the container
      public: void Clear()
      {
        this->systems.clear();
        this->freeSpots.clear();
        this->occupied.clear();
      }

      //////////////////////////////////////////
      /// Number of active systems
      public: std::size_t Size() const
      {
        return this->systems.size() - this->freeSpots.size();
      }

      //////////////////////////////////////////
      /// Remove an item if a condition is met.
      public: void RemoveIf(std::function<bool(const T&)> fn)
      {
        for (std::size_t i = 0; i < systems.size(); i++)
        {
          if (occupied[i] && fn(systems[i]))
          {
            occupied[i] = false;
            freeSpots.push_back(i);
          }
        }
      }

      private: std::vector<T> systems;

      private: std::vector<bool> occupied;

      private: std::vector<std::size_t> freeSpots;

      /// Simulates a linked list.
      /// TODO(arjoc) use this to build a cache.
      //private: std::vector<std::optional<std::size_t>> nextPtr, prevPtr;

      //////////////////////////////////////////
      class iterator : public std::iterator<
                                std::input_iterator_tag, // iterator_category
                                T,                    // value_type
                                long,                    // difference_type
                                T*,             // pointer
                                T&              // reference
                            > {
        std::size_t num;
        SystemContainer<T>* parent;
      public:
        explicit iterator(SystemContainer<T>* parent, std::size_t _num = 0) :
          num(_num), parent(parent)
        {

        }
        iterator& operator++() {
          // O(n) for now
          do {
            ++num;
          } while(num < parent->Size() && !parent->occupied[num]);
          return *this;
        }
        bool operator==(iterator other) const { return num == other.num; }
        bool operator!=(iterator other) const { return !(*this == other); }
        T& operator*() const {
          return parent->systems[num];
        }
      };

      //////////////////////////////////////////
      public: iterator begin()
      {
        return iterator(this);
      }

      //////////////////////////////////////////
      public: iterator end()
      {
        auto lastIdx = this->occupied.size();

        if (lastIdx == 0)
        {
          return iterator(this);
        }

        while(!this->occupied[lastIdx-1] && lastIdx != 0)
        {
          lastIdx--;
        }
        return iterator(this, lastIdx);
      }
    };
  }
}

#endif