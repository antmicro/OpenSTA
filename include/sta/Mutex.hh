// OpenSTA, Static Timing Analyzer
// Copyright (c) 2024, Parallax Software, Inc.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but SETOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <atomic>
#include <mutex>
#include <shared_mutex>

namespace sta {

struct SharedMutex
{
  std::atomic<unsigned> count = 0;

  void lock_shared()
  {
    unsigned expected = count.load(std::memory_order_acquire);
    do {
      while (expected == std::numeric_limits<unsigned>::max()) {
        expected = count.load(std::memory_order_acquire);
      }
    } while (!count.compare_exchange_weak(expected, expected + 1, std::memory_order_acquire, std::memory_order_relaxed));
  }

  void unlock_shared()
  {
    count.fetch_sub(1, std::memory_order_release);
  }

  void lock()
  {
    unsigned expected = 0;
    while (!count.compare_exchange_weak(expected, std::numeric_limits<unsigned>::max(), std::memory_order_acquire, std::memory_order_relaxed)) {
      expected = 0;
    }
  }

  void unlock()
  {
    count.store(0, std::memory_order_release);
  }
};

using UniqueLock = std::unique_lock<SharedMutex>;
using SharedLock = std::shared_lock<SharedMutex>;

} // namespace
