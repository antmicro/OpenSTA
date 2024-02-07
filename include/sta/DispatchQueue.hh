// Author Phillip Johnston
// Licensed under CC0 1.0 Universal
// https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/dispatch.cpp
// https://embeddedartistry.com/blog/2017/2/1/dispatch-queues?rq=dispatch

#pragma once

#include <thread>
#include <functional>
#include <vector>
#include <cstdint>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace sta {

class DispatchQueue
{
  typedef std::function<void(int thread)> fp_t;

public:
  DispatchQueue(size_t thread_cnt);
  ~DispatchQueue();
  void setThreadCount(size_t thread_count);
  // Dispatch and copy.
  void dispatch(const fp_t& op);
  // Dispatch and move.
  void dispatch(fp_t&& op);
  void finishTasks();

  // Deleted operations
  DispatchQueue(const DispatchQueue& rhs) = delete;
  DispatchQueue& operator=(const DispatchQueue& rhs) = delete;
  DispatchQueue(DispatchQueue&& rhs) = delete;
  DispatchQueue& operator=(DispatchQueue&& rhs) = delete;

private:
  void terminateThreads();

  struct Worker {
    Worker(DispatchQueue& pool, size_t thread_id) :
      thread([this, &pool, thread_id] { Worker::dispatch_thread_handler(pool, thread_id); }) {}
    void dispatch_thread_handler(DispatchQueue& pool, size_t thread_id);

    std::thread thread;
    std::vector<fp_t> queue = {};
    std::mutex mtx = {};
    std::condition_variable cv = {};
  };

  std::vector<std::unique_ptr<Worker>> workers_;
  std::atomic<size_t> pending_count_ = 0;
  std::atomic<bool> quit_ = false;
  std::size_t next_worker_ = 0;
};

} // namespace
