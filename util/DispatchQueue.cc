// Author Phillip Johnston
// Licensed under CC0 1.0 Universal
// https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/dispatch.cpp
// https://embeddedartistry.com/blog/2017/2/1/dispatch-queues?rq=dispatch

#include <stdio.h>
#include <thread>
#include "DispatchQueue.hh"

namespace sta {

DispatchQueue::DispatchQueue(size_t thread_count)
{
  for (size_t i = 0; i < thread_count; i++)
    workers_.emplace_back(new Worker{*this, i});
}

DispatchQueue::~DispatchQueue()
{
  terminateThreads();
}

void
DispatchQueue::terminateThreads()
{
  // Signal to dispatch threads that it's time to wrap up
  quit_ = true;
  for (auto& worker : workers_) {
    worker->cv.notify_one();
  }

  // Wait for threads to finish before we exit
  for (auto& worker : workers_) {
    if (worker->thread.joinable()) {
      worker->thread.join();
    }
  }
  workers_.clear();
}

void
DispatchQueue::setThreadCount(size_t thread_count)
{
  terminateThreads();

  for (size_t i = 0; i < thread_count; i++)
    workers_.emplace_back(new Worker{*this, i});
  next_worker_ = 0;
}

void
DispatchQueue::dispatch(const fp_t& op)
{
  {
    std::unique_lock lock(workers_[next_worker_]->mtx);
    workers_[next_worker_]->queue.push_back(op);
    pending_count_++;
  }
  workers_[next_worker_]->cv.notify_one();
  next_worker_ = (next_worker_ + 1) % workers_.size();
}

void
DispatchQueue::dispatch(fp_t&& op)
{
  {
    std::unique_lock lock(workers_[next_worker_]->mtx);
    workers_[next_worker_]->queue.push_back(std::move(op));
    pending_count_++;
  }
  workers_[next_worker_]->cv.notify_one();
  next_worker_ = (next_worker_ + 1) % workers_.size();
}

void
DispatchQueue::finishTasks()
{
  while (pending_count_.load(std::memory_order_acquire) != 0)
    std::this_thread::yield();
}

void
DispatchQueue::Worker::dispatch_thread_handler(DispatchQueue& pool, size_t thread_id)
{
  do {
    std::unique_lock<std::mutex> lock(mtx);
    // Wait until we have data or a quit signal
    cv.wait(lock, [this, &pool] { return !queue.empty() || pool.quit_; });

    if (!pool.quit_) {
      auto q = std::move(queue);
      lock.unlock();

      for (auto& job : q) {
        job(thread_id);
      }
      pool.pending_count_ -= q.size();
    }
  } while (!pool.quit_);
}

} // namespace
