#include <gtest/gtest.h>

#include "thread_pool.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

TEST(ThreadPoolBasic, ExecutesSubmittedJobs)
{
  std::atomic<int> counter{0};

  {
    ThreadPool pool(4);

    for (int i = 0; i < 20; ++i)
    {
      pool.submit([&counter] {
        counter.fetch_add(1, std::memory_order_relaxed);
      });
    }
  }

  EXPECT_EQ(counter.load(std::memory_order_relaxed), 20);
}

TEST(ThreadPoolBasic, DestructorWaitsForPendingJobsToFinish)
{
  std::atomic<int> counter{0};

  {
    ThreadPool pool(4);

    for (int i = 0; i < 50; ++i)
    {
      pool.submit([&counter] {
        counter.fetch_add(1, std::memory_order_relaxed);
      });
    }
  }

  EXPECT_EQ(counter.load(std::memory_order_relaxed), 50);
}

TEST(ThreadPoolBasic, JobsCanNotifyCompletion)
{
  ThreadPool pool(2);

  std::mutex mutex;
  std::condition_variable cv;
  int completed = 0;
  constexpr int target = 10;

  for (int i = 0; i < target; ++i)
  {
    pool.submit([&] {
      std::lock_guard<std::mutex> lock(mutex);
      ++completed;
      cv.notify_one();
    });
  }

  std::unique_lock<std::mutex> lock(mutex);
  const bool finished = cv.wait_for(lock, std::chrono::seconds(2), [&] {
    return completed == target;
  });

  EXPECT_TRUE(finished);
  EXPECT_EQ(completed, target);
}

TEST(ThreadPoolBasic, MultipleWorkersCanMakeProgress)
{
  std::atomic<int> started{0};
  std::atomic<int> finished{0};

  {
    ThreadPool pool(4);

    for (int i = 0; i < 8; ++i)
    {
      pool.submit([&] {
        started.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        finished.fetch_add(1, std::memory_order_relaxed);
      });
    }
  }

  EXPECT_EQ(started.load(std::memory_order_relaxed), 8);
  EXPECT_EQ(finished.load(std::memory_order_relaxed), 8);
}

TEST(ThreadPoolBasic, ParallelForVisitsEveryItem)
{
  ThreadPool pool(4);
  std::vector<std::atomic<int>> visits(100);

  pool.parallel_for(100, 7, [&](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i)
    {
      visits[i].fetch_add(1, std::memory_order_relaxed);
    }
  });

  for (const auto& visit : visits)
  {
    EXPECT_EQ(visit.load(std::memory_order_relaxed), 1);
  }
}

TEST(ThreadPoolBasic, ParallelForProvidesStableTaskIndices)
{
  ThreadPool pool(4);
  std::vector<std::atomic<int>> task_visits(4);

  pool.parallel_for(16, 4, [&](std::size_t begin, std::size_t end, std::size_t task_index) {
    EXPECT_LT(task_index, task_visits.size());
    EXPECT_EQ(end - begin, 4u);
    task_visits[task_index].fetch_add(1, std::memory_order_relaxed);
  });

  for (const auto& task_visit : task_visits)
  {
    EXPECT_EQ(task_visit.load(std::memory_order_relaxed), 1);
  }
}

TEST(ThreadPoolBasic, ParallelForPropagatesFirstException)
{
  ThreadPool pool(4);

  EXPECT_THROW(
    pool.parallel_for(32, 4, [](std::size_t begin, std::size_t) {
      if (begin == 8)
      {
        throw std::runtime_error("boom");
      }
    }),
    std::runtime_error);
}
