#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

[[noreturn]] inline void thread_pool_contract_fail(
  const char *file,
  int line,
  const char *expression,
  const char *message)
{
  std::fprintf(
    stderr,
    "thread_pool contract violation at %s:%d\n"
    "  expression: %s\n"
    "  message: %s\n",
    file,
    line,
    expression,
    message);
  std::abort();
}

#define THREAD_POOL_CHECK(expr, message)                             \
  do                                                                 \
  {                                                                  \
    if (!(expr))                                                     \
    {                                                                \
      thread_pool_contract_fail(__FILE__, __LINE__, #expr, message); \
    }                                                                \
  } while (false)

class ThreadPool
{
private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> jobs_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = false;

  void worker_loop();
  void assert_locked_invariants() const;

public:
  explicit ThreadPool(std::size_t thread_count);
  ~ThreadPool();

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;
  ThreadPool(ThreadPool &&) = delete;
  ThreadPool &operator=(ThreadPool &&) = delete;

  void submit(std::function<void()> job);

  template <typename Fn>
  void parallel_for(std::size_t item_count, std::size_t grain_size, Fn&& fn);
};

template <typename Fn>
void ThreadPool::parallel_for(std::size_t item_count, std::size_t grain_size, Fn&& fn)
{
  if (item_count == 0)
  {
    return;
  }

  THREAD_POOL_CHECK(grain_size > 0, "parallel_for() requires a non-zero grain size");

  const std::size_t task_count = (item_count + grain_size - 1) / grain_size;

  if (task_count == 1)
  {
    if constexpr (std::is_invocable_v<Fn&, std::size_t, std::size_t, std::size_t>)
    {
      fn(0, item_count, 0);
    }
    else
    {
      fn(0, item_count);
    }

    return;
  }

  struct SharedState
  {
    std::atomic<std::size_t> remaining{0};
    std::mutex mutex;
    std::condition_variable cv;
    std::exception_ptr first_exception;
  };

  auto state = std::make_shared<SharedState>();
  state->remaining.store(task_count, std::memory_order_release);

  for (std::size_t task_index = 0; task_index < task_count; ++task_index)
  {
    const std::size_t begin = task_index * grain_size;
    const std::size_t end = begin + grain_size < item_count
      ? begin + grain_size
      : item_count;

    submit([state, task_index, begin, end, &fn] {
      try
      {
        if constexpr (std::is_invocable_v<Fn&, std::size_t, std::size_t, std::size_t>)
        {
          fn(begin, end, task_index);
        }
        else
        {
          fn(begin, end);
        }
      }
      catch (...)
      {
        std::lock_guard<std::mutex> lock(state->mutex);

        if (!state->first_exception)
        {
          state->first_exception = std::current_exception();
        }
      }

      const std::size_t previous =
        state->remaining.fetch_sub(1, std::memory_order_acq_rel);

      THREAD_POOL_CHECK(previous > 0, "parallel_for() completion counter underflow");

      if (previous == 1)
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->cv.notify_one();
      }
    });
  }

  {
    std::unique_lock<std::mutex> lock(state->mutex);
    state->cv.wait(lock, [&state] {
      return state->remaining.load(std::memory_order_acquire) == 0;
    });

    if (state->first_exception)
    {
      std::rethrow_exception(state->first_exception);
    }
  }
}
