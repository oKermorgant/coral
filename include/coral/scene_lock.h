#ifndef CORAL_SCENE_LOCK_H
#define CORAL_SCENE_LOCK_H

#define CORAL_WITH_SCENE_LOCK

#ifdef CORAL_WITH_SCENE_LOCK
#include <mutex>
#endif
#include <iostream>

namespace coral
{

/*
struct scene_lock
{
  static inline std::mutex mutex;


  inline scene_lock()
  {
    std::cout << " trying locking mutex...";
    mutex.lock();
    std::cout << " locked." << std::endl;
  }
  inline ~scene_lock()
  {
    std::cout << "  ...unlocking" << std::endl;
    mutex.unlock();
  }

};*/


[[nodiscard]] inline auto scene_lock()
{
#ifdef CORAL_WITH_SCENE_LOCK
  static std::mutex mutex;
  return std::lock_guard(mutex);
#else
  return nullptr;
#endif
}
}

#endif // CORAL_SCENE_LOCK_H
