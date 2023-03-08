#ifndef CORAL_SCENE_LOCK_H
#define CORAL_SCENE_LOCK_H

#define CORAL_WITH_SCENE_LOCK

#ifdef CORAL_WITH_SCENE_LOCK
#include <mutex>
#endif

namespace coral
{

#ifdef CORAL_WITH_SCENE_LOCK
  [[nodiscard]] inline auto coral_lock()
{
  static std::mutex mutex;
  return std::lock_guard(mutex);
}
#else
  inline auto coral_lock() {return nullptr;}
#endif

}

#endif // CORAL_SCENE_LOCK_H
