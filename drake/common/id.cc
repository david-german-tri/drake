#include "drake/common/id.h"

#include <atomic>

namespace drake {

Id MakeId() {
  static std::atomic<std::int64_t>* last_index = new std::atomic<std::int64_t>(0);
  return Id{(*last_index)++};
}

}  // namespace
