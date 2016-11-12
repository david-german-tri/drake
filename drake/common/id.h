/// @file
/// Provides unique event identifiers.

#pragma once

#include <cstdint>

namespace drake {

struct Id {
  int64_t in_process_index{0};
};

/// Returns an event identifier, which is process-unique.
/// This method is thread-safe.
/// TODO(david-german-tri): Make this globally unique by including IP address
/// and process ID as well.
Id MakeId();

}  // namespace
