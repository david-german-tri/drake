#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// StateSupervector is a concrete class template that implements
/// StateVector by providing a sliced view of a StateVector.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateSupervector : public StateVector<T> {
 public:
  /// Constructs a supervector consisting of all the vectors in
  /// subvectors, which must live at least as long as this supervector.
  explicit StateSupervector(const std::vector<StateVector<T>*>& subvectors)
      : vectors_(subvectors) {
    int sum = 0;
    for (const StateVector<T>* vec : vectors_) {
      sum += vec->size();
      lookup_table_.push_back(sum);
    }
  }

  int size() const override { return lookup_table_.back(); }

  const T GetAtIndex(int index) const override {
    const auto target = GetSubvectorAndOffset(index);
    return target.first->GetAtIndex(target.second);
  }

  void SetAtIndex(int index, const T& value) override {
    const auto target = GetSubvectorAndOffset(index);
    target.first->SetAtIndex(target.second, value);
  }

 private:
  // Given an @p index into the supervector, returns the subvector that
  // contains that index, and its offset within the subvector. This operation
  // is O(log(N)) in the number of subvectors. Throws std::out_of_range for
  // invalid indices.
  //
  // Example: if the lookup table is [1, 4, 9], and @p index is 5, this function
  // returns a pointer to the third of three subvectors, with offset 1, because
  // the element at index 5 in the supervector is at index 1 in that subvector.
  //
  // 0 | 1 2 3 | 4 5 6 7 8
  //               ^ index 5
  std::pair<StateVector<T>*, int> GetSubvectorAndOffset(int index) const {
    if (index >= size() || index < 0) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state supervector of size " +
                              std::to_string(size()));
    }
    // Binary-search the lookup_table_ for the first element that is larger
    // than the specified index.
    auto it = std::upper_bound(lookup_table_.begin(), lookup_table_.end(),
                               index);

    // Use the lookup result to identify the subvector that contains the index.
    const int subvector_id = std::distance(lookup_table_.begin(), it);
    StateVector<T>* subvector = vectors_[subvector_id];

    // The item at index 0 in vectors_[subvector_id] corresponds to index
    // lookup_table_[subvector_id - 1] in the supervector.
    const int start_of_subvector = (subvector_id == 0) ? 0 : *(--it);
    return std::make_pair(subvector, index - start_of_subvector);
  }

  // StateSupervector objects are neither copyable nor moveable.
  StateSupervector(const StateSupervector& other) = delete;
  StateSupervector& operator=(const StateSupervector& other) = delete;
  StateSupervector(StateSupervector&& other) = delete;
  StateSupervector& operator=(StateSupervector&& other) = delete;

  // A list of all the subvectors comprising this supervector.
  std::vector<StateVector<T>*> vectors_;

  // Each value in the lookup table is the cumulative number of vector elements
  // over all subvectors with a less-than-or-equal instance. For example, if
  // the lengths of the subvectors are [1, 3, 5], the lookup table is [1, 4, 9].
  std::vector<int> lookup_table_;
};

}  // namespace systems
}  // namespace drake
