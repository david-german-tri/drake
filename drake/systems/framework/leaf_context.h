#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// %LeafContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a leaf System. Specifically, a
/// %LeafContext contains and owns the State, and also contains (but does not
/// own) pointers to the value sources for Inputs, as well as the simulation
/// time and the cache.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class LeafContext : public Context<T> {
 public:
  LeafContext(int num_input_ports, int num_output_ports)
      : inputs_(num_input_ports),
        num_output_ports_(num_output_ports){
    Context<T>::BuildCacheTickets();
  }

  virtual ~LeafContext() {}

  int get_num_input_ports() const override {
    return static_cast<int>(inputs_.size());
  }

  int get_num_output_ports() const override {
    return num_output_ports_;
  }

  const State<T>& get_state() const override { return state_; }

  // =========================================================================
  // Accessors and Mutators for Parameters.
  // TODO(david-german-tri): Add accessors for modal parameters.

  /// Sets the parameters to @p params, deleting whatever was there before.
  void set_parameters(std::unique_ptr<Parameters<T>> params) {
    parameters_ = std::move(params);
  }

  /// Returns the number of vector-valued parameters.
  int num_numeric_parameters() const {
    return parameters_->num_numeric_parameters();
  }

  /// Returns a const pointer to the vector-valued parameter at @p index.
  /// Asserts if @p index doesn't exist.
  const BasicVector<T>* get_numeric_parameter(int index) const {
    return parameters_->get_numeric_parameter(index);
  }

  /// Returns a mutable pointer to element @p index of the vector-valued
  /// parameters. Asserts if @p index doesn't exist.
  BasicVector<T>* get_mutable_numeric_parameter(int index) {
    return parameters_->get_mutable_numeric_parameter(index);
  }

  // =========================================================================
  // Miscellaneous public methods.


  void PropagateInvalidOutputs(int context_index,
                               int port_index) const override {
    DRAKE_ABORT_MSG("Leaf systems should never be asked to propagate.");
  }

 protected:
  void DoSetInputPort(int index, std::unique_ptr<InputPort> port) override {
    inputs_[index] = std::move(port);
  }

  /// The caller owns the returned memory.
  Context<T>* DoClone() const override {
    LeafContext<T>* clone = new LeafContext<T>(get_num_input_ports(),
                                               get_num_output_ports());

    // Make a deep copy of the continuous state using BasicVector::Clone().
    if (this->get_continuous_state() != nullptr) {
      const ContinuousState<T>& xc = *this->get_continuous_state();
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const BasicVector<T>& xc_vector =
          dynamic_cast<const BasicVector<T>&>(xc.get_vector());
      clone->set_continuous_state(std::make_unique<ContinuousState<T>>(
          xc_vector.Clone(), num_q, num_v, num_z));
    }

    // Make deep copies of the difference and modal states.
    clone->set_difference_state(get_state().get_difference_state()->Clone());
    clone->set_modal_state(get_state().get_modal_state()->Clone());

    // Make deep copies of the parameters.
    clone->set_parameters(parameters_->Clone());

    // Make deep copies of the inputs into FreestandingInputPorts.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (int i = 0; i < get_num_input_ports(); ++i) {
      const InputPort* port = this->inputs_[i].get();
      if (port == nullptr) {
        clone->inputs_[i] = nullptr;
      } else {
        clone->inputs_[i] = std::make_unique<FreestandingInputPort>(
            port->template get_vector_data<T>()->Clone());
      }
    }
    return clone;
  }

  const InputPort* GetInputPort(int index) const override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    return inputs_[index].get();
  }

 private:
  // LeafContext objects are neither copyable nor moveable.
  LeafContext(const LeafContext& other) = delete;
  LeafContext& operator=(const LeafContext& other) = delete;
  LeafContext(LeafContext&& other) = delete;
  LeafContext& operator=(LeafContext&& other) = delete;

  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPort>> inputs_;

  // The number of output ports of the System.
  int num_output_ports_{0};

  // The internal state of the System.
  State<T> state_;

  // The parameters of the system.
  std::unique_ptr<Parameters<T>> parameters_;

  // The cache. The System may insert arbitrary key-value pairs, and configure
  // invalidation on a per-entry basis.
  mutable Cache cache_;
};

}  // namespace systems
}  // namespace drake
