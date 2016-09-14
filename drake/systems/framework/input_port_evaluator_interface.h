#pragma once

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

template <typename T> class Context;

namespace detail {

/// InputPortEvaluatorInterface is implemented by classes that are able to
/// evaluate the OutputPort connected to a particular InputPort.
///
/// This interface is a Drake-internal detail. Users should never implement
/// it. In fact, only Diagram should implement it.
///
/// @tparam T A mathematical type that is a valid Eigen scalar.
template <typename T>
class DRAKESYSTEMFRAMEWORK_EXPORT InputPortEvaluatorInterface {
 public:
  virtual ~InputPortEvaluatorInterface() {}

  /// Evaluates the input port with the given @p id in the given @p context.
  /// Aborts if @p context is nullptr and there is any evaluation to do.
  virtual void EvaluateInputPort(const Context<T>* context,
                                 const SystemPortDescriptor<T>& id) const = 0;
};

}  // namespace detail
}  // namespace systems
}  // namespace drake
