#pragma once

#include <functional>

#include "drake/common/drake_throw.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

namespace internal {

/// ContextCacheTickets is a container for cache tickets corresponding to
/// Context fields and groups of fields. In the typical use case, no data will
/// be stored in these caceh entries. They exist only so that other cache
/// entries may depend on them.
struct ContextCacheTickets {
  // time
  CacheTicket time_ticket{kNullCacheTicket};
  // input
  std::vector<CacheTicket> input_tickets;
  // continuous state
  CacheTicket generalized_position_ticket{kNullCacheTicket};
  CacheTicket generalized_velocity_ticket{kNullCacheTicket};
  CacheTicket misc_continuous_state_ticket{kNullCacheTicket};
  // difference state
  std::vector<CacheTicket> difference_state_tickets;
  // modal state
  std::vector<CacheTicket> modal_state_tickets;
  // groups
  CacheTicket inputs_ticket{kNullCacheTicket};
  CacheTicket continuous_state_ticket{kNullCacheTicket};
  CacheTicket difference_state_ticket{kNullCacheTicket};
  CacheTicket modal_state_ticket{kNullCacheTicket};
  CacheTicket state_ticket{kNullCacheTicket};
  CacheTicket context_ticket{kNullCacheTicket};
};


}  // namespace internal

/// Contains information about the independent variable including time and
/// step number.
// TODO(sherm1) Add step information.
template <typename T>
struct StepInfo {
  /// The time, in seconds. For typical T implementations based on
  /// doubles, time resolution will gradually degrade as time increases.
  // TODO(sherm1): Consider whether this is sufficiently robust.
  T time_sec{};
};

/// Context is an abstract base class template that represents all
/// the inputs to a System: time, state, and input vectors. The framework
/// provides two concrete subclasses of Context: LeafContext (for
/// leaf Systems) and DiagramContext (for composite Systems). Users are
/// discouraged from creating additional subclasses.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class Context {
 public:
  virtual ~Context() {}

  // =========================================================================
  // Accessors and Mutators for Time.

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Set the current time in seconds.
  virtual void set_time(const T& time_sec)  {
    get_mutable_step_info()->time_sec = time_sec;
  }

  // =========================================================================
  // Accessors and Mutators for State.

  virtual const State<T>& get_state() const = 0;

  State<T>* get_mutable_state() {
    InvalidateState();
    return const_cast<State<T>*>(&get_state());
  }

  /// Sets the continuous state to @p xc, deleting whatever was there before.
  /// This is a violent action that changes the shape of the Context. It
  /// therefore invalidates the entire cache and generates completely new
  /// cache tickets for every cache field.
  void set_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    get_mutable_state()->set_continuous_state(std::move(xc));
    BuildCacheTickets();
  }

  /// Returns a mutable pointer to the continuous component of the state,
  /// which may be of size zero. Invalidates all cache entries that depend on
  /// continuous state.
  /// TODO(david-german-tri): Provide finer-grained accessors for q, v, and z,
  /// with correspondingly less dramatic cache invalidation.
  ContinuousState<T>* get_mutable_continuous_state() {
    InvalidateContinuousState();
    return get_mutable_state()->get_mutable_continuous_state();
  }

  /// Returns a mutable pointer to the continuous state, devoid of second-order
  /// structure. The vector may be of size zero. Invalidates all cache entries
  /// that depend on continuous state.
  VectorBase<T>* get_mutable_continuous_state_vector() {
    InvalidateContinuousState();
    return get_mutable_continuous_state()->get_mutable_vector();
  }

  /// Returns a const pointer to the continuous component of the state,
  /// which may be of size zero.
  const ContinuousState<T>* get_continuous_state() const {
    return get_state().get_continuous_state();
  }

  /// Returns a reference to the continuous state vector, devoid of second-order
  /// structure. The vector may be of size zero.
  const VectorBase<T>& get_continuous_state_vector() const {
    return get_continuous_state()->get_vector();
  }


  /// Returns a mutable pointer to the difference component of the state,
  /// which may be of size zero. Invalidates all cache entries that depend on
  /// difference state.
  DifferenceState<T>* get_mutable_difference_state() {
    InvalidateDifferenceState();
    return get_mutable_state()->get_mutable_difference_state();
  }

  /// Returns a mutable pointer to element @p index of the difference state.
  /// Asserts if @p index doesn't exist. Invalidates all cache entries that
  /// depend on that element of difference state.
  BasicVector<T>* get_mutable_difference_state(int index) {
    InvalidateDifferenceState(index);
    DifferenceState<T>* xd =
        get_mutable_state()->get_mutable_difference_state();
    return xd->get_mutable_difference_state(index);
  }

  /// Sets the discrete state to @p xd, deleting whatever was there before.
  /// This is a violent action that changes the shape of the Context. It
  /// therefore invalidates the entire cache and generates completely new
  /// cache tickets for every cache field.
  void set_difference_state(std::unique_ptr<DifferenceState<T>> xd) {
    get_mutable_state()->set_difference_state(std::move(xd));
    BuildCacheTickets();
  }

  /// Returns a const pointer to the discrete difference component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  const BasicVector<T>* get_difference_state(int index) const {
    const DifferenceState<T>* xd = get_state().get_difference_state();
    return xd->get_difference_state(index);
  }

  /// Returns a mutable pointer to the modal component of the state,
  /// which may be of size zero. Invalidates all cache entries that depend
  /// on modal state.
  ModalState* get_mutable_modal_state() {
    InvalidateModalState();
    return get_mutable_state()->get_mutable_modal_state();
  }

  /// Returns a mutable pointer to element @p index of the modal state.
  /// Asserts if @p index doesn't exist. Invalidates all cache entries that
  /// depend on that element of modal state.
  template <typename U>
  U& get_mutable_modal_state(int index) {
    InvalidateModalState(index);
    ModalState* xm = get_mutable_state()->get_mutable_modal_state();
    return xm->get_mutable_modal_state(index).GetMutableValue<U>();
  }

  /// Sets the modal state to @p xm, deleting whatever was there before.
  /// This is a violent action that changes the shape of the Context. It
  /// therefore invalidates the entire cache and generates completely new
  /// cache tickets for every cache field.
  void set_modal_state(std::unique_ptr<ModalState> xm) {
    get_mutable_state()->set_modal_state(std::move(xm));
    BuildCacheTickets();
  }

  /// Returns a const pointer to the discrete modal component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  template <typename U>
  const U& get_modal_state(int index) const {
    const ModalState* xm = get_state().get_modal_state();
    return xm->get_modal_state(index).GetValue<U>();
  }

  // =========================================================================
  // Accessors and Mutators for Input.

  /// Connects the input port @p port to this Context at the given @p index.
  /// Disconnects whatever input port was previously there. Registers the new
  /// input port for invalidation notifications from the upstream output port,
  /// and deregisters the old one. Asserts if @p index is out of range.
  void SetInputPort(int index, std::unique_ptr<InputPort> port) {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    // Right now, invalidate everything that depends on this input port.
    InvalidateInput(index);
    // Whenever the value on the input port changes, invalidate all the entries
    // in our cache that depend on it.
    port->set_invalidation_callback(
        std::bind(&Context<T>::InvalidateInput, this, index));
    DoSetInputPort(index, std::move(port));
  }

  /// The NVI implementation of SetInputPort. Implementations must actually
  /// connect the input port to the context, disconnect whatever input port
  /// was previously there, and deregister that input port from the output
  /// port on which it depends. In some Context implementations, this may
  /// require a recursive search through a tree of subcontexts.
  virtual void DoSetInputPort(int index, std::unique_ptr<InputPort> port) = 0;

  /// Returns the number of input ports.
  virtual int get_num_input_ports() const = 0;

  /// Connects a FreestandingInputPort with the given @p value at the given
  /// @p index. Asserts if @p index is out of range.
  void FixInputPort(int index, std::unique_ptr<BasicVector<T>> value) {
    SetInputPort(index,
                 std::make_unique<FreestandingInputPort>(std::move(value)));
  }

  /// Evaluates and returns the input port identified by @p descriptor,
  /// using the given @p evaluator, which should be the Diagram containing
  /// the System that allocated this Context. The evaluation will be performed
  /// in this Context's parent. It is a recursive operation that may invoke
  /// long chains of evaluation through all the Systems that are prerequisites
  /// to the specified port.
  ///
  /// Returns nullptr if the port is not connected. Aborts if the port does
  /// not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const InputPort* EvalInputPort(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const int index = descriptor.get_index();
    const InputPort* port = GetInputPort(index);
    if (port == nullptr) return nullptr;
    if (port->requires_evaluation()) {
      DRAKE_DEMAND(evaluator != nullptr);
      evaluator->EvaluateSubsystemInputPort(parent_, descriptor);
    }
    return port;
  }

  /// Evaluates and returns the vector data of the input port with the given
  /// @p descriptor. This is a recursive operation that may invoke long chains
  /// of evaluation through all the Systems that are prerequisite to the
  /// specified port. It invalidates all cache entries that depend on
  /// the input port being evaluated.
  ///
  /// Returns nullptr if the port is not connected.
  /// Throws std::bad_cast if the port is not vector-valued.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const BasicVector<T>* EvalVectorInput(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const InputPort* port = EvalInputPort(evaluator, descriptor);
    if (port == nullptr) return nullptr;
    return port->template get_vector_data<T>();
  }

  /// Evaluates and returns the abstract data of the input port at @p index.
  /// This is a recursive operation that may invoke long chains of evaluation
  /// through all the Systems that are prerequisite to the specified port. It
  /// invalidates all cache entries that depend on the input port being
  /// evaluated.
  ///
  /// Returns nullptr if the port is not connected.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const AbstractValue* EvalAbstractInput(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const InputPort* port = EvalInputPort(evaluator, descriptor);
    if (port == nullptr) return nullptr;
    return port->get_abstract_data();
  }

  /// Evaluates and returns the data of the input port at @p index.
  /// This is a recursive operation that may invoke long chains of evaluation
  /// through all the Systems that are prerequisite to the specified port. It
  /// invalidates all cache entries that depend on the input port being
  /// evaluated.
  ///
  /// Returns nullptr if the port is not connected.
  /// Throws std::bad_cast if the port does not have type V.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* EvalInputValue(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const AbstractValue* value = EvalAbstractInput(evaluator, descriptor);
    if (value == nullptr) return nullptr;
    return &(value->GetValue<V>());
  }

  // =========================================================================
  // Accessors and mutators for the Cache itself.

  /// Reserves a cache entry with the given @p prerequisites on which it
  /// depends. Returns a ticket to identify the entry.
  CacheTicket CreateCacheEntry(
      const std::set<CacheTicket>& prerequisites) const {
    // TODO(david-german-tri): Provide a notation for specifying context
    // dependencies as well, and provide automatic invalidation when the
    // context dependencies change.
    return this->cache().MakeCacheTicket(prerequisites);
  }

  /// Stores the given @p value in the cache entry for the given @p ticket,
  /// and returns a bare pointer to @p value.  That pointer will be invalidated
  /// whenever any of the @p ticket's declared prerequisites change, and
  /// possibly also at other times which are not defined.
  ///
  /// Systems MUST NOT depend on a particular value being present or valid
  /// in the Cache, and MUST check the validity of cached values using
  /// the GetCachedValue interface.
  //
  /// The Cache is useful to avoid recomputing expensive intermediate data. It
  /// is not a scratch space for arbitrary state. If you cannot derive a value
  /// from other fields in the Context, do not put that value in the Cache.
  /// If you violate this rule, you may be devoured by a horror from another
  /// universe, and forced to fill out paperwork in triplicate for all eternity.
  /// You have been warned.
  AbstractValue* InitCachedValue(CacheTicket ticket,
                                 std::unique_ptr<AbstractValue> value) const {
    return this->cache().Init(ticket, std::move(value));
  }

  /// Copies the given @p value into the cache entry for the given @p ticket.
  /// May throw std::bad_cast if the type of the existing value is not V.
  ///
  /// @tparam V The type of the value to store.
  template <typename V>
  void SetCachedValue(CacheTicket ticket, const V& value) const {
    this->cache().template Set<V>(ticket, value);
  }

  // Returns the cached value for the given @p ticket, or nullptr if the
  // cache entry has been invalidated.
  const AbstractValue* GetCachedValue(CacheTicket ticket) const {
    return this->cache().Get(ticket);
  }

  // =========================================================================
  // Accessors for cache tickets.

  /// Returns a cache ticket for the whole context. If a computation should
  /// be invalidated whenever anything at all in the Context changes, store
  /// it in a cache entry that depends on this ticket.
  CacheTicket get_context_ticket() const {
    return cache_tickets_.context_ticket;
  }

  // =========================================================================
  // Miscellaneous Public Methods

  /// Returns a deep copy of this Context. The clone's input ports will
  /// hold deep copies of the data that appears on this context's input ports
  /// at the time the clone is created.
  std::unique_ptr<Context<T>> Clone() const {
    // Clone the concrete implementation's data via NVI.
    std::unique_ptr<Context<T>> clone(DoClone());
    // Clone the data that belongs to this abstract base class.
    *clone->get_mutable_step_info() = this->get_step_info();
    clone->cache_ = this->cache_;
    clone->cache_tickets_ = this->cache_tickets_;
    return clone;
  }

  /// Initializes this context's time, state, and parameters from the real
  /// values in @p source, regardless of this context's scalar type.
  /// Requires a constructor T(double). Invalidates all cache entries that
  /// depend on time, state, or parameters.
  void SetTimeStateAndParametersFrom(const Context<double>& source) {
    set_time(T(source.get_time()));
    get_mutable_state()->SetFrom(source.get_state());
    // TODO(david-german-tri): Parameters.
  }

  /// Declares that @p parent is the context of the enclosing Diagram. The
  /// enclosing Diagram context is needed to evaluate inputs recursively.
  /// Aborts if the parent has already been set to something else.
  ///
  /// This is a dangerous implementation detail. Conceptually, a Context
  /// ought to be completely ignorant of its parent Context. However, we
  /// need this pointer so that we can cause our inputs to be evaluated in
  /// EvalInputPort.  See https://github.com/RobotLocomotion/drake/pull/3455.
  void set_parent(const Context<T>* parent) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
  }

  // Throws an exception unless the given @p descriptor matches this context.
  void VerifyInputPort(const SystemPortDescriptor<T>& descriptor) const {
    const int i = descriptor.get_index();
    const InputPort* port = GetInputPort(i);
    DRAKE_THROW_UNLESS(port != nullptr);
    // TODO(david-german-tri, sherm1): Consider checking sampling here.

    // In the vector-valued case, check the size.
    if (descriptor.get_data_type() == kVectorValued) {
      const BasicVector<T>* input_vector = port->template get_vector_data<T>();
      DRAKE_THROW_UNLESS(input_vector != nullptr);
      DRAKE_THROW_UNLESS(input_vector->size() == descriptor.get_size());
    }
    // In the abstract-valued case, there is nothing else to check.
  }

 protected:
  /// Contains the return-type-covariant implementation of Clone().
  virtual Context<T>* DoClone() const = 0;

  /// Returns a const reference to current time and step information.
  const StepInfo<T>& get_step_info() const { return step_info_; }

  /// Provides writable access to time and step information, with the side
  /// effect of invaliding any computation that is dependent on them.
  /// TODO(david-german-tri) Invalidate all cached time- and step-dependent
  /// computations.
  StepInfo<T>* get_mutable_step_info() {
    cache_.Invalidate(cache_tickets_.time_ticket);
    return &step_info_;
  }

  /// Returns the InputPort at the given @p index, which may be nullptr if
  /// it has never been set with SetInputPort.
  /// Asserts if @p index is out of range.
  virtual const InputPort* GetInputPort(int index) const = 0;

  /// Returns the InputPort at the given @p index from the given @p context.
  /// Returns nullptr if the given port has never been set with SetInputPort.
  /// Asserts if @p index is out of range.
  static const InputPort* GetInputPort(const Context<T>& context, int index) {
    return context.GetInputPort(index);
  }

  Cache& cache() const {
    return cache_;
  }

  void InvalidateEverything() {
    InvalidateTime();
    InvalidateInputs();
    InvalidateState();
  }

  void InvalidateTime() {
    cache_.Invalidate(cache_tickets_.time_ticket);
  }


  // TODO - explain why const
  void InvalidateInput(int index) const {
    cache_.Invalidate(cache_tickets_.input_tickets[index]);
  }

  // TODO - explain why const
  void InvalidateInputs() const {
    for (const CacheTicket& ticket : cache_tickets_.input_tickets) {
      cache_.Invalidate(ticket);
    }
  }

  void InvalidateState() {
    InvalidateContinuousState();
    InvalidateDifferenceState();
    InvalidateModalState();
  }

  void InvalidateContinuousState() {
    cache_.Invalidate(cache_tickets_.generalized_position_ticket);
    cache_.Invalidate(cache_tickets_.generalized_velocity_ticket);
    cache_.Invalidate(cache_tickets_.misc_continuous_state_ticket);
  }

  void InvalidateDifferenceState() {
    for (const CacheTicket& ticket : cache_tickets_.difference_state_tickets) {
      cache_.Invalidate(ticket);
    }
  }

  void InvalidateDifferenceState(int index) {
    cache_.Invalidate(cache_tickets_.difference_state_tickets[index]);
  }

  void InvalidateModalState() {
    for (const CacheTicket& ticket : cache_tickets_.modal_state_tickets) {
      cache_.Invalidate(ticket);
    }
  }

  void InvalidateModalState(int index) {
    cache_.Invalidate(cache_tickets_.modal_state_tickets[index]);
  }

  void BuildCacheTickets() {
    InvalidateEverything();
    internal::ContextCacheTickets& tix = cache_tickets_;
    // Time.
    tix.time_ticket = cache_.MakeCacheTicket({});

    // Input.
    tix.input_tickets.clear();
    for (int i = 0; i < get_num_input_ports(); ++i) {
      tix.input_tickets.push_back(cache_.MakeCacheTicket({}));
    }
    tix.inputs_ticket = cache_.MakeCacheTicket(
        std::set<CacheTicket>(tix.input_tickets.begin(),
                              tix.input_tickets.end()));

    // Continuous state.
    // -- q
    tix.generalized_position_ticket = cache_.MakeCacheTicket({});
    // -- v
    tix.generalized_velocity_ticket = cache_.MakeCacheTicket({});
    // -- z
    tix.misc_continuous_state_ticket = cache_.MakeCacheTicket({});
    tix.continuous_state_ticket = cache_.MakeCacheTicket(
        {tix.generalized_position_ticket, tix.generalized_velocity_ticket,
      tix.misc_continuous_state_ticket});

    // Difference state.
    tix.difference_state_tickets.clear();
    for (int i = 0; i < get_state().get_difference_state()->size(); ++i) {
      tix.difference_state_tickets.push_back(cache_.MakeCacheTicket({}));
    }
    tix.difference_state_ticket = cache_.MakeCacheTicket(
        std::set<CacheTicket>(tix.difference_state_tickets.begin(),
                              tix.difference_state_tickets.end()));

    // Modal state.
    tix.modal_state_tickets.clear();
    for (int i = 0; i < get_state().get_modal_state()->size(); ++i) {
      tix.modal_state_tickets.push_back(cache_.MakeCacheTicket({}));
    }
    tix.modal_state_ticket = cache_.MakeCacheTicket(
        std::set<CacheTicket>(tix.modal_state_tickets.begin(),
                              tix.modal_state_tickets.end()));

    // All state.
    tix.state_ticket = cache_.MakeCacheTicket(
        {tix.continuous_state_ticket, tix.difference_state_ticket,
      tix.modal_state_ticket});

    // All context.
    tix.context_ticket = cache_.MakeCacheTicket(
        {tix.time_ticket, tix.inputs_ticket, tix.state_ticket});
    // TODO(david-german-tri): Parameters.
  }

 private:
  // Current time and step information.
  StepInfo<T> step_info_;

  // The context of the enclosing Diagram, used in EvalInputPort.
  // This pointer MUST be treated as a black box. If you call any substantive
  // methods on it, you are probably making a mistake.
  const Context<T>* parent_ = nullptr;

  // The cache. The System may insert arbitrary key-value pairs, and configure
  // invalidation on a per-entry basis.
  mutable Cache cache_;

  internal::ContextCacheTickets cache_tickets_;
};

}  // namespace systems
}  // namespace drake

