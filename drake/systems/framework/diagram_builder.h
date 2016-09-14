#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram. It collects the dependency
/// graph of constituent systems, and topologically sorts them. It is single
/// use: after calling Build or BuildInto, DiagramBuilder gives up ownership
/// of the constituent systems, and should therefore be discarded.
template <typename T>
class DiagramBuilder {
 public:
  DiagramBuilder() {}
  virtual ~DiagramBuilder() {}

  /// Adds a given subsystem to the DiagramBuilder.  A system must be added
  /// before it can be wired up in any other way.  The DiagramBuilder takes
  /// ownership, and returns a raw pointer to the System, which is guaranteed
  /// to remain valid for the lifetime of the builder or the Diagram it
  /// builds, whichever is longer.
  template <typename S>
  S* Register(std::unique_ptr<S> subsystem) {
    S* raw_subsystem_ptr = subsystem.get();
    systems_.insert(raw_subsystem_ptr);
    registered_systems_.push_back(std::move(subsystem));
    return raw_subsystem_ptr;
  }

  /// Declares that input port @p dest is connected to output port @p src.
  void Connect(const SystemPortDescriptor<T>& src,
               const SystemPortDescriptor<T>& dest) {
    DRAKE_DEMAND(src.get_face() == kOutputPort);
    DRAKE_DEMAND(dest.get_face() == kInputPort);
    PortIdentifier dest_id{dest.get_system(), dest.get_index()};
    PortIdentifier src_id{src.get_system(), src.get_index()};
    ThrowIfInputAlreadyWired(dest_id);
    AbortIfNotRegistered(src.get_system());
    AbortIfNotRegistered(dest.get_system());
    dependency_graph_[dest_id] = src_id;
  }

  /// Declares that sole input port on the @p dest system is connected to sole
  /// output port on the @p src system.  Throws an exception if the sole-port
  /// precondition is not met (i.e., if @p dest has no input ports, or @p dest
  /// has more than one input port, or @p src has no output ports, or @p src
  /// has more than one output port).
  void Connect(const System<T>& src, const System<T>& dest) {
    DRAKE_THROW_UNLESS(src.get_num_output_ports() == 1);
    DRAKE_THROW_UNLESS(dest.get_num_input_ports() == 1);
    Connect(src.get_output_port(0), dest.get_input_port(0));
  }

  /// Cascades @p src and @p dest.  The sole input port on the @p dest system
  /// is connected to sole output port on the @p src system.  Throws an
  /// exception if the sole-port precondition is not met (i.e., if @p dest has
  /// no input ports, or @p dest has more than one input port, or @p src has no
  /// output ports, or @p src has more than one output port).
  void Cascade(const System<T>& src, const System<T>& dest) {
    Connect(src, dest);
  }

  /// Declares that the given @p input port of a constituent system is an input
  /// to the entire Diagram.
  void ExportInput(const SystemPortDescriptor<T>& input) {
    DRAKE_DEMAND(input.get_face() == kInputPort);
    PortIdentifier id{input.get_system(), input.get_index()};
    ThrowIfInputAlreadyWired(id);
    AbortIfNotRegistered(input.get_system());
    input_port_ids_.push_back(id);
    diagram_input_set_.insert(id);
  }

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.
  void ExportOutput(const SystemPortDescriptor<T>& output) {
    DRAKE_DEMAND(output.get_face() == kOutputPort);
    AbortIfNotRegistered(output.get_system());
    output_port_ids_.push_back(
        PortIdentifier{output.get_system(), output.get_index()});
  }

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput. Throws std::logic_error if the graph is
  /// not buildable.
  std::unique_ptr<Diagram<T>> Build() const {
    return std::unique_ptr<Diagram<T>>(new Diagram<T>(std::move(Compile())));
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput. Throws
  /// std::logic_error if the graph is not buildable.
  ///
  /// Only Diagram subclasses should call this method. The target must not
  /// already be initialized.
  void BuildInto(Diagram<T>* target) const {
    target->Initialize(std::move(Compile()));
  }

 private:
  typedef typename Diagram<T>::PortIdentifier PortIdentifier;

  void ThrowIfInputAlreadyWired(const PortIdentifier& id) const {
    if (dependency_graph_.find(id) != dependency_graph_.end() ||
        diagram_input_set_.find(id) != diagram_input_set_.end()) {
      throw std::logic_error("Input port is already wired.");
    }
  }

  void AbortIfNotRegistered(const System<T>* system) const {
    DRAKE_DEMAND(systems_.find(system) != systems_.end());
  }

  // Runs Kahn's algorithm to compute the topological sort order of the
  // Systems in the graph. If EvalOutput is called on each System in
  // the order that is returned, each System's inputs will be valid by
  // the time its EvalOutput is called.
  //
  // TODO(david-german-tri): Update this sort to operate on individual
  // output ports once #2890 is resolved.
  //
  // TODO(david-german-tri, bradking): Consider using functional form to
  // produce a separate execution order for each output of the Diagram.
  std::vector<const System<T>*> SortSystems() const {
    std::vector<const System<T>*> sorted_systems;

    // Build two maps:
    // A map from each system, to every system that depends on it.
    std::map<const System<T>*, std::set<const System<T>*>> dependents;
    // A map from each system, to every system on which it depends.
    std::map<const System<T>*, std::set<const System<T>*>> dependencies;

    for (const auto& connection : dependency_graph_) {
      const System<T>* src = connection.second.first;
      const System<T>* dest = connection.first.first;
      // If a system is not direct-feedthrough, the connections to its inputs
      // are not relevant for detecting algebraic loops or determining
      // execution order.
      //
      // TODO(david-german-tri): Make direct-feedthrough resolution more
      // fine-grained once #3170 is resolved.
      if (dest->has_any_direct_feedthrough()) {
        dependents[src].insert(dest);
        dependencies[dest].insert(src);
      }
    }

    // Find the systems that have no direct-feedthrough inputs.
    std::vector<const System<T>*> nodes_with_in_degree_zero;
    for (const auto& system : registered_systems_) {
      const System<T>* sys = system.get();
      if (dependencies.find(sys) == dependencies.end()) {
        nodes_with_in_degree_zero.push_back(sys);
      }
    }

    while (!nodes_with_in_degree_zero.empty()) {
      // Pop a node with in-degree zero.
      const System<T>* node = nodes_with_in_degree_zero.back();
      nodes_with_in_degree_zero.pop_back();

      // Push the node onto the sorted output.
      sorted_systems.push_back(node);

      for (const System<T>* dependent : dependents[node]) {
        dependencies[dependent].erase(node);
        if (dependencies[dependent].empty()) {
          nodes_with_in_degree_zero.push_back(dependent);
        }
      }
    }

    if (sorted_systems.size() != systems_.size()) {
      throw std::logic_error("Algebraic loop detected in DiagramBuilder.");
    }
    return sorted_systems;
  }

  /// Produces the Blueprint that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  typename Diagram<T>::Blueprint Compile() const {
    if (registered_systems_.size() == 0) {
      throw std::logic_error("Cannot Compile an empty DiagramBuilder.");
    }
    typename Diagram<T>::Blueprint blueprint;
    blueprint.input_port_ids = input_port_ids_;
    blueprint.output_port_ids = output_port_ids_;
    blueprint.dependency_graph = dependency_graph_;
    blueprint.sorted_systems = SortSystems();
    return blueprint;
  }

  // DiagramBuilder objects are neither copyable nor moveable.
  DiagramBuilder(const DiagramBuilder<T>& other) = delete;
  DiagramBuilder& operator=(const DiagramBuilder<T>& other) = delete;
  DiagramBuilder(DiagramBuilder<T>&& other) = delete;
  DiagramBuilder& operator=(DiagramBuilder<T>&& other) = delete;

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For fast membership queries: has this input port already been declared?
  std::set<PortIdentifier> diagram_input_set_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The unsorted set of Systems in this DiagramBuilder. Used for fast
  // membership queries.
  std::set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;
};

}  // namespace systems
}  // namespace drake
