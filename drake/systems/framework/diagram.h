#pragma once

#include <map>
#include <set>
#include <vector>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_interface.h"

namespace drake {
namespace systems {

typedef enum {
  UNKNOWN = 0,
  CONTINUOUS_PORT = 1
} PortType;

template <typename T>
struct PortIdentifier {
  SystemInterface<T>* system;
  PortType port_type;
  int port_index;
};

template <typename T> class Diagram
    : public ContinuousSystemInterface<T> {
 public:
  Diagram(const std::string& name) : name_(name) {}
  virtual ~Diagram() {}

  std::string get_name() const override { return name_; }

  void Connect(SystemInterface<T>* src,
               SystemInterface<T>* dest,
               PortType port_type, int src_port_index, int dest_port_index) {
    ThrowIfFinal();
    Register(src);
    Register(dest);
    dependents_[src].insert(dest);
    dependencies_[dest].insert(src);

    switch (port_type) {
      case CONTINUOUS_PORT:
      {
        auto& src_ports = outputs_[src]->continuous_ports;
        auto& dest_ports = contexts_[dest]->get_mutable_input()->continuous_ports;
        if (src_port_index >= src_ports.size()) {
          throw std::runtime_error("Invalid output port.");
        }
        if (dest_port_index >= dest_ports.size()) {
          throw std::runtime_error("Invalid input port.");
        }
        dest_ports[dest_port_index].input = src_ports[src_port_index].output.get();
        break;
      }
    }
  }

  void AddInput(SystemInterface<T>* sys,
                PortType port_type, int port_index) {
    ThrowIfFinal();
    Register(sys);
    diagram_inputs_.push_back(PortIdentifier<T>{sys, port_type, port_index});
  }

  void AddOutput(SystemInterface<T>* sys,
                 PortType port_type, int port_index) {
    ThrowIfFinal();
    Register(sys);
    diagram_outputs_.push_back(PortIdentifier<T>{sys, port_type, port_index});
  }

  void Finalize() {
    ThrowIfFinal();
    sorted_systems_ = SortSystems();
    ThrowIfInputsInvalid();
    ThrowIfOutputsInvalid();
  }

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    ThrowIfNotFinal();
    std::unique_ptr<Context<T>> context(new Context<T>);
    // Reserve inputs as specified during Diagram initialization.
    context->get_mutable_input()->continuous_ports.resize(diagram_inputs_.size());
    // TODO: Reserve state for all constituent Systems.
    return context;
  }

  // Returns a default output, initialized with the correct number of
  // concrete output ports for this System.
  std::unique_ptr<SystemOutput<T>> CreateDefaultOutput() const override {
    ThrowIfNotFinal();
    std::unique_ptr<SystemOutput<T>> output(new SystemOutput<T>);
    // TODO: something.
    return output;
  }

  void Output(const Context<T>& context, SystemOutput<T>* output) const {
    // TODO: something
  }

  virtual void GetDerivativesOfGeneralizedPosition(
      const Context<T>& context, VectorInterface<T>* derivatives) const {
    // TODO: something
  }

  virtual void GetDerivativesOfGeneralizedVelocity(
      const Context<T>& context, VectorInterface<T>* derivatives) const {
    // TODO: something
  }

  virtual void GetDerivativesOfOtherContinuousState(
      const Context<T>& context, VectorInterface<T>* derivatives) const {
    // TODO: something
  }

  virtual void MapVelocityToConfigurationDerivative(
      const Context<T>& context, VectorInterface<T>* derivatives) const {
    // TODO: something
  }

 private:
  void Register(SystemInterface<T>* sys) {
    if (contexts_.find(sys) != contexts_.end()) {
      // This system is already registered.
      return;
    }
    contexts_[sys] = sys->CreateDefaultContext();
    outputs_[sys] = sys->CreateDefaultOutput();
    dependencies_[sys] = {};
    dependents_[sys] = {};
  };

  void ThrowIfFinal() const {
    if (!sorted_systems_.empty()) {
      throw std::runtime_error("Diagram is already finalized.");
    }
  }

  void ThrowIfNotFinal() const {
    if (sorted_systems_.empty()) {
      throw std::runtime_error("Diagram is not finalized.");
    }
  }

  void ThrowIfInputsInvalid() {
    for (const PortIdentifier<T>& input_port : diagram_inputs_) {
      // TODO(david-german-tri): Actually validate something.
    }
  }

  void ThrowIfOutputsInvalid() {
    for (const PortIdentifier<T>& output_port : diagram_outputs_) {
      // TODO(david-german-tri): Actually validate something.
    }
  }

  std::vector<SystemInterface<T>*> SortSystems() const {
    // Kahn's Algorithm.
    std::vector<SystemInterface<T>*> output;

    // Make a temporary copy of the graph structure.
    auto dependents = dependents_;
    auto dependencies = dependencies_;

    // Find the systems that have no inputs within the Diagram.
    std::set<SystemInterface<T>*> nodes_with_in_degree_zero;
    for (const auto& entry : dependencies) {
      if (entry.second.empty()) {
        nodes_with_in_degree_zero.insert(entry);
      }
    }

    while (!nodes_with_in_degree_zero.empty()) {
      // Pop a node with in-degree zero.
      auto it = nodes_with_in_degree_zero.begin();
      SystemInterface<T>* node = *it;
      nodes_with_in_degree_zero.erase(it);

      // Push the node onto the sorted output.
      output.push_back(node);

      for (SystemInterface<T>* dependent : dependents[node]) {
        dependencies[dependent].erase(node);
        if (dependencies[dependent].empty()) {
          nodes_with_in_degree_zero.insert(dependent);
        }
      }
    }

    if (!nodes_with_in_degree_zero.empty()) {
      // TODO(david-german-tri): Attempt to break cycles using
      // the direct-feedthrough configuration of a System.
      throw new std::runtime_error("Cycle detected.");
    }
    return output;
  }

  // Diagram objects are neither copyable nor moveable.
  Diagram(const Diagram<T>& other) = delete;
  Diagram& operator=(const Diagram<T>& other) = delete;
  Diagram(Diagram<T>&& other) = delete;
  Diagram& operator=(Diagram<T>&& other) = delete;

  std::string name_;

  std::map<SystemInterface<T>*, std::unique_ptr<Context<T>>> contexts_;
  std::map<SystemInterface<T>*, std::unique_ptr<SystemOutput<T>>> outputs_;

  std::vector<PortIdentifier<T>> diagram_inputs_;
  std::vector<PortIdentifier<T>> diagram_outputs_;

  std::map<SystemInterface<T>*, std::set<SystemInterface<T>*>> dependents_;
  std::map<SystemInterface<T>*, std::set<SystemInterface<T>*>> dependencies_;
  std::vector<SystemInterface<T>*> sorted_systems_;
};

}  // namespace systems
}  // namespace drake
