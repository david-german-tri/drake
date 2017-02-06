#pragma once

#include "drake/systems/framework/sparsity_matrix.h"

#include <sstream>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

SparsityMatrix::SparsityMatrix(const System<symbolic::Expression>& system)
    : context_(system.CreateDefaultContext()),
      output_(system.AllocateOutput(*context_)),
      input_variables_(system.get_num_input_ports()) {
  // Time
  context_->set_time(symbolic::Expression("t"));
  // Input
  for (int i = 0; i < system.get_num_input_ports(); ++i) {
    if (system.get_input_port(i).get_data_type() == kVectorValued) {
      const int n = system.get_input_port(i).size();
      auto value = std::make_unique<BasicVector<symbolic::Expression>>(n);
      for (int j = 0; j < n; ++j) {
        std::ostringstream name;
        name << "u" << i << "_" << j;
        value->SetAtIndex(j, symbolic::Expression(name.str()));
        input_variables_[i].push_back(name.str());
      }
      context_->FixInputPort(i, std::move(value));
    }
  }
  // Continuous State
  VectorBase<symbolic::Expression>& xc =
      *context_->get_mutable_continuous_state_vector();
  for (int i = 0; i < xc.size(); ++i) {
    std::ostringstream name;
    name << "xc" << i;
    xc[i] = symbolic::Expression(name.str());
  }
  // Discrete State
  auto& xd = *context_->get_mutable_discrete_state();
  for (int i = 0; i < context_->get_num_discrete_state_groups(); ++i) {
    auto& xdi = *xd.get_mutable_discrete_state(i);
    for (int j = 0; j < xdi.size(); ++j) {
      std::ostringstream name;
      name << "xd" << i << "_" << j;
      xdi[j] = symbolic::Expression(name.str());
    }
  }
  // TODO(david-german-tri): Parameters. This will require promoting them
  // from LeafContext to Context. See #5072.

}

SparsityMatrix::~SparsityMatrix() {}

// TODO note - this will return true for abstract valued input ports.  or output?  TBD.  Tricky!
bool SparsityMatrix::IsConnectedInputToOutput(int input_port,
                                              int output_port) const {
  for (int i = 0; i < input_variables_[input_port].size(); ++i) {

  }
}

}  // namespace systems
}  // namespace drake
