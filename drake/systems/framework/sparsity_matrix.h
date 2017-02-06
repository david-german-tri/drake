#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

class SparsityMatrix {
 public:
  explicit SparsityMatrix(const System<symbolic::Expression>& system);
  ~SparsityMatrix();

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SparsityMatrix)

  bool IsConnectedInputToOutput(int input_port, int output_port) const;

 private:
  std::unique_ptr<Context<symbolic::Expression>> context_;
  std::unique_ptr<SystemOutput<symbolic::Expression>> output_;
  std::vector<std::vector<symbolic::Expression>> input_variables_;
};

}  // namespace systems
}  // namespace drake
