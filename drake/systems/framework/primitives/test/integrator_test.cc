#include "drake/systems/framework/primitives/integrator.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 3;

class IntegratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    integrator_.reset(new Integrator<double>(kLength));
    context_ = integrator_->CreateDefaultContext();
    derivatives_ = integrator_->AllocateTimeDerivatives();
    output_ = integrator_->AllocateOutput(*context_);
    input_.reset(new BasicVector<double>(kLength));

    // Set the state to zero initially.
    ContinuousState<double>* xc =
        context_->get_mutable_state()->continuous_state.get();
    EXPECT_EQ(3, xc->get_state().size());
    EXPECT_EQ(3, xc->get_misc_continuous_state().size());
    xc->get_mutable_state()->SetFromVector(Eigen::VectorXd::Zero(kLength));
  }

  static std::unique_ptr<FreestandingInputPort<double>> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return std::unique_ptr<FreestandingInputPort<double>>(
        new FreestandingInputPort<double>(std::move(data)));
  }

  std::unique_ptr<Integrator<double>> integrator_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

// Tests that the output of an integrator is its state.
TEST_F(IntegratorTest, Output) {
  ASSERT_EQ(1, context_->get_num_input_ports());
  input_->get_mutable_value() << 1.0, 2.0, 3.0;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  integrator_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port =
      dynamic_cast<const BasicVector<double>*>(
          output_->get_port(0).get_vector_data());
  ASSERT_NE(nullptr, output_port);

  Eigen::Vector3d expected;
  expected << 0.0, 0.0, 0.0;
  EXPECT_EQ(expected, output_port->get_value());
}

// Tests that the derivatives of an integrator's state are its input.
TEST_F(IntegratorTest, Derivatives) {
  ASSERT_EQ(1, context_->get_num_input_ports());
  input_->get_mutable_value() << 1.0, 2.0, 3.0;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  integrator_->EvalTimeDerivatives(*context_, derivatives_.get());
  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  EXPECT_EQ(expected, derivatives_->get_state().CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
