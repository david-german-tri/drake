#ifndef DRAKE_FEEDBACK_SYSTEM_H
#define DRAKE_FEEDBACK_SYSTEM_H

namespace Drake {

/** FeedbackSystem<System1,System2>
 * @brief Builds a new system from the feedback connection of two simpler
 * systems
 * @concept{system_concept}
 *
 * ![Feedback combination of two
 * systems](http://underactuated.csail.mit.edu/figures/feedback_system.svg)
 *
 */

template <class ScalarType, class System1, class System2>
class FeedbackSystem
    : public BaseSystem<ScalarType, std::pair<typename System1::StateVector, typename System2::StateVector>,
                             typename System1::InputVector, typename System1::OutputVector>
{
public:
  static_assert(std::is_same<typename System1::OutputVector, typename System2::InputVector>::value,
                "System 2 input vector must match System 1 output vector");
  static_assert(std::is_same<typename System2::OutputVector, typename System1::InputVector>::value,
                "System 1 input vector must match System 2 output vector");

  using StateVector = typename std::pair<typename System1::StateVector, typename System2::StateVector>;
  using InputVector = typename System1::InputVector;
  using OutputVector = typename System1::OutputVector;

  FeedbackSystem(std::shared_ptr<System1> sys1, std::shared_ptr<System2> sys2_) : sys1_(sys1_), sys2_(sys2_){};

  StateVector dynamics(const ScalarType& t, const StateVector& x, const InputVector& u) override
  {
    OutputVector y1;
    InputVector y2;
    const auto& x1 = x.first;
    const auto& x2 = x.second;
    subsystemOutputs(t, x1, x2, u, y1, y2);

    StateVector xdot = std::make_pair(sys1_->dynamics(t, x1, static_cast<InputVector>(toEigen(y2) + toEigen(u))),
                                      sys2_->dynamics(t, x2, y1));
    return xdot;
  }

  OutputVector output(const ScalarType& t, const StateVector& x, const InputVector& u) override
  {
    OutputVector y1;
    const auto& x1 = x.first;
    if (!sys1_->isDirectFeedthrough())
    {
      y1 = sys1_->output(t, x1, u);  // then don't need u+y2 here, u will be ignored
    }
    else
    {
      InputVector y2;
      const auto& x2 = x.second;
      y2 = sys2_->output(t, x2, y1);  // y1 might be uninitialized junk, but has to be ok
      y1 = sys1_->output(t, x1, static_cast<InputVector>(toEigen(y2) + toEigen(u)));
    }
    return y1;
  }

  StateVector CreateStateVector() const override
  {
    return std::make_pair(sys1_->CreateStateVector(), sys2_->CreateStateVector());
  }

  bool isTimeVarying() const override
  {
    return sys1_->isTimeVarying() || sys2_->isTimeVarying();
  }
  bool isDirectFeedthrough() const override
  {
    return sys1_->isDirectFeedthrough();
  }
  size_t getNumStates() const override
  {
    return Drake::getNumStates(*sys1_) + Drake::getNumStates(*sys2_);
  };
  size_t getNumInputs() const override
  {
    return Drake::getNumInputs(*sys1_);
  };
  size_t getNumOutputs() const override
  {
    return Drake::getNumOutputs(*sys1_);
  };

  static StateVector getInitialState(const FeedbackSystem<ScalarType, System1, System2>& sys)
  {
    return std::make_pair(getInitialState(*(sys.sys1_)), getInitialState(*(sys.sys2_)));
  }

private:
 void subsystemOutputs(const ScalarType& t, const typename System1::StateVector& x1, const typename System2::StateVector& x2,
                               const InputVector& u, OutputVector& y1, InputVector& y2)
  {
    if (!sys1_->isDirectFeedthrough())
    {
      y1 = sys1_->output(t, x1, u);  // output does not depend on u (so it's ok
                                     // that we're not passing u+y2)
      y2 = sys2_->output(t, x2, y1);
    }
    else
    {
      y2 = sys2_->output(t, x2, y1);  // y1 might be uninitialized junk, but has to be ok
      y1 = sys1_->output(t, x1, static_cast<InputVector>(toEigen(y2) + toEigen(u)));
    }
  }

  std::shared_ptr<System1> sys1_;
  std::shared_ptr<System2> sys2_;
};

/** feedback(sys1, sys2_)
 * @brief Convenience method to create a feedback combination of two systems
 * @ingroup modeling
 */
template <typename ScalarType, typename System1, typename System2>
std::shared_ptr<FeedbackSystem<ScalarType, System1, System2>> feedback(const std::shared_ptr<System1>& sys1,
                                                                       const std::shared_ptr<System2>& sys2_)
{
  return std::make_shared<FeedbackSystem<ScalarType, System1, System2>>(sys1, sys2_);
};

}  // namespace Drake 

#endif  // DRAKE_FEEDBACK_SYSTEM_H
