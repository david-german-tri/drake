#ifndef DRAKE_CASCADE_SYSTEM_H
#define DRAKE_CASCADE_SYSTEM_H

namespace Drake {

/** CascadeSystem<ScalarType, System1,System2>
 * @brief Builds a new system from the cascade connection of two simpler systems
 * @concept{system_concept}
 *
 * ![Cascade combination of two systems](http://underactuated.csail.mit.edu/figures/cascade_system.svg)
 *
 */
template <class ScalarType, class System1, class System2>
class CascadeSystem
    : public BaseSystem<ScalarType, std::pair<typename System1::StateVector, typename System2::StateVector>,
                             typename System1::InputVector, typename System2::OutputVector>
{
public:
  static_assert(std::is_same<typename System1::OutputVector, typename System2::InputVector>::value,
                "System 2 input vector must match System 1 output vector");

  using StateVector = typename std::pair<typename System1::StateVector, typename System2::StateVector>;
  using InputVector = typename System1::InputVector;
  using OutputVector = typename System2::OutputVector;

  CascadeSystem(std::shared_ptr<System1> sys1, std::shared_ptr<System2> sys2) : sys1_(sys1), sys2_(sys2){};

  StateVector dynamics(const ScalarType& t, const StateVector& x, const InputVector& u) override
  {
    const auto& x1 = x.first;
    const auto& x2 = x.second;
    typename System1::OutputVector y1 = sys1_->output(t, x1, u);
    StateVector xdot = std::make_pair(sys1_->dynamics(t, x1, u), sys2_->dynamics(t, x2, y1));
    return xdot;
  }

  OutputVector output(const ScalarType& t, const StateVector& x, const InputVector& u) override
  {
    const auto& x1 = x.first;
    const auto& x2 = x.second;
    typename System1::OutputVector y1 = sys1_->output(t, x1, u);
    OutputVector y2 = sys2_->output(t, x2, y1);
    return y2;
  }

  bool isTimeVarying() const override
  {
    return sys1_->isTimeVarying() || sys2_->isTimeVarying();
  }
  bool isDirectFeedthrough() const override
  {
    return sys1_->isDirectFeedthrough() && sys2_->isDirectFeedthrough();
  }
  size_t getNumStates() const override
  {
    return Drake::getNumStates(*sys1_) + Drake::getNumStates(*sys2_);
  }
  size_t getNumInputs() const override
  {
    return Drake::getNumInputs(*sys1_);
  }
  size_t getNumOutputs() const override
  {
    return Drake::getNumOutputs(*sys2_);
  }

  static StateVector getInitialState(const CascadeSystem<ScalarType, System1, System2>& sys)
  {
    return std::make_pair(getInitialState(*(sys.sys1_)), getInitialState(*(sys.sys2_)));
  }

private:
  std::shared_ptr<System1> sys1_;
  std::shared_ptr<System2> sys2_;
};

/** cascade(sys1, sys2_)
 * @brief Convenience method to create a cascade combination of two systems
 * @ingroup modeling
 */
template <typename ScalarType, typename System1, typename System2>
std::shared_ptr<CascadeSystem<ScalarType, System1, System2>> cascade(std::shared_ptr<System1> sys1,
                                                                     std::shared_ptr<System2> sys2)
{
  return std::make_shared<CascadeSystem<ScalarType, System1, System2>>(sys1, sys2);
};

}  // namespace Drake

#endif  // DRAKE_CASCADE_SYSTEM_H