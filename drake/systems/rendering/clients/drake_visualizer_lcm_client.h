#pragma once

#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace rendering {
namespace clients {

lcmt_viewer_load_robot CreateLoadMessage(const RigidBodyTree<double>& tree);

}  // namespace clients
}  // namespace rendering
}  // namespace systems
}  // namespace drake
