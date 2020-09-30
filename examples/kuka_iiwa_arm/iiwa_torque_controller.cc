/// @file
///
/// Implements a controller for a KUKA iiwa arm.

#include <cstdlib>
#include <iostream>
#include <memory>

#include "robotlocomotion/robot_plan_t.hpp"
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/examples/kuka_iiwa_arm/lcm_plan_interpolator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

using robotlocomotion::robot_plan_t;

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(interp_type, "cubic",
              "Robot plan interpolation type. Can be {zoh, foh, cubic, pchip}");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_command_channel, "IIWA_COMMAND",
              "Channel on which to publish lcmt_iiwa_command messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to listen for robot_plan_t messages.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using manipulation::planner::RobotPlanInterpolator;
using manipulation::planner::InterpolatorType;

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

// Create a system which has an integrator on the interpolated
// reference position for received plans.
int DoMain() {
  const std::string kLcmStatusChannel = FLAGS_lcm_status_channel;
  const std::string kLcmCommandChannel = FLAGS_lcm_command_channel;
  const std::string kLcmPlanChannel = FLAGS_lcm_plan_channel;
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  drake::log()->debug("Status channel: {}", kLcmStatusChannel);
  drake::log()->debug("Command channel: {}", kLcmCommandChannel);
  drake::log()->debug("Plan channel: {}", kLcmPlanChannel);

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));

  // Sets the robot plan interpolation type.
  std::string interp_str(FLAGS_interp_type);
  std::transform(interp_str.begin(), interp_str.end(),
                 interp_str.begin(), ::tolower);
  InterpolatorType interpolator_type{};
  if (interp_str == "zoh") {
    interpolator_type = InterpolatorType::ZeroOrderHold;
  } else if (interp_str == "foh") {
    interpolator_type = InterpolatorType::FirstOrderHold;
  } else if (interp_str == "cubic") {
    interpolator_type = InterpolatorType::Cubic;
  } else if (interp_str == "pchip") {
    interpolator_type = InterpolatorType::Pchip;
  } else {
    std::cerr <<
        "Robot plan interpolation type not recognized. "
        "Use the gflag --helpshort to display "
        "flag options for interpolator type.\n";
    return EXIT_FAILURE;
  }

  auto plan_interpolator =
      builder.AddSystem<LcmPlanInterpolator>(urdf, interpolator_type);
  const int kNumJoints = plan_interpolator->num_joints();

  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));
  plan_sub->set_name("plan_sub");

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
          kLcmCommandChannel, &lcm));
  command_pub->set_name("command_pub");

  // //------------------------------------- Beginning Torque Control -------------------------------------------
  // // Define desired states, here we choose 0 positions and 0 velocities.
  // VectorX<double> desired_state = VectorX<double>::Zeros(plan_interpolator->num_joints()*2);
  // auto desired_state_source =
  //     builder.AddSystem<systems::ConstantVectorSource<double>>(desired_state);
  // desired_state_source->set_name("constant_source");

  // drake::log()->info(
  //     "positions numbers: " + std::to_string(plan_interpolator->num_joints()) +
  //     ", velocities numbers: " + std::to_string(plan_interpolator->num_joints()) +
  //     ", actuators numbers: " + std::to_string(plan_interpolator->num_joints()));

  // // Create inverse dynamics controller.
  // const int U = plan_interpolator->num_joints();
  // auto IDC =
  //     builder
  //         .AddSystem<systems::controllers::InverseDynamicsController<double>>(
  //             plant, Eigen::VectorXd::Ones(U) * 2.0,
  //             Eigen::VectorXd::Ones(U) * 0.02,
  //             Eigen::VectorXd::Ones(U) * 1.0, false);
  // builder.Connect(IDC->get_output_port_control(),
  //                 plant.get_applied_generalized_force_input_port());
  // builder.Connect(plant.get_state_output_port(),
  //                 IDC->get_input_port_estimated_state());
  // builder.Connect(desired_state_source->get_output_port(),
  //                 IDC->get_input_port_desired_state());

  // // Constant zero load for plant actuation_input_port.
  // auto constant_zero_torque =
  //     builder.AddSystem<systems::ConstantVectorSource<double>>(
  //         Eigen::VectorXd::Zero(plant.num_actuators()));
  // builder.Connect(constant_zero_torque->get_output_port(),
  //                 plant.get_actuation_input_port());

  // DRAKE_DEMAND(!!plant.get_source_id());
  // builder.Connect(
  //     plant.get_geometry_poses_output_port(),
  //     scene_graph.get_source_pose_port(plant.get_source_id().value()));
  // builder.Connect(scene_graph.get_query_output_port(),
  //                 plant.get_geometry_query_input_port());

  // geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  // std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  // //------------------------------------- End Torque Control -------------------------------------------





  // Connect subscribers to input ports.
  builder.Connect(plan_sub->get_output_port(),
                  plan_interpolator->get_input_port_iiwa_plan());

  // Connect publisher to output port.
  // builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
  //                 command_pub->get_input_port());

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();
  auto& plan_interpolator_context =
      diagram.GetMutableSubsystemContext(*plan_interpolator, &diagram_context);

  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_iiwa_status");
  lcm::Subscriber<lcmt_iiwa_status> status_sub(&lcm, kLcmStatusChannel);
  LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });
  DRAKE_DEMAND(status_sub.message().num_joints == kNumJoints);

  // Initialize the context based on the first message.
  const double t0 = status_sub.message().utime * 1e-6;
  VectorX<double> q0(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    q0[i] = status_sub.message().joint_position_measured[i];
  }
  diagram_context.SetTime(t0);
  plan_interpolator->Initialize(t0, q0, &plan_interpolator_context);
  auto& status_value = plan_interpolator->get_input_port_iiwa_status().FixValue(
      &plan_interpolator_context, status_sub.message());

  // Run forever, using the lcmt_iiwa_status message to dictate when simulation
  // time advances.  The robot_plan_t message is handled whenever the next
  // lcmt_iiwa_status occurs.


//  auto constant_zero_torque =
//       builder.AddSystem<systems::ConstantVectorSource<double>>(
//           Eigen::VectorXd::Zero(kNumJoints));
  // auto constant_zero_torque =
  //     builder.AddSystem<systems::OutputPort<double>>(Eigen::VectorXd::Ones(kNumJoints));

  // Add a block to convert the desired position vector to iiwa command.
  auto command_sender = builder.AddSystem<IiwaCommandSender>(kNumJoints);
  command_sender->set_name("command_sender");

  // Connect the subsystems.
  builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
                  command_sender->get_position_input_port());
                 
  builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
                  command_sender-> get_torque_input_port());


  drake::log()->info("Controller started");
  
  while (true) {
    // Wait for an lcmt_iiwa_status message.
    status_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });
    // Write the lcmt_iiwa_status message into the context and advance.
    status_value.GetMutableData()->set_value(status_sub.message());
    
    const double time = status_sub.message().utime * 1e-6;
    simulator.AdvanceTo(time);
    // Force-publish the lcmt_iiwa_command (via the command_pub system within
    // the diagram).
    diagram.Publish(diagram_context);
  }

  // We should never reach here.
  return EXIT_FAILURE;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
