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
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/framework/basic_vector.h"

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
using systems::Demultiplexer;
using systems::Multiplexer;
using systems::Sine;
using systems::lcm::SerializerInterface;

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

  // auto plan_interpolator =
  //     builder.AddSystem<LcmPlanInterpolator>(urdf, interpolator_type);
  const int kNumJoints = 7;//plan_interpolator->num_joints();

  std::vector<int> input_sizes = {1,1,5};
  auto plant_des_state_mux = builder.AddSystem<Multiplexer>(input_sizes);
  plant_des_state_mux->set_name("plant_des_state_mux");
  
  VectorX<double> zer =
      VectorX<double>::Zero(1);
  auto zer_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(zer);
  zer_source->set_name("zer_source");

  VectorX<double> partial_desired_state =
      VectorX<double>::Zero(5);
  auto partial_desired_state_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(partial_desired_state);
  partial_desired_state_source->set_name("constant_partial_source");

  auto sine_wave = builder.AddSystem<Sine>(.5,0.2,0.0,1);
  sine_wave->set_name("sine_wave");

  builder.Connect(zer_source->get_output_port(),
                  plant_des_state_mux->get_input_port(0));
  builder.Connect(sine_wave->get_output_port(0),
                  plant_des_state_mux->get_input_port(1));
// builder.Connect(zer_source->get_output_port(),
//                   plant_des_state_mux->get_input_port(1));
  builder.Connect(partial_desired_state_source->get_output_port(),
                  plant_des_state_mux->get_input_port(2));


  VectorX<double> zero_pos =
      VectorX<double>::Zero(7);
  auto zero_pos_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(zero_pos);
  zero_pos_source->set_name("zero_pos_source");


  auto command_sender = builder.AddSystem<IiwaCommandSender>(kNumJoints);
  command_sender->set_name("command_sender");

  auto command_pub = builder.AddSystem(
    systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
        kLcmCommandChannel, &lcm));
  command_pub->set_name("command_pub");


  builder.Connect(zero_pos_source->get_output_port(),
                  command_sender->get_position_input_port());
  builder.Connect(plant_des_state_mux->get_output_port(),
                  command_sender->get_torque_input_port());
  builder.Connect(command_sender->get_output_port(),
                  command_pub->get_input_port());

  // Connect publisher to output port.
  // builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
  //                 command_pub->get_input_port());

  // auto plan_sub =
  //     builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
  //         kLcmPlanChannel, &lcm));
  // plan_sub->set_name("plan_sub");

  // auto command_pub = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
  //         kLcmCommandChannel, &lcm));
  // command_pub->set_name("command_pub");

  // // Connect subscribers to input ports.
  // builder.Connect(plan_sub->get_output_port(),
  //                 plan_interpolator->get_input_port_iiwa_plan());

  // // Connect publisher to output port.
  // builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
  //                 command_pub->get_input_port());

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  // const auto& diagram = *owned_diagram;
  systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();
  // auto& plan_interpolator_context =
  //     diagram.GetMutableSubsystemContext(*plan_interpolator, &diagram_context);

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
  // plan_interpolator->Initialize(t0, q0, &plan_interpolator_context);
  // auto& status_value = plan_interpolator->get_input_port_iiwa_status().FixValue(
      // &plan_interpolator_context, status_sub.message());

  // Run forever, using the lcmt_iiwa_status message to dictate when simulation
  // time advances.  The robot_plan_t message is handled whenever the next
  // lcmt_iiwa_status occurs.
  drake::log()->info("Controller started");
  
  while (true) {
    // Wait for an lcmt_iiwa_status message.
    status_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });
    // Write the lcmt_iiwa_status message into the context and advance.
    // status_value.GetMutableData()->set_value(status_sub.message());
    // drake::log()->info("status_sub: {}",status_value.GetMutableData);
    const double time = status_sub.message().utime * 1e-6;
    drake::log()->info("status_sub: {}", time);
    simulator.AdvanceTo(time);
    // Force-publish the lcmt_iiwa_command (via the command_pub system within
    // the diagram).
    // diagram.Publish(diagram_context);
    const int kIiwaArmNumJoints = 7;
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kIiwaArmNumJoints;
    iiwa_command.joint_position.resize(kIiwaArmNumJoints, 0.);
    iiwa_command.num_torques = kIiwaArmNumJoints;
    iiwa_command.joint_torque.resize(kIiwaArmNumJoints, 0.);
    iiwa_command.utime = status_sub.message().utime;
    for (int joint = 0; joint < kIiwaArmNumJoints; joint++) {
        iiwa_command.joint_position[joint] = status_sub.message().joint_position_measured[joint];
        iiwa_command.joint_torque[joint] = 0;
    }
    VectorX<double> aux1 = VectorX<double>::Zero(7);
    // command_sender->get_input_port(0).FixValue(diagram_context,aux1);
    //drake::log()->info("status_sub: {}", diagram.GetSystemIndexOrAbort(command_sender));
    // auto& station_context = owned_diagram.GetMutableSubsystemContext(diagram.GetSystemIndexOrAbort(command_sender));
    // command_sender.GetInputPort("iiwa_position").FixValue(station_context, aux1)
    // command_sender->get_input_port(0).get_system();
    // auto& station_context = owned_diagram.GetMutableSubsystemContext(command_sender->get_input_port(0).get_system(), simulator.get_mutable_context())
    command_sender->get_input_port(0).FixValue(&diagram_context,aux1);
    drake::log()->info("size: {}", command_sender->get_input_port(0).size());
    // const AbstractValue& input = get_input_port().Eval<AbstractValue>(context);
    // std::vector<uint8_t> message_bytes;
    // std::unique_ptr<SerializerInterface> serializer;
     
    
    // serializer->Serialize(iiwa_command, &message_bytes);

    // Publishes onto the specified LCM channel.
    // lcm.Publish(kLcmCommandChannel, iiwa_command, 15);
    // lcm.publish(kLcmCommandChannel, &iiwa_command);
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
