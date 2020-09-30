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
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"


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
DEFINE_double(max_time_step, 5.0e-3,
              "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using systems::Demultiplexer;
using systems::Multiplexer;
using systems::Sine;

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";


// Original Values
// DEFINE_double(Kp, 10.0, "Kp");
// DEFINE_double(Ki, 0.1, "Ki");
// DEFINE_double(Kd, 5.0, "Kd");

DEFINE_double(Kp, 2.0, "Kp");
DEFINE_double(Ki, 0.02, "Ki");
DEFINE_double(Kd, 1.0, "Kd");

// Create a system which has an integrator on the interpolated
// reference position for received plans.
int DoMain() {
  const std::string kLcmStatusChannel = FLAGS_lcm_status_channel;
  const std::string kLcmCommandChannel = FLAGS_lcm_command_channel;
  const std::string kLcmPlanChannel = FLAGS_lcm_plan_channel;
  
  systems::DiagramBuilder<double> builder;
  
  // auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  lcm::DrakeLcm lcm;

  drake::log()->debug("Status channel: {}", kLcmStatusChannel);
  drake::log()->debug("Command channel: {}", kLcmCommandChannel);
  

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));


  multibody::MultibodyPlant<double>& kuka_iiwa =
      *builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
  kuka_iiwa.set_name("plant");
  multibody::Parser parser(&kuka_iiwa);
  multibody::ModelInstanceIndex iiwa_instance =
      parser.AddModelFromFile(urdf);

  kuka_iiwa.WeldFrames(kuka_iiwa.world_frame(), kuka_iiwa.GetFrameByName("iiwa_link_0"));

  // Now the model is complete.
  kuka_iiwa.Finalize();

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
          kLcmCommandChannel, &lcm));
  command_pub->set_name("command_pub");

  
//----------------------------------------------------------------------

  // Declare a multiplaxer with 3 different inputs
  std::vector<int> input_sizes = {1,1,12};
  auto plant_des_state_mux = builder.AddSystem<Multiplexer>(input_sizes);
  plant_des_state_mux->set_name("plant_des_state_mux");
  
  VectorX<double> zer =
      VectorX<double>::Zero(1);
  auto zer_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(zer);
  zer_source->set_name("zer_source");

  VectorX<double> partial_desired_state =
      VectorX<double>::Zero(12);
  auto partial_desired_state_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(partial_desired_state);
  partial_desired_state_source->set_name("constant_partial_source");

  auto sine_wave = builder.AddSystem<Sine>(.5,0.2,0.0,1);
  sine_wave->set_name("sine_wave");

  builder.Connect(zer_source->get_output_port(),
                  plant_des_state_mux->get_input_port(0));
  builder.Connect(sine_wave->get_output_port(0),
                  plant_des_state_mux->get_input_port(1));
  builder.Connect(partial_desired_state_source->get_output_port(),
                  plant_des_state_mux->get_input_port(2));


  // Create inverse dynamics controller.
  const int U = kuka_iiwa.num_actuators();
  auto IDC =
      builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(
              kuka_iiwa, Eigen::VectorXd::Ones(U) * FLAGS_Kp,
              Eigen::VectorXd::Ones(U) * FLAGS_Ki,
              Eigen::VectorXd::Ones(U) * FLAGS_Kd, false);


  auto status_receiver = builder.AddSystem<IiwaStatusReceiver>(kuka_iiwa.num_joints());
  status_receiver->set_name("status_receiver");

  // auto status_sub = builder.AddSystem(
  //       systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
  //           "IIWA_STATUS", &lcm));
  // status_sub->set_name("status_sub");

  // builder.Connect(status_sub->get_output_port(),
  //                 status_receiver->get_input_port());

  
  // Multiplex the state of the manipulator joining position_measured and velocity_estimated
  std::vector<int> state_sizes = {7,7};
  auto state_mux = builder.AddSystem<Multiplexer>(state_sizes);
  state_mux->set_name("state_mux");

  builder.Connect(status_receiver->get_position_measured_output_port(),
                  state_mux->get_input_port(0));
  builder.Connect(status_receiver->get_velocity_estimated_output_port(),
                  state_mux->get_input_port(1));
  
  builder.Connect(state_mux->get_output_port(),
                  IDC->get_input_port_estimated_state()); 
  
  builder.Connect(plant_des_state_mux->get_output_port(),
                    IDC->get_input_port_desired_state());
  
  
  
  builder.Connect(IDC->get_output_port_control(),
                  kuka_iiwa.get_applied_generalized_force_input_port());

  //   builder.Connect(desired_state_source->get_output_port(),
  //                   IDC->get_input_port_desired_state());
  

  //----------------------------------------------------------------------








  // // Connect publisher to output port.
  // builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
  //                 command_pub->get_input_port());

  // // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();
  // auto& plan_interpolator_context =
  //     diagram.GetMutableSubsystemContext(*plan_interpolator, &diagram_context);

  // Wait for the first message.
  const int kNumJoints = 7;
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
  //     &plan_interpolator_context, status_sub.message());

  // Run forever, using the lcmt_iiwa_status message to dictate when simulation
  // time advances.  The robot_plan_t message is handled whenever the next
  // lcmt_iiwa_status occurs.
  // systems::Context<double>& plant_context =
      // diagram->GetMutableSubsystemContext(kuka_iiwa, diagram_context.get());
  Eigen::VectorXd q = kuka_iiwa.GetPositions(diagram_context, iiwa_instance);
  q[0] = 0.1;
  q[1] = 0.5;
  q[2] = 0.3;
  kuka_iiwa.SetPositions(&diagram_context, q);


  drake::log()->info("Controller started");
  
  while (true) {
    // Wait for an lcmt_iiwa_status message.
    status_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });
    // Write the lcmt_iiwa_status message into the context and advance.
    // status_value.GetMutableData()->set_value(status_sub.message());
    // drake::log()->info("status_sub: {}",status_value.GetMutableData);
    const double time = status_sub.message().utime * 1e-6;
    simulator.AdvanceTo(time);
    drake::log()->info("Controller time: {}", time);
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


