/// @file
///
/// Demo of moving the iiwa's end effector in cartesian space.  This
/// program creates a plan to move the end effector from the current
/// position to the location specified on the command line.  The
/// current calculated position of the end effector is printed before,
/// during, and after the commanded motion.

#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"


#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsers/sdf_parser.h"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");

DEFINE_double(x, 0., "x coordinate to move to");
DEFINE_double(y, 0., "y coordinate to move to");
DEFINE_double(z, 0., "z coordinate to move to");
DEFINE_double(roll, 0., "target roll about world x axis for end effector");
DEFINE_double(pitch, 0., "target pitch about world y axis for end effector");
DEFINE_double(yaw, 0., "target yaw about world z axis for end effector");
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

DEFINE_string(kLcmCommandChannel, "IIWA_COMMAND", "Command channel");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {


using manipulation::planner::ConstraintRelaxingIk;
using manipulation::kuka_iiwa::kIiwaArmNumJoints;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::ModelInstanceInfo;
using systems::RigidBodyPlant;


const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";



class MoveDemoRunner {
 public:
    lcmt_iiwa_status iiwa_status_;
    lcm::LCM lcm_;
    std::string urdf_;
    RigidBodyTree<double> kukaTree_;
    std::unique_ptr<RigidBodyTree<double>> totalTree_;

    VectorX<double> states;
    Eigen::VectorXd iiwa_q;
    Eigen::VectorXd iiwa_v;
    std::unique_ptr<WorldSimTreeBuilder<double>> tree_builder; 
    std::unique_ptr<RigidBodyPlant<double>> plant;

    int counter = 0;
    int waypoint_counter = 1;

    bool controller_trigger_;// control runner wait for the first message from plan runner 


    //This stuff is to set up iiwa world sim file. This has been deprecated in the latest drake version
  MoveDemoRunner() {
    urdf_ =
        (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));

    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf_, multibody::joints::kFixed, &kukaTree_);

    lcm_.subscribe(FLAGS_lcm_status_channel,
                   &MoveDemoRunner::HandleStatus, this);

    iiwa_q = Eigen::VectorXd(7);
    iiwa_v = Eigen::VectorXd(7);

    tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
    tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);

    const Eigen::Vector3d kRobotBase(0, 0, 0);

    ModelInstanceInfo<double> iiwa_instance;

    int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
    iiwa_instance = tree_builder->get_model_info_for_instance(id);

    

    totalTree_ = std::move(tree_builder->Build());

  }


  // Handle the incoming status message from the iiwa.  This is a
  // fairly high rate operation (200Hz is typical).  The plan is
  // calculated on the first status message received, after that
  // periodically display the current position of the end effector.
    void Run() {

        lcmt_iiwa_command iiwa_command;
        iiwa_command.num_joints = kIiwaArmNumJoints;
        iiwa_command.joint_position.resize(kIiwaArmNumJoints, 0.);
        iiwa_command.num_torques = kIiwaArmNumJoints;
        iiwa_command.joint_torque.resize(kIiwaArmNumJoints, 0.);
        iiwa_command.utime = iiwa_status_.utime;
        
        MatrixX<double> q_mat = CreateWaypoint();
        
        drake::log()->info("q_mat size: {}", q_mat.row(0).size());
        while (true) {

            // Call lcm handle until at least one status message is
            // processed.
            while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { /*Leave Blank*/ }

            //Updating time
            iiwa_command.utime = iiwa_status_.utime;
            if(waypoint_counter==q_mat.row(0).size())
            {
                waypoint_counter = q_mat.row(0).size()-1;
            }
            drake::log()->info("waypoint_counter: {}", waypoint_counter);
            VectorX<double>  PID_output = JointPID(q_mat.col(waypoint_counter));
            //Setting new positions and torques
            for (int joint = 0; joint < kIiwaArmNumJoints; joint++) {
                iiwa_command.joint_position[joint] = iiwa_q[joint];
                iiwa_command.joint_torque[joint] = PID_output[joint]; //CalculateKukaAndSchunkTorque()[joint];
            }
            //Below simulates an external "push" at a certain time span
            // if(counter > 450 && counter < 850){
            //     //Apply torque on joint 1    
            //     iiwa_command.joint_torque[0] = 10; //Overwrite the joint torque at 1 and instead apply a torque of value 20;
            //     iiwa_command.joint_torque[1] = 10;
            //     iiwa_command.joint_torque[2] = 5;
            //     iiwa_command.joint_torque[3] = 5;
            //     iiwa_command.joint_torque[4] = 5;
            //     iiwa_command.joint_torque[5] = 2;
            //     iiwa_command.joint_torque[6] = 2;
            // }
            // if(counter > 550 && counter < 650){
            //         //Apply torque on joint 1    
            //         iiwa_command.joint_torque[1] = 0; //Overwrite the joint torque at 1 and instead apply a torque of value 20;
            //         iiwa_command.joint_torque[5] = -2;
            // }
            //  for (int i = 0; i < kIiwaArmNumJoints; i++) {
            //     drake::log()->info("Joint: {} - Torque: {}",
            //         i,
            //         iiwa_command.joint_torque[i]);
            //  }

            //Publish the commands to the simulation                              
            lcm_.publish(FLAGS_kLcmCommandChannel, &iiwa_command);

        }
    }

 private:
    void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
                        
        iiwa_status_ = *status;
        counter++;      

        
        for (int i = 0; i < kIiwaArmNumJoints; i++) {
            // iiwa_v[i] = status->joint_velocity_estimated[i];
            // drake::log()->info("status joint position : {} - {}",i,status->joint_position_measured[i]);
            iiwa_q[i] = status->joint_position_measured[i];
        }
    }

    //This function compensates for the gravitational acceleration of the kuka arm and schunk gripper
    VectorX<double> CalculateWholeRobotTorque(){

        // Desired acceleration input.
        VectorX<double> desired_vd = VectorX<double>::Zero(7);

        //Create a kinematic cache
        KinematicsCache<double> cache = totalTree_->CreateKinematicsCache();

        // Sets velocity to zero in pure gravity compensation.
        cache.initialize(iiwa_q, VectorX<double>::Zero(7)); 

        totalTree_->doKinematics(cache, true);

        //Something to do with wrenches
        eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;

        //Set false for doing only gravity comp
        VectorX<double> torque = totalTree_->inverseDynamics(
            cache, f_ext, desired_vd, false);

        return torque;
    }

    //This function compensates for the kuka's weight only. No Schunk 
    //Takes in states(14) (7 positions, 7 velocities) and outputs a force(7)
    VectorX<double> CalculateKukaTorque() {

        // Desired acceleration input.
        VectorX<double> desired_vd = VectorX<double>::Zero(kIiwaArmNumJoints);
        
        //Create a kinematic cache
        KinematicsCache<double> cache = kukaTree_.CreateKinematicsCache();

        // Sets velocity to zero in pure gravity compensation.
        cache.initialize(iiwa_q, VectorX<double>::Zero(kIiwaArmNumJoints)); 

        kukaTree_.doKinematics(cache, true);


        // --------------------------------------------------------------------------------------------
        // ----------------------------------------Testing---------------------------------------------
        KinematicsCache<double> cache_goal = kukaTree_.CreateKinematicsCache();
        // Eigen::VectorXd q_goal= VectorX<double>::Zero(7);
        Eigen::VectorXd q_goal= VectorX<double>::Zero(7);
        q_goal << 0,0,0,0,0,1,0;
        cache_goal.initialize(q_goal, VectorX<double>::Zero(kIiwaArmNumJoints)); 
        kukaTree_.doKinematics(cache_goal);
        drake::log()->info("Cache Goal : {}",cache_goal.getX());
        const RigidBody<double>* end_effector = kukaTree_.FindBody(FLAGS_ee_name);
        
        const math::RigidTransform<double> ee_pose(kukaTree_.CalcBodyPoseInWorldFrame(cache_goal, *end_effector));
        const math::RollPitchYaw<double> rpy(ee_pose.rotation());
        drake::log()->info("End effector at: {} {}",
                            ee_pose.translation().transpose(),
                            rpy.vector().transpose());




        // --------------------------------------------------------------------------------------------
        //Something to do with wrenches
        eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;

        //Set false for doing only gravity comp
        VectorX<double> torque = kukaTree_.inverseDynamics(
            cache, f_ext, desired_vd, false);

        return torque;
    } 

    //Since the kuka already has its own gravity comp built in, you have to subtract out the arm's gravity comp.
    VectorX<double> CalculateSchunkAndConnectorTorque(){
      return CalculateWholeRobotTorque() - CalculateKukaTorque();
    }
    
    VectorX<double> JointPID(VectorX<double> q_mat){
      
        // Desired acceleration input.
        VectorX<double> desired_vd = VectorX<double>::Zero(kIiwaArmNumJoints);
        
        //Create a kinematic cache for the robot in the current position
        KinematicsCache<double> cache = kukaTree_.CreateKinematicsCache();

        // Sets velocity to zero in pure gravity compensation.
        cache.initialize(iiwa_q, VectorX<double>::Zero(kIiwaArmNumJoints)); 
        
        KinematicsCache<double> cache_goal = kukaTree_.CreateKinematicsCache();
        Eigen::VectorXd q_goal= q_mat;
        drake::log()->info("q_goal: {}",q_goal);
        // q_goal << 0,0.708,1.57,1.57,0,1.57,0;
        cache_goal.initialize(q_goal, VectorX<double>::Zero(kIiwaArmNumJoints)); 
        
        VectorX<double> max_torque = VectorX<double>::Zero(kIiwaArmNumJoints);
        max_torque << 176, 176, 110, 110, 110, 40, 40;
        double frac_torque = 0.25;
        max_torque = max_torque*frac_torque;
        double kp = 30;

        VectorX<double> torque =  VectorX<double>::Zero(kIiwaArmNumJoints);
        VectorX<double> error = VectorX<double>::Zero(kIiwaArmNumJoints);

        for (int i = 0; i < kIiwaArmNumJoints; i++) {
            error[i] = q_goal[i]-iiwa_q[i];
            // drake::log()->info("Joint: {} - Error: {}",
            //     i,
            //     error);
            double p = error[i]*kp;
            if(p > max_torque[i]){
                p = max_torque[i];
            }
            if(p < -max_torque[i]){
                p = -max_torque[i];
            }
            torque[i] = p;
        }
        drake::log()->info("ERROR Norm = {}",error.norm());
        if(error.norm()<0.3){
            waypoint_counter++;
        }

        return torque;
    }

    MatrixX<double> CreateWaypoint(){
      
        // Create waypoint
        double x = 0.500;
        double y = -0.200;
        double z = 0.900;
        double roll = -1.88;
        double pitch = 1.5;
        double yaw = -1.466;
            
        ConstraintRelaxingIk ik(
            urdf_, FLAGS_ee_name, Isometry3<double>::Identity());

        // Create a single waypoint for our plan (the destination).
        // This results in a trajectory with two knot points (the
        // current pose (read from the status message currently being
        // processes and passed directly to PlanSequentialTrajectory as
        // iiwa_q) and the calculated final pose).
        std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
        ConstraintRelaxingIk::IkCartesianWaypoint wp;
        const math::RollPitchYaw<double> rpy(roll, pitch, yaw);
        for (int i = 0; i < 400; i++) {
            wp.pose.translation() = Eigen::Vector3d(x, y+double(i)/1000, z-double(i)/2000);
            wp.pose.linear() = rpy.ToMatrix3ViaRotationMatrix();
            wp.constrain_orientation = false;
            waypoints.push_back(wp);
        }
        IKResults ik_res;
        ik.PlanSequentialTrajectory(waypoints, iiwa_q, &ik_res);
        drake::log()->info("IK result: {}", ik_res.info[0]);

        if (ik_res.info[0] == 1) {
            drake::log()->info("IK sol size {}", ik_res.q_sol.size());

            // Run the resulting plan over 2 seconds (which is a fairly
            // arbitrary choice).  This may be slowed down if executing
            // the plan in that time would exceed any joint velocity
            // limits.
            // std::vector<double> times{0, 2};
            // DRAKE_DEMAND(ik_res.q_sol.size() == times.size());

            // Convert the resulting waypoints into the format expected by
            // ApplyJointVelocityLimits and EncodeKeyFrames.
            MatrixX<double> q_mat(ik_res.q_sol.front().size(), ik_res.q_sol.size());
            
            for (size_t i = 0; i < ik_res.q_sol.size(); ++i) {
            // drake::log()->info("i = {}", i);
            q_mat.col(i) = ik_res.q_sol[i];
            drake::log()->info("q_mat {}", q_mat.col(i)*180/3.14);
            }

            return q_mat;

        }
        MatrixX<double> q_mat;
        return q_mat;
    }



};




}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
 // drake::logging::HandleSpdlogGflags();
  drake::examples::kuka_iiwa_arm::MoveDemoRunner runner;
  runner.Run();
}


