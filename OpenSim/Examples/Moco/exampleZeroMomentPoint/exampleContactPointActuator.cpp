/* -------------------------------------------------------------------------- *
 *             OpenSim: exampleContactPointActuator.cpp                       *
 * -------------------------------------------------------------------------- *
 * Demonstrates the use of the ZeroMomentPointEstimator alongside the         *
 * various created Moco constraints to run different Moco problems, while     *
 * using the Fluit2014ContactPointActuator class to generate the external     *
 * forces.                                                                    *
 *                                                                            *
 * Please review the associated README.MD file for example specific details.  *
 *                                                                            *
 * @author  Aaron Fox                                                         *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Actuators/Fluit2014ContactPointActuator.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>

#include <OpenSim/Simulation/Control/PrescribedController.h>

// TODO: configure more generally to run different Moco elements
    // > Muscle-driven simulations don't seem to be working well/tuned nicely?
    // > Consider periodicity constraints?
	// > Configure to work with Fluit2014ContactPointActuator class

using namespace OpenSim;

// Helper: print a section header to console
static void printSection(const std::string& title) {
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "============================================================\n";
}

// Function for running the torque-driven tracking problem
void runTrackingStudy(Model model) {

    // Configure the Moco problem
    // ------------------------------
    std::string study_name = "torque_driven_no_grf_tracking_zmp";

    // Modify the model to prepare it for the tracking
    model.initSystem();
    ModelProcessor modelProcessor(model);
    // Weld locked joints
    modelProcessor.append(ModOpReplaceJointsWithWelds(
        {"subtalar_r", "subtalar_l", 
        "mtp_r", "mtp_l", 
        "radius_hand_r", "radius_hand_l"}));
    // Remove muscles
    modelProcessor.append(ModOpRemoveMuscles());
    // Add residual actuators to the model.
    // Inputs are rotational opt. force, translational opt. force and bounds
    // TODO: adjust these back down to 2.5 and 1.0 when working
    modelProcessor.append(ModOpAddResiduals(2.5, 1.0, 1.0));
    // Add CoordinateActuators to the non residual degrees of freedom.
    // Inputs are optimal force and bound
    modelProcessor.append(ModOpAddReserves(250.0, 1.0));

    // Create and name an instance of the MocoTrack tool.
    MocoTrack track;
    track.setName(study_name);

    // Set model in tracking tool
    track.setModel(modelProcessor);
    Model modelSolution = modelProcessor.process();

    // Construct a TableProcessor of the coordinate data and pass it to the
    // tracking tool. TableProcessors can be used in the same way as
    // ModelProcessors by appending TableOperators to modify the base table.
    // A TableProcessor with no operators, as we have here, simply returns the
    // base table.
    track.setStatesReference(TableProcessor("subject01_run3_states.sto"));

    // This setting allows extra data columns contained in the states
    // reference that don't correspond to model coordinates.
    track.set_allow_unused_references(true);

    // Set the global states tracking weight
    track.set_states_global_tracking_weight(1.0);

    // Initial time, final time, and mesh interval.
    // These are just set to match the initial states data
    double initialTime = 0.514;
    double finalTime = 1.231;
    track.set_initial_time(initialTime);
    track.set_final_time(finalTime);
    track.set_mesh_interval(0.01);
    
    // Apply tracked states to guess
    track.set_apply_tracked_states_to_guess(true);

    // Don't track the vertical position of the pelvis and only lightly track
    // the speed. Let the optimization determine the vertical position of the
    // model, which will make it easier to find the position of the feet that
    // leads to the best tracking of the kinematics and ground reaction forces.

    // TODO: needs to track lightly maybe?

    MocoWeightSet statesWeightSet;
    statesWeightSet.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/value", 0.0});
    statesWeightSet.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/speed", 0.1});
    track.set_states_weight_set(statesWeightSet);

    // Instead of calling solve(), call initialize() to receive a pre-configured
    // MocoStudy object based on the settings above. Use this to customize the
    // problem beyond the MocoTrack interface.
    MocoStudy study = track.initialize();

    // Get a reference to the MocoControlCost that is added to every MocoTrack
    // problem by default and set the overall weight to 0.01.
    MocoProblem& problem = study.updProblem();
    MocoControlGoal& effort =
            dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    effort.setWeight(0.01);

    // Put larger individual weights on the pelvis CoordinateActuators, which
    // act as the residual, or 'hand-of-god', forces which we would like to keep
    // as small as possible.
    // TODO: put this back in once working...
    effort.setWeightForControlPattern(".*pelvis.*", 10);

    // Low weight on contact actuators to avoid conflict with external forces
    effort.setWeightForControlPattern("/forceset/contact_actuator_.*", 1.0);  // TODO: setting low may allow their use at non-force times?

    // Set control info bounds for point actuators
    problem.setControlInfoPattern(
            "/forceset/contact_actuator_.*", MocoBounds(0.0, 1.0));

    // TODO: does the ground constraint fight the contact point actuators?

    //// Add constraint that prevents contact points penetrating ground plane.
    //auto* groundContactConstraint =
    //        problem.addPathConstraint<MocoContactPointConstraint>();
    //// Set the properties
    //groundContactConstraint->setName("no_ground_penetration");
    //groundContactConstraint->setGroundPlaneHeight(0.00);
    //// Add each contact point by its absolute path in the model tree.
    //// The path follows: /<estimator_name>/<side_name>/<point_name>
    //// Right side contact points
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/right_side/contact_point_RCAL");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/right_side/contact_point_RTOE");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/right_side/contact_point_RMT5");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/right_side/contact_point_RTOE_mid");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/right_side/contact_point_RMT5_mid");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/right_side/contact_point_R_end");
    //// Left side contact points
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/left_side/contact_point_LCAL");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/left_side/contact_point_LTOE");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/left_side/contact_point_LMT5");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/left_side/contact_point_LTOE_mid");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/left_side/contact_point_LMT5_mid");
    //groundContactConstraint->addContactPointPath(
    //        "/zmp_grf_estimator/left_side/contact_point_L_end");
    
    // Add and configure the ZMP path constraint
     auto* zmpConstraint =
     problem.addPathConstraint<MocoZeroMomentPointConstraint>();
     zmpConstraint->setName("zmp_ground_reactions_constraint");
     zmpConstraint->set_zmp_estimator_path("/zmp_grf_estimator");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_RCAL");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_RTOE");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_RMT5");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_RTOE_mid");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_RMT5_mid");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_R_end");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_LCAL");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_LTOE");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_LMT5");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_LTOE_mid");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_LMT5_mid");
     zmpConstraint->append_actuator_paths("/forceset/contact_actuator_L_end");

    // TODO: customised GRF tracking for this?

    // Set coordinate bounds in the problem
    // This adds bounds around coordinate max and min based on 20% of 
    // the coordinate value range
    // Get model, coordinate and states information
    const TimeSeriesTable statesTable =
            TimeSeriesTable("subject01_run3_states.sto");
    const auto& coordSet = modelSolution.getCoordinateSet();
    int numCoords = coordSet.getSize();
    // Loop through coordinates to set bounds
    for (int i = 0; i < numCoords; ++i) {
        // Get coordinate path
        std::string coordPath = coordSet.get(i).getAbsolutePathString();
        // Set the corresponding state value column name
        std::string statePath = coordPath + "/value";
        // Extract the vector of tracking data for this coordinate
        auto statesData = statesTable.getDependentColumn(statePath);
        // Find the absolute min and max values within the trajectory
        double dataMin = SimTK::min(statesData);
        double dataMax = SimTK::max(statesData);
        double range = dataMax - dataMin;
        // Calculate the 20% bounds adjustments
        double lowerBound = dataMin - (0.20 * range);
        double upperBound = dataMax + (0.20 * range);
        // Apply the updated bounds to the state variable in Moco
        problem.setStateInfo(statePath, {lowerBound, upperBound});
    }

    // Customise the solver settings
    // ------------------------------
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_transcription_scheme("legendre-gauss-radau-3"); // TODO: use this? Need to update guess if doing?
    solver.set_optim_convergence_tolerance(1e-3);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.resetProblem(problem);

    // Print problem for reference
    problem.print("contact-point-example-problem.omoco");
    
    // Solve!
    // ------------------------------
    MocoSolution solution = study.solve();
    
    // Write solution to file
    // Note that this won't work if the problem doesn't solver properly
    solution.write(fmt::format("{}_solution.sto", study_name));

    //MocoTrajectory solution =
    //        MocoTrajectory(fmt::format("{}_solution.sto", study_name));

    // Print the model.
    modelSolution.initSystem();
    modelSolution.print(fmt::format("{}_model.osim", study_name));

    // Extract ground reaction forces from contact point actuators
    // TODO: could efficiently do this in a loop
    std::vector<std::string> contact_r;
    std::vector<std::string> contact_l;
    contact_r.push_back("/forceset/contact_actuator_RCAL");
    contact_r.push_back("/forceset/contact_actuator_RTOE");
    contact_r.push_back("/forceset/contact_actuator_RMT5");
    contact_r.push_back("/forceset/contact_actuator_RTOE_mid");
    contact_r.push_back("/forceset/contact_actuator_RMT5_mid");
    contact_r.push_back("/forceset/contact_actuator_R_end");
    contact_l.push_back("/forceset/contact_actuator_LCAL");
    contact_l.push_back("/forceset/contact_actuator_LTOE");
    contact_l.push_back("/forceset/contact_actuator_LMT5");
    contact_l.push_back("/forceset/contact_actuator_LTOE_mid");
    contact_l.push_back("/forceset/contact_actuator_LMT5_mid");
    contact_l.push_back("/forceset/contact_actuator_L_end");
    TimeSeriesTable exLoads = createExternalLoadsTableFromContactPointActuators(
            modelSolution, solution, contact_r, contact_l);
    STOFileAdapter::write(exLoads, 
        fmt::format("{}_ground_reactions.sto", study_name));

    // TODO: extracting ground reactions from contact point actuators...
    // This would probably use a force reporter approach combined with a utility
    // in a similar way to the contact spheres approach

} // end of runTrackingProblem()

// Run main function
int main() {

    
    // ======================================================================
    // STEP 1: Create the model with contact point actuator components
    // ======================================================================
    printSection("Step 1. Adding contact point actuator components to model");

    /// Model preparation.
    /// -----------------
    /// Update the model to prepare it for tracking optimization. This adds 
    /// the actuator components to use in the tracking simulations.
    
    // Load the model
    Model model("subject01_run3.osim");
    model.initSystem();

    // Create the component with default properties
    auto* zmp = new ZeroMomentPointEstimator();
    zmp->setName("zmp_grf_estimator");
    zmp->setFreeJointName("ground_pelvis");
    zmp->setContactForceThreshold(50.0);

    // Create the contact sides
    const PhysicalFrame& calcn_r = model.getBodySet().get("calcn_r");
    const PhysicalFrame& calcn_l = model.getBodySet().get("calcn_l");
    auto* rightContactSide = new ContactSide("right_side", "calcn_r");
    auto* leftContactSide = new ContactSide("left_side", "calcn_l");

    // To get the contact point locations we look for the marker position
    // locations in the model. Here a list is set for the markers to look for.
    // Right side
    Array<std::string> contactPointsRight;
    contactPointsRight.append("RCAL");
    contactPointsRight.append("RTOE");
    contactPointsRight.append("RMT5");
    contactPointsRight.append("RTOE_mid");
    contactPointsRight.append("RMT5_mid");
    contactPointsRight.append("R_end");
    // Left side
    Array<std::string> contactPointsLeft;
    contactPointsLeft.append("LCAL");
    contactPointsLeft.append("LTOE");
    contactPointsLeft.append("LMT5");
    contactPointsLeft.append("LTOE_mid");
    contactPointsLeft.append("LMT5_mid");
    contactPointsLeft.append("L_end");

    // Loop through the right side contact points, get the marker locations
    // and then create the contact actuators for these points. Note that all
    // markers are attached to the model via the calcaneus bodies, even though
    // that they are more applicable to the toes bodies. Contact point locations
    // are also added to the ZMP estimator here.
    // Right side
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker = model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point actuator
        Fluit2014ContactPointActuator* conAct =
                new Fluit2014ContactPointActuator(
                        "contact_actuator_" + contactName, "calcn_r");
        // Set the parameters
        conAct->set_point(pointLocation);
        conAct->set_max_force(1000.0);
        conAct->set_ground_height(0.0);
        conAct->set_contact_height_threshold(0.02);
        //conAct->set_contact_velocity_threshold(1.6);
        conAct->set_use_velocity_in_scaling(false);
        conAct->set_smoothing(8.0); // 
        conAct->set_friction_coefficient(0.5);
        conAct->set_min_control(0.0);
        conAct->set_max_control(1.0);
        // Add the contact point to the model
        model.addForce(conAct);
        // Build the contact point
        ContactPoint* cp = new ContactPoint(
                "contact_point_" + contactName, calcn_r, pointLocation);
        cp->set_distance_threshold(0.02);
        cp->set_contact_checking_method("distance");
        // Add the contact point to the contact side
        rightContactSide->addContactPoint(cp);
    }
    // Left side
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker = model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point actuator
        Fluit2014ContactPointActuator* conAct =
                new Fluit2014ContactPointActuator(
                        "contact_actuator_" + contactName, "calcn_l");
        // Set the parameters
        conAct->set_point(pointLocation);
        conAct->set_max_force(1000.0);
        conAct->set_ground_height(0.0);
        conAct->set_contact_height_threshold(0.02);
        //conAct->set_contact_velocity_threshold(1.6);
        conAct->set_use_velocity_in_scaling(false);
        conAct->set_smoothing(8.0);
        conAct->set_friction_coefficient(0.5);
        conAct->set_min_control(0.0);
        conAct->set_max_control(1.0);
        // Add the contact point to the model
        model.addForce(conAct);
        // Build the contact point
        ContactPoint* cp = new ContactPoint(
                "contact_point_" + contactName, calcn_l, pointLocation);
        cp->set_distance_threshold(0.02);
        cp->set_contact_checking_method("distance");
        // Add the contact point to the contact side
        leftContactSide->addContactPoint(cp);
    }

    // Add contact sides to estimator
    zmp->addContactSide(rightContactSide);
    zmp->addContactSide(leftContactSide);

    // Add estimator component to model
    model.addComponent(zmp);

    // Finalize model connections
    model.finalizeConnections();

    // ======================================================================
    // STEP 2: Run a torque-driven states tracking problem
    // ======================================================================
    printSection("Step 2. Running a torque-driven states tracking example");

    // Solve the torque-driven states tracking problem with GRF tracking
    runTrackingStudy(model);

    return EXIT_SUCCESS;
	
}
