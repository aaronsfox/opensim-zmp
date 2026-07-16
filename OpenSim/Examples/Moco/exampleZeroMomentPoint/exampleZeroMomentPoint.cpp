/* -------------------------------------------------------------------------- *
 *            OpenSim: exampleZeroMomentPoint.cpp                             *
 * -------------------------------------------------------------------------- *
 * Demonstrates the use of the ZeroMomentPointEstimator alongside the         *
 * various created Moco constraints to run different Moco problems.           *
 *                                                                            *
 * Please review the associated README.MD file for example specific details.  *
 *                                                                            *
 * @author  Aaron Fox                                                         *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Moco/MocoZeroMomentPointConstraint.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Actuators/BodyWrenchActuator.h>

// TODO: configure more generally to run different Moco elements
    // > Muscle-driven simulations don't seem to be working well/tuned nicely?
    // > Consider periodicity constraints?

using namespace OpenSim;

// Helper: print a section header to console
static void printSection(const std::string& title) {
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "============================================================\n";
}

// Function for running the torque-driven tracking problem
void runTrackingStudy(Model model, bool muscleDriven, bool trackGRF) {

    // Configure the Moco problem
    // ------------------------------
    std::string study_name;
    if (muscleDriven && trackGRF) {
        study_name = "muscle_driven_with_grf_tracking_zmp";
    } else if (muscleDriven && !trackGRF) {
        study_name = "muscle_driven_no_grf_tracking_zmp";
    } else if (!muscleDriven && trackGRF) {
        study_name = "torque_driven_with_grf_tracking_zmp";
    } else {
        study_name = "torque_driven_no_grf_tracking_zmp";
    }

    // Modify the model to prepare it for the tracking
    model.initSystem();
    ModelProcessor modelProcessor(model);
    // Weld locked joints
    modelProcessor.append(ModOpReplaceJointsWithWelds(
        {"subtalar_r", "subtalar_l", 
        "mtp_r", "mtp_l", 
        "radius_hand_r", "radius_hand_l"}));
    if (muscleDriven) {
        // Alter muscle parameters to suit tracking optimisation
        modelProcessor.append(ModOpIgnoreTendonCompliance());
        modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
        modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
        modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    } else {
        // Remove muscles
        modelProcessor.append(ModOpRemoveMuscles());
        // Add residual actuators to the model.
        // Inputs are rotational opt. force, translational opt. force and bounds
        modelProcessor.append(ModOpAddResiduals(2.5, 1.0, 1.0));
        // Add CoordinateActuators to the non residual degrees of freedom.
        // Inputs are optimal force and bound
        modelProcessor.append(ModOpAddReserves(250.0, 1.0));
    }

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
    if (muscleDriven) {
        // Use a longer mesh interval to speed up computation
        track.set_mesh_interval(0.03);
    } else {
        track.set_mesh_interval(0.01);
    }
    

    // Apply tracked states to guess
    track.set_apply_tracked_states_to_guess(true);

    // Don't track the vertical position of the pelvis and only lightly track
    // the speed. Let the optimization determine the vertical position of the
    // model, which will make it easier to find the position of the feet that
    // leads to the best tracking of the kinematics and ground reaction forces.
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
    effort.setWeightForControlPattern(".*pelvis.*", 10);

    // No weight on body actuators to avoid conflict with tracking goal
    effort.setWeightForControlPattern("/forceset/force_r_.*", 0.0);
    effort.setWeightForControlPattern("/forceset/force_l_.*", 0.0);

    // Define the optimisation bounds for the body actuator inputs
    // Restrict bounds to reasonable values for force, COP and torque
    // Note that these are reasonable for the experimental data included
    // with this example, but might need to change for other scenarios.
    // Right foot control signals
    // Force channels (0, 1, 2)
    problem.setControlInfo(
            "/forceset/force_r_0", MocoBounds(-300.0, 300.0)); // Fx
    problem.setControlInfo(
            "/forceset/force_r_1", MocoBounds(0.0, 2000.0)); // Fy
    problem.setControlInfo(
            "/forceset/force_r_2", MocoBounds(-75.0, 75.0)); // Fz
    // Center of Pressure channels (3, 4, 5)
    problem.setControlInfo(
            "/forceset/force_r_3", MocoBounds(0.0, 2.0));           // CoP x
    problem.setControlInfo("/forceset/force_r_4", MocoBounds(0.0)); // CoP y
    problem.setControlInfo(
            "/forceset/force_r_5", MocoBounds(-1.0, 0.0)); // CoP z
    // Torque/Moment channels (6, 7, 8)
    problem.setControlInfo("/forceset/force_r_6", MocoBounds(0.0)); // Mx
    problem.setControlInfo(
            "/forceset/force_r_7", MocoBounds(-25.0, 25.0));      // My
    problem.setControlInfo("/forceset/force_r_8", MocoBounds(0)); // Mz
    // Left foot control signals
    // Force channels (0, 1, 2)
    problem.setControlInfo(
            "/forceset/force_l_0", MocoBounds(-300.0, 300.0)); // Fx
    problem.setControlInfo(
            "/forceset/force_l_1", MocoBounds(0.0, 2000.0)); // Fy
    problem.setControlInfo(
            "/forceset/force_l_2", MocoBounds(-75.0, 75.0)); // Fz
    // Center of Pressure channels (3, 4, 5)
    problem.setControlInfo(
            "/forceset/force_l_3", MocoBounds(0.0, 2.0));           // CoP x
    problem.setControlInfo("/forceset/force_l_4", MocoBounds(0.0)); // CoP y
    problem.setControlInfo(
            "/forceset/force_l_5", MocoBounds(-1.0, 0.0)); // CoP z
    // Torque/Moment channels (6, 7, 8)
    problem.setControlInfo("/forceset/force_l_6", MocoBounds(0.0)); // Mx
    problem.setControlInfo(
            "/forceset/force_l_7", MocoBounds(-25.0, 25.0));        // My
    problem.setControlInfo("/forceset/force_l_8", MocoBounds(0.0)); // Mz

    // Add constraint that prevents contact points penetrating ground plane.
    auto* groundContactConstraint = problem.addPathConstraint<MocoContactPointConstraint>();
    // Set the properties
    groundContactConstraint->setName("no_ground_penetration");
    groundContactConstraint->setGroundPlaneHeight(0.00);
    // Add each contact point by its absolute path in the model tree.
    // The path follows: /<estimator_name>/<side_name>/<point_name>
    // Right side contact points
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/right_side/contact_point_RCAL");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/right_side/contact_point_RTOE");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/right_side/contact_point_RMT5");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/right_side/contact_point_RTOE_mid");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/right_side/contact_point_RMT5_mid");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/right_side/contact_point_R_end");
    // Left side contact points
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/left_side/contact_point_LCAL");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/left_side/contact_point_LTOE");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/left_side/contact_point_LMT5");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/left_side/contact_point_LTOE_mid");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/left_side/contact_point_LMT5_mid");
    groundContactConstraint->addContactPointPath(
            "/zmp_grf_estimator/left_side/contact_point_L_end");

    // Add and configure the ZMP path constraint
    auto* zmpConstraint = problem.addPathConstraint<MocoZeroMomentPointConstraint>();
    zmpConstraint->setName("zmp_ground_reactions_constraint");
    zmpConstraint->set_zmp_estimator_path("/zmp_grf_estimator");
    zmpConstraint->append_body_wrench_paths("/forceset/force_r");
    zmpConstraint->append_body_wrench_paths("/forceset/force_l");

    // The path constraint manages 2 actuators * 3 equations = 6 equations total.
    // The below code demonstrates an option to set potential bounds for these 
    // constraints. However, without setting bounds this should essentially try
    // and constrain the problem to enforce the equations equalling exactly 0.0,
    // or at least within the bounds of the contraint tolerance as close to 0.0
    // as possible.     
    //
    // Comment the below out if wishing to allow larger bounds on constraint equations.
    //std::vector<MocoBounds> zmpConstraintBounds;
    //for (size_t f = 0; f < 2; ++f) {  // loop through the two forces added
    //    zmpConstraintBounds.emplace_back(-1.0, 1.0);
    //    zmpConstraintBounds.emplace_back(-1.0, 1.0);
    //    zmpConstraintBounds.emplace_back(-1.0, 1.0);
    //}
    //zmpConstraint->updConstraintInfo().setBounds(zmpConstraintBounds);

    // As the body actuators include the force, point and torque as control signals
    // we can essentially replicate GRF contact tracking through a control tracking
    // goal. Depending on the function input boolean, a tracking goal can be added
    // here. 
    // Create the reference paths for the tracking goal
    std::vector<std::pair<std::string, std::string>> grfMappings = {
            // Right side - Forces
            {"/forceset/force_r_0", "R_ground_force_vx"},
            {"/forceset/force_r_1", "R_ground_force_vy"},
            {"/forceset/force_r_2", "R_ground_force_vz"},
            // Right side - COP
            {"/forceset/force_r_3", "R_ground_force_px"},
            {"/forceset/force_r_4", "R_ground_force_py"},
            {"/forceset/force_r_5", "R_ground_force_pz"},
            // Right side - Torque
            {"/forceset/force_r_6", "R_ground_torque_x"},
            {"/forceset/force_r_7", "R_ground_torque_y"},
            {"/forceset/force_r_8", "R_ground_torque_z"},
            // Left side - Forces
            {"/forceset/force_l_0", "L_ground_force_vx"},
            {"/forceset/force_l_1", "L_ground_force_vy"},
            {"/forceset/force_l_2", "L_ground_force_vz"},
            // Left side - COP
            {"/forceset/force_l_3", "L_ground_force_px"},
            {"/forceset/force_l_4", "L_ground_force_py"},
            {"/forceset/force_l_5", "L_ground_force_pz"},
            // Left side - Torque
            {"/forceset/force_l_6", "L_ground_torque_x"},
            {"/forceset/force_l_7", "L_ground_torque_y"},
            {"/forceset/force_l_8", "L_ground_torque_z"}};
    // Add tracking goal if set
    if (trackGRF) {
        // Add control goal to track new actuator forces against GRF
        // Create the Control Tracking Goal
        auto* grfTrackGoal =
                problem.addGoal<OpenSim::MocoControlTrackingGoal>("grf_track");
        // Set reference table as the experimental GRF data
        grfTrackGoal->setReference(TableProcessor("Run_3_grf.mot"));
        // Loop through and set the reference labels for tracking
        for (const auto& mapping : grfMappings) {
            grfTrackGoal->setReferenceLabel(mapping.first, mapping.second);
        }
        // Customize tracking weights and behavior
        grfTrackGoal->setWeight(0.1);
        grfTrackGoal->setAllowUnusedReferences(true);
    }

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

    // Grab guess to update control signals with GRF values
    // This should theoretically improve the initial guess
    MocoTrajectory guess = solver.getGuess();

    // Read the ground reaction force file
    TimeSeriesTable grfTable("Run_3_grf.mot");

    // Trim the GRF data to the desired times
    grfTable.trim(initialTime, finalTime);

    // Resample the guess to match the GRF times
    guess.resampleWithNumTimes(grfTable.getNumRows());
    
    // Populate actuator control trajectories in guess
    // This uses earlier created mappings
    for (const auto& mapping : grfMappings) {
        // Get the relevant experimental data
        SimTK::Vector expData = grfTable.getDependentColumn(mapping.second);
        // Set the control signal in the guess to match
        guess.setControl(mapping.first, expData);
    }

    // Set guess in solver using file
    solver.setGuess(guess);
    
    //// Reset problem before solving
    //solver.resetProblem(problem);

    //// Print problem for reference
    //if (trackGRF) {
    //    problem.print("states-grf_tracking-problem_with-zmp.omoco"); 
    //} else {
    //    problem.print("states-only_tracking-problem_with-zmp.omoco"); 
    //
    //}

    // Solve!
    // ------------------------------
    MocoSolution solution = study.solve();
    
    // Write solution to file
    // Note that this won't work if the problem doesn't solver properly
    solution.write(fmt::format("{}_solution.sto", study_name));

    // TODO: configure these appropriately...

    // Print the model.
    modelSolution.initSystem();
    modelSolution.print(fmt::format("{}_model.osim", study_name));

    // We can extract the ground reactions from the actuators in the problem
    // Export to a controls table
    TimeSeriesTable controlsTable = solution.exportToControlsTable();
    // Delete any columns that aren't in the GRF mappings list
    // Create a label map
    std::map<std::string, std::string> labelMap;
    for (const auto& mapping : grfMappings) {
        labelMap[mapping.first] = mapping.second;
    }
    // Get column labels to lookup
    auto columnLabels = controlsTable.getColumnLabels();
    // Loop backwards to avoid indexing issues
    for (int i = static_cast<int>(columnLabels.size()) - 1; i >= 0; --i) {
        const std::string& currentLabel = columnLabels[i];
        auto it = labelMap.find(currentLabel);
        if (it != labelMap.end()) {
            // If found, rename the column using its index and the paired value
            controlsTable.setColumnLabel(i, it->second);
        } else {
            // If not found, remove the column from the table
            controlsTable.removeColumn(currentLabel);
        }
    }
    // Write to file
    STOFileAdapter::write(controlsTable,
        fmt::format("{}_ground_reactions.sto", study_name));

} // end of runTrackingProblem()

// Function for running the torque-driven predictive problem
// TODO: move this elsewhere, create a muscle-driven problem...
//void runPredictiveProblem() {
//
//    // TODO: significantly clean up this example...
//    // TODO: a purely torque-driven predictive problem might not really work...
//    //  >> Kind of results in a weird motion...but sort of works...without the ZMP constraint at first...
//
//    // NOTE: specifically note that this can't run without the tracking solution...
//    
//
//    // Create and name an instance of the MocoStudy tool.
//    MocoStudy study;
//    study.setName("torque_driven_predictive_with_zmp");
//
//    // Construct a ModelProcessor and add it to the tool. ModelProcessors
//    // accept a base model (or model file) and allow you to easily modify the
//    // model by appending ModelOperators. Operations are performed in the order
//    // that they are appended to the model.
//    ModelProcessor modelProcessor("subject01_run3_with_zmp.osim");
//    // Add ground reaction external loads in lieu of a ground-contact model.
//    // Removed to see if residuals track the same as GRF
//    // modelProcessor.append(ModOpAddExternalLoads("Run_3_grf.xml"));
//    // Remove all the muscles in the model's ForceSet.
//    // Fairly sure this model already has no muscles, but anyway...
//    modelProcessor.append(ModOpRemoveMuscles());
//    // Add CoordinateActuators to the pelvis coordinates.
//    // Inputs are rotational opt. force, translational opt. force, and bound
//    // Residuals could be increased to support lack of GRFs
//    // Or taken out because we have other forces to support
//    // modelProcessor.append(ModOpAddResiduals(25.0, 5.0, 1.0));
//    // modelProcessor.append(ModOpAddResiduals(2500.0, 3000.0, 1.0));
//    // modelProcessor.append(ModOpAddResiduals(1.0, 1.0,
//    //    std::numeric_limits<double>::infinity()));
//    // Add CoordinateActuators to the remaining degrees-of-freedom.
//    // Inputs are optimal force and bound
//    modelProcessor.append(ModOpAddReserves(250.0, 1.0));
//
//    // Define the optimal control problem.
//    MocoProblem& problem = study.updProblem();
//    problem.setModelProcessor(modelProcessor);
//
//    // Set a symmetry goal
//    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
//    Model model = modelProcessor.process();
//    model.initSystem();
//    // Symmetric coordinate values for full gait cycle
//    // Coordinates ending in _r and _l are made to be symmetrical
//    // Pelvis ty and tz are also included - pelvis_tz is not
//    for (const auto& coord : model.getComponentList<Coordinate>()) {
//        if (IO::EndsWith(coord.getName(), "_r")) {
//            symmetryGoal->addStatePair({coord.getStateVariableNames()[0]});
//        }
//        if (IO::EndsWith(coord.getName(), "_l")) {
//            symmetryGoal->addStatePair({coord.getStateVariableNames()[0]});
//        }
//        if (!IO::EndsWith(coord.getName(), "_l") &&
//                !IO::EndsWith(coord.getName(), "_r") &&
//                !IO::EndsWith(coord.getName(), "_tx")) {
//            symmetryGoal->addStatePair({coord.getStateVariableNames()[0]});
//        }
//    }
//    // Set symmetry for the pelvis_tx speed
//    symmetryGoal->addStatePair({"/jointset/ground_pelvis/pelvis_tx/speed"});
//    
//    // TODO: torque control symmetry?
//
//    // Prescribed average gait speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speed");
//    speedGoal->set_desired_average_speed(3.0);  // reflects 3m/s running from original study
//    
//    // Set effort goal for control signals
//    // TODO: does this work for torque signals?
//    auto* effortGoal = problem.addGoal<MocoControlGoal>("effort", 10.0);
//
//    // No weight on ZMP forces to avoid conflict with tracking goal
//    effortGoal->setWeightForControlPattern(".*zmp.*", 0.0);
//
//    // Set coordinate bounds in the problem
//    // Get model, coordinate and states information
//    const TimeSeriesTable statesTable =
//            TimeSeriesTable("subject01_run3_states.sto");
//    const auto& coordSet = model.getCoordinateSet();
//    int numCoords = coordSet.getSize();
//    // Loop through coordinates to set bounds
//    // Constrain pelvis_tx in a more general way to suit the problem
//    for (int i = 0; i < numCoords; ++i) {
//        // Get coordinate names and path
//        std::string coordName = coordSet.get(i).getName();
//        std::string coordPath = coordSet.get(i).getAbsolutePathString();
//        std::string statePath = coordPath + "/value";
//        // Check for pelvis tx
//        if (coordName == "pelvis_tx") {
//            // Set explicitly
//            problem.setStateInfo(statePath, {0.0, 3.0}, // total bounds
//                    {0.0}                               // initial bounds
//            );
//            continue;
//        }
//        // Extract the vector of tracking data for this coordinate
//        auto statesData = statesTable.getDependentColumn(statePath);
//        // Find the absolute min and max values within the trajectory
//        double dataMin = SimTK::min(statesData);
//        double dataMax = SimTK::max(statesData);
//        double range = dataMax - dataMin;
//        // Calculate the 20% bounds adjustments
//        double lowerBound = dataMin - (0.20 * range);
//        double upperBound = dataMax + (0.20 * range);
//        // Calculate similar bounds for initial and final coordinate states
//         double initialVal = statesData[0];
//         double initLowerBound = initialVal - (0.20 * range);
//         double initUpperBound = initialVal + (0.20 * range);
//        // Safety check on bounds
//         initLowerBound = std::clamp(initLowerBound, lowerBound, upperBound);
//         initUpperBound = std::clamp(initUpperBound, lowerBound, upperBound);
//        // Double check that clamping didn't invert them
//         if (initLowerBound > initUpperBound)
//             std::swap(initLowerBound, initUpperBound);
//        //  Apply the updated bounds to the state variable in Moco
//         problem.setStateInfo(statePath, {lowerBound, upperBound},
//             {initLowerBound, initUpperBound});
//    }
//
//    // Set time bounds
//    // These are related to the initial time values from the tracking states
//    problem.setTimeBounds(0, {0.7, 0.8});
//
//    // TODO: add low weight states or GRF goal?
//
//    // Add constraint that prevents contact points penetrating ground plane.
//    auto* groundContactConstraint =
//            problem.addPathConstraint<MocoContactPointConstraint>();
//    // Set the properties
//    groundContactConstraint->setName("no_ground_penetration");
//    groundContactConstraint->setGroundPlaneHeight(0.00);
//    // Add each contact point by its absolute path in the model tree.
//    // The path follows: /<estimator_name>/<side_name>/<point_name>
//    // Right side contact points
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/right_side/contact_point_RCAL");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/right_side/contact_point_RTOE");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/right_side/contact_point_RMT5");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/right_side/contact_point_RTOE_mid");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/right_side/contact_point_RMT5_mid");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/right_side/contact_point_R_end");
//    // Left side contact points
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/left_side/contact_point_LCAL");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/left_side/contact_point_LTOE");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/left_side/contact_point_LMT5");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/left_side/contact_point_LTOE_mid");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/left_side/contact_point_LMT5_mid");
//    groundContactConstraint->addContactPointPath(
//            "/zmp_grf_estimator/left_side/contact_point_L_end");
//
//    // Define the optimisation bounds for the ZMP force input ports
//    // Restrict bounds to reasonable values for force, COP and torque
//    // TODO: what are reasonable bounds?
//    // Right foot control signals
//    // Force channels (0, 1, 2)
//    problem.setControlInfo(
//            "/forceset/force_r_0", MocoBounds(-500.0, 500)); // Fx
//    problem.setControlInfo(
//            "/forceset/force_r_1", MocoBounds(0, 2500)); // Fy
//    problem.setControlInfo(
//            "/forceset/force_r_2", MocoBounds(-100, 100.0)); // Fz
//    // Center of Pressure channels (3, 4, 5)
//    problem.setControlInfo(
//            "/forceset/force_r_3", MocoBounds(-2.0, 2.0)); // CoP x
//    problem.setControlInfo(
//            "/forceset/force_r_4", MocoBounds(-0.1, 0.1)); // CoP y
//    problem.setControlInfo(
//            "/forceset/force_r_5", MocoBounds(-1.0, 1.0)); // CoP z
//    // Torque/Moment channels (6, 7, 8)
//    problem.setControlInfo(
//            "/forceset/force_r_6", MocoBounds(-1.0, 1.0)); // Mx
//    problem.setControlInfo(
//            "/forceset/force_r_7", MocoBounds(-250.0, 250)); // My
//    problem.setControlInfo(
//            "/forceset/force_r_8", MocoBounds(-1.0, 1.0)); // Mz
//    // Left foot control signals
//    // Force channels (0, 1, 2)
//    problem.setControlInfo(
//            "/forceset/force_l_0", MocoBounds(-500, 500.0)); // Fx
//    problem.setControlInfo(
//            "/forceset/force_l_1", MocoBounds(0, 2500)); // Fy
//    problem.setControlInfo(
//            "/forceset/force_l_2", MocoBounds(-100.0, 100.0)); // Fz
//    // Center of Pressure channels (3, 4, 5)
//    problem.setControlInfo(
//            "/forceset/force_l_3", MocoBounds(-2.0, 2.0)); // CoP x
//    problem.setControlInfo(
//            "/forceset/force_l_4", MocoBounds(-0.1, 0.0)); // CoP y
//    problem.setControlInfo(
//            "/forceset/force_l_5", MocoBounds(-1.0, 1.0)); // CoP z
//    // Torque/Moment channels (6, 7, 8)
//    problem.setControlInfo(
//            "/forceset/force_l_6", MocoBounds(-1.0, 1.0)); // Mx
//    problem.setControlInfo(
//            "/forceset/force_l_7", MocoBounds(-250.0, 250.0)); // My
//    problem.setControlInfo(
//            "/forceset/force_l_8", MocoBounds(-1.0, 1.0)); // Mz
//
//    // Add and configure the ZMP path constraint
//    auto* zmpConstraint =
//            problem.addPathConstraint<MocoZeroMomentPointConstraint>();
//    zmpConstraint->setName("zmp_ground_reactions_constraint");
//    zmpConstraint->set_zmp_estimator_path("/zmp_grf_estimator");
//    zmpConstraint->append_zmp_force_paths("/forceset/zmp_force_r");
//    zmpConstraint->append_zmp_force_paths("/forceset/zmp_force_l");
//
//    // The path constraint manages 2 actuators * 9 indices = 18 equations total.
//    // Set reasonable bounds for around these constraints. Enforcing exactly 0.0
//    // would probably be perfect dynamic consistency, which seems unreasonable.
//    // The constraint calculations are in the order of force, COP and torque
//    // so can be placed in this order.
//    std::vector<MocoBounds> zmpConstraintBounds;
//    for (size_t f = 0; f < 2; ++f) { // loop through the two forces added
//        // 3 rows for Forces (Allowing 5 N of discrepancy)
//        zmpConstraintBounds.emplace_back(-5.0, 5.0); // Fx
//        zmpConstraintBounds.emplace_back(-5.0, 5.0); // Fy
//        zmpConstraintBounds.emplace_back(-5.0, 5.0); // Fz
//        // 3 rows for CoPs (Allowing 2 cm / 0.02 m discrepancy)
//        zmpConstraintBounds.emplace_back(-0.02, 0.02); // CoP x
//        zmpConstraintBounds.emplace_back(-0.02, 0.02); // CoP y
//        zmpConstraintBounds.emplace_back(-0.02, 0.02); // CoP z
//        // TODO: taken out torque constraint as free moment calculation looks
//        // off...
//        //// 3 rows for Free Moments/Torques (Allowing 2 Nm discrepancy)
//        // zmpConstraintBounds.emplace_back(-2.0, 2.0); // Tx
//        // zmpConstraintBounds.emplace_back(-2.0, 2.0); // Ty
//        // zmpConstraintBounds.emplace_back(-2.0, 2.0); // Tz
//    }
//    zmpConstraint->updConstraintInfo().setBounds(zmpConstraintBounds);
//
//    // Update the solver tolerances
//    auto& solver = study.updSolver<MocoCasADiSolver>();
//    solver.set_num_mesh_intervals(50);
//    solver.set_optim_convergence_tolerance(1e-3);
//    solver.set_optim_constraint_tolerance(1e-3);
//    // solver.set_enforce_path_constraint_mesh_interior_points(true);
//    // solver.set_transcription_scheme("hermite-simpson");
//
//    // Set guess in solver using file
//    solver.setGuessFile("tracking_solution_with_zmp.sto");
//
//    // Reset problem before solving
//    solver.resetProblem(problem);
//
//    // Print problem for reference
//    problem.print("predictive_problem_with_zmp.omoco");
//
//    // Solve!
//    MocoSolution solution = study.solve();
//
//    // Write solution to file
//    const std::string outputSolutionFile = "predictive_solution_with_zmp.sto";
//    solution.write(outputSolutionFile);
//    std::cout << "Predictive solution saved to: " << outputSolutionFile << "\n";
//
//    //// TODO: will visualiser work?
//    //// Visualize the solution.
//    // study.visualize(solution);
// 
// } // end of runPredictiveProblem()


// Run main function
int main() {

    
    // ======================================================================
    // STEP 1: Create the model with ZMP components
    // ======================================================================
    printSection("Step 1. Adding ZMP and actuator components to model");

    /// Model preparation.
    /// -----------------
    /// Update the model to prepare it for tracking optimization. This adds 
    /// the ZMP components to use in the tracking simulations.
    
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
    // and then append to the relevant contact side. Note that all markers
    // are attached to the model via the calcaneus bodies, even though that
    // they are more applicable to the toes bodies
    // Right side
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker = model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point
        ContactPoint* cp = new ContactPoint(
                "contact_point_" + contactName, calcn_r, pointLocation);
        cp->set_distance_threshold(0.04);
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
        // Build the contact point
        ContactPoint* cp = new ContactPoint(
                "contact_point_" + contactName, calcn_l, pointLocation);
        cp->set_distance_threshold(0.04);
        cp->set_contact_checking_method("distance");
        // Add the contact point to the contact side
        leftContactSide->addContactPoint(cp);
    }

    // Add contact sides to estimator
    zmp->addContactSide(rightContactSide);
    zmp->addContactSide(leftContactSide);

    // Add estimator component to model
    model.addComponent(zmp);

    // Configure and add actuators to generate external loads
    auto* rightForce = new BodyWrenchActuator("force_r", "calcn_r");
    auto* leftForce = new BodyWrenchActuator("force_l", "calcn_l");
    model.addForce(rightForce);
    model.addForce(leftForce);

    // Finalize model connections
    model.finalizeConnections();

    // ======================================================================
    // STEP 2: Run a torque-driven problem with GRF tracking
    // ======================================================================
    printSection("Step 2. Running torque-driven states & GRF tracking example");

    //// Solve the torque-driven states tracking problem with GRF tracking
    //runTrackingStudy(model, false, true);

    // ======================================================================
    // STEP 3: Run a torque-driven tracking simulation with no GRF tracking
    // ======================================================================
    printSection("Step 3. Running torque-driven states only tracking example");

    //// Solve the torque-driven states tracking problem with no GRF tracking
    //runTrackingStudy(model, false, false);

    // ======================================================================
    // STEP 4: Run a muscle-driven tracking simulation with GRF tracking
    // ======================================================================
    printSection("Step 4. Running muscle-driven states & GRF tracking example");

    // Solve the torque-driven states tracking problem with no GRF tracking
    runTrackingStudy(model, true, true);

    //// Solve the torque-driven predictive problem.
    //runPredictiveProblem();

    return EXIT_SUCCESS;
	
}
