/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleZmpTrackingSimulation.cpp 	                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author: Aaron Fox                                                          *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "OpenSim/Simulation/Model/ZeroMomentPointGroundReactions.h"
#include "OpenSim/Simulation/Model/ZeroMomentPointContactBody.h"
#include "OpenSim/Simulation/Model/ZeroMomentPointContactBodySet.h"
#include "OpenSim/Simulation/Model/ZeroMomentPointContactPoint.h"
#include "OpenSim/Simulation/Model/ZeroMomentPointContactPointSet.h"

#include "OpenSim/Simulation/SimbodyEngine/ZeroMomentPointContactPointConstraint.h"
#include "OpenSim/Simulation/SimbodyEngine/RollingOnSurfaceConstraint.h"

#include "OpenSim/Simulation/Model/Model.h"
//#include "OpenSim/Simulation/SimulationUtilities.h"
#include "OpenSim/Moco/osimMoco.h"
#include "OpenSim/Moco/MocoProblem.h"
#include "OpenSim/Moco/MocoOutputConstraint.h"
#include "OpenSim/Moco/MocoUtilities.h"
#include "OpenSim/Actuators/ModelOperators.h"
#include "OpenSim/Moco/MocoTrajectory.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/OpenSim.h"

// TODO:
    // > Not sure if constraint points limit ground penetration...?
        // >> Test with rolling on surface constraint and no external loads...

//=============================================================================
//=============================================================================
/**
 * TODO: add detailed notes...
 * 
 * See the README.txt next to this file for more information on the data used.
 */

int main() {
	
    using namespace OpenSim;

    // Read in the example model to connect components
    Model osimModel("subject01.osim");

    //=============================================================================
    //  Create the base ZeroMomentGroundReactions component
    //=============================================================================

    // Create an instance of the component
    ZeroMomentPointGroundReactions* zmpGroundReactions =
            new ZeroMomentPointGroundReactions();

    // Set the overall name of the component
    zmpGroundReactions->setName("zmpGroundReactions");

    // Set the free joint name
    zmpGroundReactions->setFreeJointName("ground_pelvis");

    // Set the force checking threshold
    zmpGroundReactions->setForceThreshold(50.0);

    //=============================================================================
    //  Create and append ZeroMomentContactBody components
    //=============================================================================

    // Create the right and left contact bodies
    ZeroMomentPointContactBody* contactBodyRight =
            new ZeroMomentPointContactBody(
                    "calcn_r_contact", "calcn_r", "distance");
    ZeroMomentPointContactBody* contactBodyLeft =
            new ZeroMomentPointContactBody(
                    "calcn_l_contact", "calcn_l", "distance");

    // Set the desired distance and velocity thresholds for contact checking
    double distThreshold = 0.04;
    double velThreshold = 1.50;

    // The model has a number of markers added to each foot to reference for
    // adding the contact points. These are added here with the specified
    // distance and velocity threshold checking limits.

    // Create arrays to loop through for adding contact body points

    // Right side
    Array<std::string> contactPointsRight;
    contactPointsRight.append("RCAL");
    contactPointsRight.append("RTOE");
    contactPointsRight.append("RMT5");
    contactPointsRight.append("RTOE_mid");
    contactPointsRight.append("RMT5_mid");
    contactPointsRight.append("R_end");

    // Add the contact points using the marker locations
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);

        // Add the contact point
        contactBodyRight->addContactPoint("contact_point_" + contactName,
                osimModel.getBodySet().get("calcn_r"),
                osimModel.getMarkerSet()
                        .get(contactName + "_ground")
                        .get_location(),
                distThreshold, velThreshold);
    }

    // Left side
    Array<std::string> contactPointsLeft;
    contactPointsLeft.append("LCAL");
    contactPointsLeft.append("LTOE");
    contactPointsLeft.append("LMT5");
    contactPointsLeft.append("LTOE_mid");
    contactPointsLeft.append("LMT5_mid");
    contactPointsLeft.append("L_end");

    // Add the contact points using the marker locations
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);

        // Add the contact point
        contactBodyLeft->addContactPoint("contact_point_" + contactName,
                osimModel.getBodySet().get("calcn_l"),
                osimModel.getMarkerSet()
                        .get(contactName + "_ground")
                        .get_location(),
                distThreshold, velThreshold);
    }

    // Add the contact bodies to the ZMP ground reactions component
    zmpGroundReactions->upd_ZeroMomentPointContactBodySet().adoptAndAppend(
            contactBodyRight);
    zmpGroundReactions->upd_ZeroMomentPointContactBodySet().adoptAndAppend(
            contactBodyLeft);

    //=============================================================================
    //  Add the component to the model
    //=============================================================================

    // Append the component to the model
    osimModel.addComponent(zmpGroundReactions);

    //=============================================================================
    //  Create and append ZeroMomentPointContactPointConstraint components
    //=============================================================================

    // The same arrays of contact points from earlier can be used to add the
    // constraints

    //// Right side
    //for (int ii = 0; ii < contactPointsRight.size(); ii++) {

    //    // Get the marker name
    //    std::string contactName = contactPointsRight.get(ii);

    //    // Create the constraint
    //    ZeroMomentPointContactPointConstraint* constraint =
    //            new ZeroMomentPointContactPointConstraint(
    //                    "constraintPoint_" + contactName,
    //                    osimModel.getBodySet().get("calcn_r"),
    //                    osimModel.getGround(),
    //                    osimModel.getMarkerSet()
    //                            .get(contactName + "_ground")
    //                            .get_location());

    //    // Add the constraint
    //    osimModel.addConstraint(constraint);
    //}

    //// Left side
    //for (int ii = 0; ii < contactPointsLeft.size(); ii++) {

    //    // Get the marker name
    //    std::string contactName = contactPointsLeft.get(ii);

    //    // Create the constraint
    //    ZeroMomentPointContactPointConstraint* constraint =
    //            new ZeroMomentPointContactPointConstraint(
    //                    "constraintPoint_" + contactName,
    //                    osimModel.getBodySet().get("calcn_l"),
    //                    osimModel.getGround(),
    //                    osimModel.getMarkerSet()
    //                            .get(contactName + "_ground")
    //                            .get_location());

    //    // Add the constraint
    //    osimModel.addConstraint(constraint);
    //}

    /*RollingOnSurfaceConstraint* calcn_r_constraint =
            new RollingOnSurfaceConstraint();
    RollingOnSurfaceConstraint* calcn_l_constraint =
            new RollingOnSurfaceConstraint();
    RollingOnSurfaceConstraint* toes_r_constraint =
            new RollingOnSurfaceConstraint();
    RollingOnSurfaceConstraint* toes_l_constraint =
            new RollingOnSurfaceConstraint();*/

    /*calcn_r_constraint->connectSocket_rolling_body(osimModel.getBodySet().get("calcn_r"));
    calcn_r_constraint->connectSocket_surface_body(osimModel.getGround());

    calcn_l_constraint->connectSocket_rolling_body(osimModel.getBodySet().get("calcn_l"));
    calcn_l_constraint->connectSocket_surface_body(osimModel.getGround());

    toes_r_constraint->connectSocket_rolling_body(osimModel.getBodySet().get("toes_r"));
    toes_r_constraint->connectSocket_surface_body(osimModel.getGround());

    toes_l_constraint->connectSocket_rolling_body(osimModel.getBodySet().get("toes_l"));
    toes_l_constraint->connectSocket_surface_body(osimModel.getGround());

    osimModel.addConstraint(calcn_r_constraint);
    osimModel.addConstraint(calcn_l_constraint);
    osimModel.addConstraint(toes_r_constraint);
    osimModel.addConstraint(toes_l_constraint);*/


    //=============================================================================
    //  TODO: Leaving space to connect ZMP Contact Forces
    //=============================================================================

    //=============================================================================
    //  Set-up model for tracking simulation
    //=============================================================================

    // Finalize model connections
    osimModel.finalizeConnections();

    // Construct a model processor to set on the tracking tool
    ModelProcessor modelProc(osimModel);

    // Append external loads
    // TODO: eventually remove this given ZMP will be used for tracking...
    modelProc.append(ModOpAddExternalLoads("Run_3_grf.xml"));

    //=============================================================================
    //  Set-up tracking simulation
    //=============================================================================
    
    // Create tracking tool and set name
    MocoTrack track;
    track.setName("ZmpTrackingSimulation");

    // Set the model processor
    track.setModel(modelProc);

    // Set the coordinates to track
    track.setStatesReference(TableProcessor("ikTrackingData.sto"));

    // Set general parameters
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_apply_tracked_states_to_guess(true);

    // Set the initial and final times
    // These correspond to a full gait cycle in the data
    track.set_initial_time(0.514);
    track.set_final_time(1.231);

    // Initialise to a Moco study and problem
    MocoStudy study = track.initialize();
    MocoProblem& problem = study.updProblem();

    // Get a reference to the Moco control goal to edit
    MocoControlGoal& effort =
            dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    effort.setWeight(1e-3);
    effort.setExponent(2);

    // Update individual weights for actuators in the control effort goal
    // NOTE: low residuals given removal of external loads...
    effort.setWeightForControlPattern("/forceset/.*_residual", 0.1);
    effort.setWeightForControlPattern("/forceset/.*_actuator", 1.0);

    // Get a reference to the state tracking goal to edit
    MocoStateTrackingGoal& tracking =
            dynamic_cast<MocoStateTrackingGoal&>(problem.updGoal("state_tracking"));
    tracking.setScaleWeightsWithRange(true);

    // Add constraints to problem to avoid ground penetration
    // TODO: this should be replaced by model constraints (or another constraint function?)

    // Right side
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);

        // Add to problem
        auto markerConstraint = problem.addPathConstraint<MocoOutputConstraint>();

        // Get a reference to the constraint to edit
        /*MocoOutputConstraint& markerConstraint = dynamic_cast<MocoOutputConstraint&>(
            problem.updPhase(0).updPathConstraint(contactName + "_constraint"));*/
        markerConstraint->setName(contactName + "_constraint");
        markerConstraint->setOutputPath(
                "/markerset/" + contactName + "_ground|location");
        markerConstraint->setOutputIndex(1);
        std::vector<MocoBounds> markerBounds;
        markerBounds.push_back(MocoBounds(0.0, 1.0));
        markerConstraint->updConstraintInfo().setBounds(markerBounds);

    }

    // Left side
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);

        // Add to problem
        auto markerConstraint =
                problem.addPathConstraint<MocoOutputConstraint>();

        // Get a reference to the constraint to edit
        /*MocoOutputConstraint& markerConstraint =
           dynamic_cast<MocoOutputConstraint&>(
            problem.updPhase(0).updPathConstraint(contactName +
           "_constraint"));*/
        markerConstraint->setName(contactName + "_constraint");
        markerConstraint->setOutputPath(
                "/markerset/" + contactName + "_ground|location");
        markerConstraint->setOutputIndex(1);
        std::vector<MocoBounds> markerBounds;
        markerBounds.push_back(MocoBounds(0.0, 1.0));
        markerConstraint->updConstraintInfo().setBounds(markerBounds);
    }

    //=============================================================================
    //  Define and configure the solver
    //=============================================================================

    // Review problem
    problem.print("problem.omoco");

    // Get the solver
    auto& solver = study.updSolver<MocoCasADiSolver>();

    // Solver settings
    solver.set_optim_max_iterations(2000);
    solver.set_num_mesh_intervals(10); // TODO: back to 50 - setting lower for speediness during testing
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);

    //=============================================================================
    //  Solve the problem!
    //=============================================================================

    //// Solve and visualize
    //MocoSolution solution = study.solve();
    //study.visualize(solution);


    //=============================================================================
    //  Get solution outputs
    //=============================================================================
    
    // TODO: outputs are in component but not a model output? Or not a Vec3? So not getting connected to reporter...

    // Write solution to file
    //solution.write("ZmpTrackingSolution.sto");
    MocoTrajectory traj = MocoTrajectory("ZmpTrackingSolution.sto");

    // Set ground reaction as outputs to extract
    std::vector<std::string> outputsToGet;
    /*outputsToGet.push_back(".*contact_.*");*/
    outputsToGet.push_back("/zmpGroundReactions.*");

    // Get solution as states and controls table
    /*TimeSeriesTable statesTable = solution.exportToStatesTable();
    TimeSeriesTable controlsTable = solution.exportToControlsTable();*/
    TimeSeriesTable statesTable = traj.exportToStatesTable();
    TimeSeriesTable controlsTable = traj.exportToControlsTable();

    // Get model outputs in table
    Model trackingModel = modelProc.process();
    SimTK::State s = trackingModel.initSystem();
    trackingModel.print("trackingModel.osim");

    // TODO: does this work now? Using SimTK::Vector here doesn't work for some reason? SpatialVec does though?
    // Vec3 works as well though, so should it be something different? Why doesn't Vector work?
    // TODO: are outputs actually needed?
    const auto& outputTable = analyze<SimTK::Vec3>(
            trackingModel, statesTable, controlsTable, outputsToGet).flatten();
    /*TimeSeriesTable_<SimTK::Vector> outputTable = analyze<SimTK::Vector>(
            trackingModel, statesTable, controlsTable, outputsToGet);*/
    //TimeSeriesTable flattenedTable = outputTable.flatten();
    STOFileAdapter().write(outputTable, "outputs.sto");
    

    // TODO: 
    // > Outputs ignored - "Ignoring output /zmpGroundReactions|ground_reactions of type Vector
    
    return EXIT_SUCCESS;

}
