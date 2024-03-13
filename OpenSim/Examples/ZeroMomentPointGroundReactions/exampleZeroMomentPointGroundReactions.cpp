/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleZeroMomentPointGroundReactions.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors
 * Author: Aaron Fox
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
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/SimulationUtilities.h"
#include "OpenSim/Moco/MocoTrajectory.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/OpenSim.h"

int main() {
	
    using namespace OpenSim;

    // TODO:
    //      > Figure out use of relative paths...

	//=============================================================================
    //  Read in and extract relevant data
    //=============================================================================

	// Get the model to use with the component
    // NOTE: CoordinateActuator not being registered was messing with this load?
    Model osimModel(
		"C:\\opensim-zmp\\OpenSim\\Examples\\ZeroMomentPointGroundReactions\\subject01.osim");

    // Get the Moco solution to test with this example
    // NOTE: no joints can be locked otherwise nan's end up in states trajectory
    MocoTrajectory mocoTraj = MocoTrajectory(
            "C:\\opensim-zmp\\OpenSim\\Examples\\ZeroMomentPointGroundReactions\\subject01_run3_cycle1_mocoSolution.sto");
    StatesTrajectory statesTraj = mocoTraj.exportToStatesTrajectory(osimModel);

    // To test the function that takes udot, create an accelerations table
    // using the state speeds
    mocoTraj.generateAccelerationsFromSpeeds();
    TimeSeriesTable uDotTable = mocoTraj.exportToAccelerationsTable();

    // Define number of times from states trajectory
    int nt = mocoTraj.getNumTimes();

	//=============================================================================
    //  Create ZeroMomentGroundReactions component
    //=============================================================================
	
	// Create an instance of the component
	/*ZeroMomentPointGroundReactions zmpGroundReactions;*/
    ZeroMomentPointGroundReactions* zmpGroundReactions =
            new ZeroMomentPointGroundReactions();
	
	// Set the overall name of the component
	/*zmpGroundReactions.setName("zmpGroundReactions");*/
    zmpGroundReactions->setName("zmpGroundReactions");

	// Set the free joint name
    /*zmpGroundReactions.setFreeJointName("ground_pelvis");*/
    zmpGroundReactions->setFreeJointName("ground_pelvis");

	// Set the distance checking threshold
    /*zmpGroundReactions.setDistanceThreshold(0.02);*/
    zmpGroundReactions->setDistanceThreshold(0.02);

	// Set the force checking threshold
    /*zmpGroundReactions.setForceThreshold(50.0);*/
    zmpGroundReactions->setForceThreshold(50.0);

	//=============================================================================
    //  Create the right side contact body for the component
    //=============================================================================

	// Add the right calcaneus as the contact body
    /*zmpGroundReactions.addContactBodyZMP("calcn_r_contact", "calcn_r");*/
    zmpGroundReactions->addContactBodyZMP("calcn_r_contact", "calcn_r");

	// Apend checkpoints for the contact body
	// This uses the locations of markers specified in the model as "floor" level
	// from the static pose motion.
    /*zmpGroundReactions.upd_zmp_body_list(0).addCheckpoint(
            osimModel.updMarkerSet().get("RCAL_ground").get_location());
    zmpGroundReactions.upd_zmp_body_list(0).addCheckpoint(
            osimModel.updMarkerSet().get("RTOE_ground").get_location());
    zmpGroundReactions.upd_zmp_body_list(0).addCheckpoint(
            osimModel.updMarkerSet().get("RMT5_ground").get_location());*/
    zmpGroundReactions->upd_zmp_body_list(0).addCheckpoint(
            osimModel.updMarkerSet().get("RCAL_ground").get_location());
    zmpGroundReactions->upd_zmp_body_list(0).addCheckpoint(
            osimModel.updMarkerSet().get("RTOE_ground").get_location());
    zmpGroundReactions->upd_zmp_body_list(0).addCheckpoint(
            osimModel.updMarkerSet().get("RMT5_ground").get_location());

    //=============================================================================
    //  Create the right side contact body for the component
    //=============================================================================

	// Add the left calcaneus as a contact body
    /*zmpGroundReactions.addContactBodyZMP("calcn_l_contact", "calcn_l");*/
    zmpGroundReactions->addContactBodyZMP("calcn_l_contact", "calcn_l");

	// Apend checkpoints for the contact body
    // This uses the locations of markers specified in the model as "floor"
    // level from the static pose motion.
    /*zmpGroundReactions.upd_zmp_body_list(1).addCheckpoint(
            osimModel.updMarkerSet().get("LCAL_ground").get_location());
    zmpGroundReactions.upd_zmp_body_list(1).addCheckpoint(
            osimModel.updMarkerSet().get("LTOE_ground").get_location());
    zmpGroundReactions.upd_zmp_body_list(1).addCheckpoint(
            osimModel.updMarkerSet().get("LMT5_ground").get_location());*/
    zmpGroundReactions->upd_zmp_body_list(1).addCheckpoint(
            osimModel.updMarkerSet().get("LCAL_ground").get_location());
    zmpGroundReactions->upd_zmp_body_list(1).addCheckpoint(
            osimModel.updMarkerSet().get("LTOE_ground").get_location());
    zmpGroundReactions->upd_zmp_body_list(1).addCheckpoint(
            osimModel.updMarkerSet().get("LMT5_ground").get_location());

	// Print the component to XML file to check
    /*zmpGroundReactions.print(
            "C:\\+GitRepos+\\zmp-opensim\\zmpGroundReactions.xml");*/

    // Get number of contact bodies in the component
    // (basically to test this function)
    /*int nCB = zmpGroundReactions.getNumContactBodiesZMP();*/
    int nCB = zmpGroundReactions->getNumContactBodiesZMP();

    //=============================================================================
    //  Finalise model with component
    //=============================================================================

	// Append the component to the model
    osimModel.addComponent(zmpGroundReactions);

    // Finalize model connections
    osimModel.finalizeConnections();

    // Initialize the model's underlying computational system and get its
    // default state.
    SimTK::State& sWorkingCopy = osimModel.initSystem(); // --- needed?

    // Get the number of coordinates from the model state
    int nq = sWorkingCopy.getNQ();

    //=============================================================================
    //  Set-up files to store results
    //=============================================================================

    // Create storage file to append ZMP GRFs to
    Storage zmpResults = Storage(nt);

    // Create columns for ZMP force, moment and point results
    // These get partitioned and labeled based on the contact bodies
    // Ordering of GRF files like this needs to have the force and points
    // for each body, followed by the torques for it to work right in the
    // visualizer. This requires some annoying column labelling.
    Array<std::string> zmpLabels("time", 9 * 2);
    for (int cb = 0; cb < nCB; cb++) {
    
        // Get the body name for the current contact body
        /*std::string labelBody = zmpGroundReactions.upd_zmp_body_list(cb).get_body_name();*/
        std::string labelBody =
                zmpGroundReactions->upd_zmp_body_list(cb).get_body_name();

        // Set the 9 labels for bodies force, moment and point data
        // This could be a more efficient loop, but that's a later problem
        zmpLabels.set(cb * 6 + 1, labelBody + "_ground_force_vx");
        zmpLabels.set(cb * 6 + 2, labelBody + "_ground_force_vy");
        zmpLabels.set(cb * 6 + 3, labelBody + "_ground_force_vz");
        zmpLabels.set(cb * 6 + 4, labelBody + "_ground_force_px");
        zmpLabels.set(cb * 6 + 5, labelBody + "_ground_force_py");
        zmpLabels.set(cb * 6 + 6, labelBody + "_ground_force_pz");
    
    }
    for (int cb = 0; cb < nCB; cb++) {

        // Get the body name for the current contact body
        /*std::string labelBody =
         * zmpGroundReactions.upd_zmp_body_list(cb).get_body_name();*/
        std::string labelBody =
                zmpGroundReactions->upd_zmp_body_list(cb).get_body_name();

        // Set the 9 labels for bodies force, moment and point data
        // This could be a more efficient loop, but that's a later problem
        zmpLabels.set((nCB * 6) + (cb * 3 + 1), labelBody + "_ground_torque_x");
        zmpLabels.set((nCB * 6) + (cb * 3 + 2), labelBody + "_ground_torque_y");
        zmpLabels.set((nCB * 6) + (cb * 3 + 3), labelBody + "_ground_torque_z");

    }

    // Set the column labels in storage
    zmpResults.setColumnLabels(zmpLabels);

    //=============================================================================
    //  Extract ground reactions from states
    //=============================================================================
	
    // Get back the component from the model for calculations
    const auto& zmpCalculator =
            osimModel.getComponent<ZeroMomentPointGroundReactions>(
                    "zmpGroundReactions");

    // Loop through times
    for (int i = 0; i < nt; i++) {

        // Get the current state
        SimTK::State s = statesTraj[i];

        // Get accelerations from the Moco trajectory at the current state
        SimTK::Vector udot(nq);
        for (int k = 0; k < nq; k++) {
            udot.set(k, uDotTable.getRowAtIndex(i).getAnyElt(0, k));
        }

        // Run the function with state and udot to calculate ground reactions
        osimModel.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
        SimTK::Vector groundReactions = zmpCalculator.calcGroundReactions(s, udot, true);

        // Create a state vector that includes the time and ground reactions
        StateVector zmpStateVec = StateVector(s.getTime(), groundReactions);

        // Append current vector to ZMP results
        zmpResults.append(zmpStateVec);

    }

    // Test output of storage file
    /// Distance metric not working --- closer to ground?
    Storage::printResult(
        &zmpResults,
        "testZMP_outputs",
        "C:\\opensim-zmp\\OpenSim\\Examples\\ZeroMomentPointGroundReactions",
        -1, ".sto");

    return 0;

}
