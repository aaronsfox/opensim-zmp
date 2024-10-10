/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleZmpContactForce.cpp 	                          *
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
#include "OpenSim/Simulation/Model/ZeroMomentPointContactForce.h"

#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/SimulationUtilities.h"
#include "OpenSim/Moco/MocoTrajectory.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/OpenSim.h"

//=============================================================================
//=============================================================================
/**
 * TODO: add detailed notes aboute example
 * 
 * See the README.txt next to this file for more information on the data used.
 */

int main() {
	
    using namespace OpenSim;

    // Read in the example model to connect components
    Model osimModel("subject01_run3.osim");

    // Set a state index to use
    // Current state index corresponds to peak vertical GRF on right side
    int stateInd = 17;

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
                "calcn_r",
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
                "calcn_l",
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
    // Ensure connections for component socket is finalized
    osimModel.addComponent(zmpGroundReactions);
    osimModel.finalizeConnections();

    //=============================================================================
    //  Create and connect the ZeroMomentPointContactForce objects
    //=============================================================================

    // Get the ZeroMomentPointGroundReactions object back from the model
    /*ZeroMomentPointGroundReactions zmpComponent =
            osimModel.getComponent<ZeroMomentPointGroundReactions>(
                    "zmpGroundReactions");*/

    // Use the constructors to create a force for both sides
    ZeroMomentPointContactForce* zmpContactForce_r =
            new ZeroMomentPointContactForce();
    ZeroMomentPointContactForce* zmpContactForce_l =
            new ZeroMomentPointContactForce();

    // Set names
    zmpContactForce_r->setName("zmpContactForce_r");
    zmpContactForce_l->setName("zmpContactForce_l");

    // Set the Zero Moment Point contact body names
    zmpContactForce_r->setZeroMomentPointContactBodyName("calcn_r_contact");
    zmpContactForce_l->setZeroMomentPointContactBodyName("calcn_l_contact");

    // Set applied to body names
    zmpContactForce_r->setAppliedToBody("calcn_r");
    zmpContactForce_l->setAppliedToBody("calcn_l");

    // Connect to ZMP ground reactions object
    zmpContactForce_r->setZeroMomentPointGroundReactions(*zmpGroundReactions);
    zmpContactForce_l->setZeroMomentPointGroundReactions(*zmpGroundReactions);

    // Add the forces to the model
    osimModel.addForce(zmpContactForce_r);
    osimModel.addForce(zmpContactForce_l);

    //=============================================================================
    //  Finalize model for use
    //=============================================================================

    // Finalize model connections
    osimModel.finalizeConnections();

    // Initialize the model's underlying computational system and get its
    // default state.
    SimTK::State& sWorkingCopy = osimModel.initSystem();

    //=============================================================================
    //  Read in the motion data to use
    //=============================================================================

    // Get the Moco solution to test with this example
    // NOTE: no joints can be locked otherwise nan's end up in states trajectory
    MocoTrajectory mocoTraj =
            MocoTrajectory("subject01_run3_trackingSolution.sto");
    StatesTrajectory states = mocoTraj.exportToStatesTrajectory(osimModel);
    /*TimeSeriesTable controls = mocoTraj.exportToControlsTable();*/

    // In lieu of a dynamic simulation where udot could be calculated, here
    // we get the accelerations to be set across states
    mocoTraj.generateAccelerationsFromSpeeds();
    TimeSeriesTable udot = mocoTraj.exportToAccelerationsTable();

    // Get the state to calculate ground reactions at
    SimTK::State s = states[stateInd];
    osimModel.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

    // Get udot at state from trajectory
    int nq = s.getNQ();
    SimTK::Vector s_udot(nq);
    for (int k = 0; k < nq; k++) {
        s_udot.set(k, udot.getRowAtIndex(stateInd).getAnyElt(0, k));
    }

	//=============================================================================
    //  TODO: use force component...
    //=============================================================================

    // Get right side force component from model
    const auto& contactForce_r =
            osimModel.getForceSet().get("zmpContactForce_r");

    // Get record values from force
    Array<double> recordVals = contactForce_r.getRecordValues(s);
    std::cout << recordVals << std::endl;

    return 0;

}
