/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleZmpGroundReactionsFromState.cpp 	              *
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
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/SimulationUtilities.h"
#include "OpenSim/Moco/MocoTrajectory.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/OpenSim.h"

/** TODO: consider if this is feasible outside of a dynamic simulation? */

//=============================================================================
//=============================================================================
/**
 * This example demonstrates how to create and connect the Zero Moment Point
 * Ground Reactions model component to a model, and read in a motion to generate
 * ground reaction estimates from a singular state. In this example, a gait cycle
 * solution generated from a MocoTrack simulation is used. The singular state
 * examined occurs at peak vertical ground reaction force on the right limb,
 * which corresponds to t ~ 0.636 or the state index 17.
 *
 * See the README.txt next to this file for more information on the data used.
 */

int main() {
	
    using namespace OpenSim;

    // Read in the example model to connect components
    Model osimModel("subject01_run3.osim");

    // Set the state index to calculate ground reactions for
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
    osimModel.addComponent(zmpGroundReactions);

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
    MocoTrajectory mocoTraj = MocoTrajectory("subject01_run3_trackingSolution.sto");
    StatesTrajectory states = mocoTraj.exportToStatesTrajectory(osimModel);
    /*TimeSeriesTable controls = mocoTraj.exportToControlsTable();*/

    // In lieu of a dynamic simulation where udot could be calculated, here
    // we get the accelerations to be set across states
    mocoTraj.generateAccelerationsFromSpeeds();
    TimeSeriesTable udot = mocoTraj.exportToAccelerationsTable();

    // Get the state to calculate ground reactions at
    SimTK::State s = states[stateInd];

    //// TODO: setting controls in model
    //int nc = osimModel.getNumControls();
    //SimTK::Vector s_controls(nc);
    //for (int k = 0; k < nc; k++) {
    //    s_controls.set(k, controls.getRowAtIndex(stateInd).getAnyElt(0, k));
    //}
    //std::cout << "Controls from MocoTrajectory:" << std::endl;
    //std::cout << s_controls << std::endl;

    //osimModel.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
    //osimModel.setControls(s, s_controls);

    //// TODO: checks around controls
    //SimTK::Vector modelControls = osimModel.getControls(s);
    //std::cout << "Starting controls from model:" << std::endl;
    //std::cout << modelControls << std::endl;

    // TODO: remove this as it's just a check to see if uDot ends up the same
    int nq = s.getNQ();
    SimTK::Vector s_udot(nq);
    for (int k = 0; k < nq; k++) {
        s_udot.set(k, udot.getRowAtIndex(stateInd).getAnyElt(0, k));
    }
    std::cout << "Accelerations from MocoTrajectory:" << std::endl;
    std::cout << s_udot << std::endl;

    //// TODO: remove as checking mobility forces after setting controls
    //// This is getting somewhere as forces are being applied...
    //osimModel.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
    //SimTK::Vector appliedMobilityForces =
    //        osimModel.getMultibodySystem().getMobilityForces(
    //            s, SimTK::Stage::Dynamics);
    //std::cout << "Applied Mobility Forces at State:" << std::endl;
    //std::cout << appliedMobilityForces;

    //// TODO: remove as this is checking accelerations from realized state
    //osimModel.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
    //SimTK::Vector man_udot = osimModel.getMatterSubsystem().getUDot(s);
    //std::cout << "Accelerations from State:" << std::endl;
    //std::cout << man_udot << std::endl;


    // NOTE: states trajectory only has values and speeds - so no controls/forces are present... zero actuator forces
    // TODO: delete --- this is just a check of actuator controls...
    // Model Forces --- defaulting to zero so there is no actuator forces
    //osimModel.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
    //auto forces = osimModel.getComponentList<Force>();
    //for (auto& force : forces) {
    //    // If body force we need to record six values for torque+force
    //    // If muscle we record one scalar
    //    if (!force.appliesForce(s)) continue;
    //    Array<double> values = force.getRecordValues(s);
    //    std::cout << force.getName() << std::endl;
    //    std::cout << values << std::endl;
    //}

    // TODO: this is a test from SimbodyEngine Test testJoints.cpp
    // This is getting somewhere --- if calcAcceleration can work then it might get going...
    //const SimTK::SimbodyMatterSubsystem& matter = osimModel.getMatterSubsystem();

    //// The number of mobilities for the entire system.
    //int nm = matter.getNumMobilities();

    //SimTK::Vector genForces(nm, 0.0);
    //SimTK::Random::Uniform genForceRandom(-1000, 1000);
    //for (int i = 0; i < nm; ++i) { genForces[i] = genForceRandom.getValue(); }

    //int nb = matter.getNumBodies();
    //SimTK::Vector_<SimTK::SpatialVec> bodyForces(nb, SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0)));

    //SimTK::Vector udot1(nm);
    //SimTK::Vector_<SimTK::SpatialVec> bodyAccs(nb);

    //osimModel.getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    //matter.calcAcceleration(s, genForces, bodyForces, udot1, bodyAccs);
    //SimTK::Vector man_udot = matter.getUDot(s);
    //std::cout << "Accelerations from manual implementation:" << std::endl;
    //std::cout << man_udot << std::endl;

    //=============================================================================
    //  Get the ZMP estimates of ground reactions from the singular state
    //=============================================================================

    // Get back the ZMP component from the model to use
    const auto& zmpCalculator =
            osimModel.getComponent<ZeroMomentPointGroundReactions>(
                    "zmpGroundReactions");

    //// Use the convenience function to calculate ground reactions from state
    //SimTK::Vector groundReactions =
    //    zmpCalculator.getGroundReactions(s);


    //// Get back the ZMP component from the model to use
    //const auto& zmpCalculator =
    //        osimModel.getComponent<ZeroMomentPointGroundReactions>(
    //                "zmpGroundReactions");

    //// Use the convenience function that takes states and accelerations
    //Storage zmpResults =
    //        zmpCalculator.getGroundReactionsFromMotion(states, udot);

    //// Print result to file
    //Storage::printResult(
    //    &zmpResults, "exampleZMP_fromMotion", ".", -1, ".sto");

    return 0;

}
