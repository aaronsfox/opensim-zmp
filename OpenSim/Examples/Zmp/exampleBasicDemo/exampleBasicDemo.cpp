/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleComponentDemo.cpp   						      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2025 Stanford University and the Authors                     *
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

// TODO: not sure if all of these inclusions are necessary

#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/Marker.h"
#include "OpenSim/Actuators/CoordinateActuator.h"

#include "OpenSim/Simulation/Model/ContactPoint.h"
#include "OpenSim/Simulation/Model/ContactPointSet.h"
#include "OpenSim/Simulation/Model/ZmpGroundReactions.h"

//#include "OpenSim/Simulation/Model/ZmpForce.h"
//#include "OpenSim/Zmp/ZmpContactPointConstraint.h"

#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/SimulationUtilities.h"
#include "OpenSim/Moco/MocoTrajectory.h"
#include "OpenSim/Common/IO.h"
#include "OpenSim/Common/STOFileAdapter.h"
//#include "OpenSim/Simulation/SimbodyEngine/RollingOnSurfaceConstraint.h"

//=============================================================================
//=============================================================================
/**
 * 
 * This example demonstrates how to create and connect the Zero Moment Point 
 * Ground Reactions component to a model, and specify associated properties.
 * This includes adding relevant contact points into the component for ground
 * contact checking.
 * 
 * TODO: any other classes added/included?
 * 
 * TODO: demo explores relevant features, or refer to other examples?
 * 
 * See the README.txt next to this file for more information on the data used.
 */

int main() {
	
    using namespace OpenSim;
    
    // Read in the example model to connect components
    Model osimModel("subject01_run3.osim");

    //=============================================================================
    //  Create the base ZeroMomentGroundReactions component
    //=============================================================================

    // Create an instance of the component
     ZmpGroundReactions* zmpGroundReactions =
             new ZmpGroundReactions();

    // Set the overall name of the component
     zmpGroundReactions->setName("zmpGroundReactions");

    // Set the free joint name
     zmpGroundReactions->setFreeJointName("ground_pelvis");

    // Set the force checking threshold
     zmpGroundReactions->setForceThreshold(50.0);

    //=============================================================================
    //  Udpdate the ContactPointSet of the component to include contact points
    //=============================================================================

    // The model has a number of markers added to each foot to reference for
    // adding the contact points. These are added here with the default
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

    // Add the contact points using the marker locations on right foot
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);

        // Get the marker associated to the current point name
        Marker pointMarker =
                osimModel.getMarkerSet().get(contactName + "_ground");

        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();

        // Add the contact point to the ZMP component
        zmpGroundReactions->addContactPoint(
            "contact_point_" + contactName,
            osimModel.getBodySet().get("calcn_r"),
            pointLocation);

    }

    // Left side
    Array<std::string> contactPointsLeft;
    contactPointsLeft.append("LCAL");
    contactPointsLeft.append("LTOE");
    contactPointsLeft.append("LMT5");
    contactPointsLeft.append("LTOE_mid");
    contactPointsLeft.append("LMT5_mid");
    contactPointsLeft.append("L_end");

    // Add the contact points using the marker locations on left foot
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);

        // Get the marker associated to the current point name
        Marker pointMarker =
                osimModel.getMarkerSet().get(contactName + "_ground");

        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();

        // Add the contact point to the ZMP component
        zmpGroundReactions->addContactPoint(
            "contact_point_" + contactName,
            osimModel.getBodySet().get("calcn_l"),
            pointLocation);

    }

    // Get number of contact points in the component
     int nCP = zmpGroundReactions->get_ContactPointSet().getSize();

    // Print the number of contact points in model
    std::cout << "Number of contact points in ZMP component: " << std::to_string(nCP) << std::endl;
     
    //=============================================================================
    //  Add the component to the model
    //=============================================================================

    // append the component to the model
    osimModel.addComponent(zmpGroundReactions);
    
    //=============================================================================
    //  Print the model to file for viewing
    //=============================================================================

    // Finalize model connections
    osimModel.finalizeConnections();

    // Print model to file
    osimModel.print("subject01_run3_withZMP.osim");

    //=============================================================================
    //  Demonstrate the function for getting contact bodies from component
    //=============================================================================
    
    // Get the contact bodies from the component based on the contact point set
    std::vector<std::string> bodies = zmpGroundReactions->getContactBodyNames();

    // Print the body names
    for (const auto& str : bodies) { 
        std::cout << "Identified contact body: " << str << " " << std::endl;
    }

    //=============================================================================
    //  Demonstrate functionality of using motion data to generate outputs
    //=============================================================================
    
    // TODO: essentially want to write functions in ZMP component that replicate 
    // generating the values, speeds, accel tables etc. to feed into functions...

    // Get the Moco trajectory that comes with this example and convert it to
    // a TimeSeriesTable of coordinate values
    MocoTrajectory mocoTraj = MocoTrajectory("subject01_run3_trackingSolution.sto");
    TimeSeriesTable values = mocoTraj.exportToValuesTable();
    //TimeSeriesTable values = mocoTraj.exportToSpeedsTable();

    // Calculate ZMP estimates of ground reactions from provided motion
    Storage zmpResults = zmpGroundReactions->calcFromQ(values);


    /*mocoTraj.generateAccelerationsFromSpeeds();
    TimeSeriesTable udot = mocoTraj.exportToAccelerationsTable();    
    TimeSeriesTable speeds = mocoTraj.exportToSpeedsTable();
    STOFileAdapter::write(values, "values_table.sto");
    STOFileAdapter::write(speeds, "speeds_table.sto");
    STOFileAdapter::write(udot, "acell_table.sto");*/

    //StatesTrajectory states = 
    //    zmpGroundReactions->convertMotionToStatesTrajectory(table);
    /*std::cout << states[0];*/

    //=============================================================================
    //  Demonstrate the outputs of the model component
    //=============================================================================

    //// Get the model outputs
    //std::vector<std::string> componentOutputs = zmpGroundReactions->getOutputNames();

    //// Print out component outputs
    //for (int ii = 0; ii < componentOutputs.size(); ii++) {        
    //    std::cout << "Component output: " + componentOutputs.at(ii) << std::endl;
    //}

    //=============================================================================
    //  Create and append ZeroMomentPointContactPointConstraint components
    //=============================================================================

    //// The same arrays of contact points from earlier can be used to add the constraints

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

    //=============================================================================
    //  Demonstrate creating and connecting the ZeroMomentPointContactForce
    //=============================================================================

    //// New model connections need to be finalised before connecting forces
    //osimModel.finalizeConnections();

    //// Get the ZeroMomentPointGroundReactions object back from the model
    ///*ZeroMomentPointGroundReactions zmpComponent =
    //        osimModel.getComponent<ZeroMomentPointGroundReactions>(
    //                "zmpGroundReactions");*/

    //// Use the constructors to create a force for both sides
    //ZeroMomentPointContactForce* zmpContactForce_r =
    //        new ZeroMomentPointContactForce();
    //ZeroMomentPointContactForce* zmpContactForce_l =
    //        new ZeroMomentPointContactForce();

    //// Set names
    //zmpContactForce_r->setName("zmpContactForce_r");
    //zmpContactForce_l->setName("zmpContactForce_l");

    //// Set the Zero Moment Point contact body names
    //zmpContactForce_r->setZeroMomentPointContactBodyName("calcn_r_contact");
    //zmpContactForce_l->setZeroMomentPointContactBodyName("calcn_l_contact");
    //
    //// Set applied to body names
    //zmpContactForce_r->setAppliedToBody("calcn_r");
    //zmpContactForce_l->setAppliedToBody("calcn_l");

    //// Connect to ZMP ground reactions object
    //zmpContactForce_r->setZeroMomentPointGroundReactions(*zmpGroundReactions);
    //zmpContactForce_l->setZeroMomentPointGroundReactions(*zmpGroundReactions);

    //// Add the forces to the model
    //osimModel.addForce(zmpContactForce_r);
    //std::cout << "Added right side ZeroMomentPointContactForce to model" << std::endl;
    //osimModel.addForce(zmpContactForce_l);
    //std::cout << "Added left side ZeroMomentPointContactForce to model" << std::endl;

    //=============================================================================
    //  Demonstrate some methods associated with ZeroMomentPointContactForce
    //=============================================================================

    //// Get one of the ZMP contact force objects recording labels
    //OpenSim::Array<std::string> labels = osimModel.getForceSet().get(
    //    "zmpContactForce_r").getRecordLabels();

    //// Get number of reporting labels    
    //int nLabels = labels.size();

    //// Loop through and print out labels
    //for (int ii = 0; ii < nLabels; ii++) { 
    //    std::cout << labels.get(ii) << std::endl;    
    //}

    //=============================================================================
    //  Demonstrate ability to check ground contact at a particular state
    //=============================================================================

    // When estimating ground reactions using the ZMP method, the contact body points
    // are queried at each state to determine whether ground contact is occurring.
    // The following code demonstrates this function at a particular state from the
    // included Moco tracking solution. The dataset included with this particular 
    // example is treadmill running, which is better suited to using the distance only
    // method for checking ground contact given that the treadmill belt moves the foot
    // during this period. This distance only approach was set with each of the contact
    // bodies when added earlier.

    //// Get the Moco solution to test with this example
    //// NOTE: no joints can be locked otherwise nan's end up in states trajectory
    //MocoTrajectory trajectory = MocoTrajectory("subject01_run3_trackingSolution.sto");
    //StatesTrajectory statesTraj = trajectory.exportToStatesTrajectory(osimModel);

    //// Set the state index from the trajectory to query
    //// This index corresponds to t ~ 0.636 from the data where the right foot
    //// is in pretty flat contact with the ground
    //int i = 17;

    //// Get the desired state
    //SimTK::State s = statesTraj[i];

    //// Initialise the model system for use in function
    //osimModel.initSystem();

    //// Get back the component from the model for calculations
    //const auto& zmpCalculator =
    //        osimModel.getComponent<ZeroMomentPointGroundReactions>(
    //                "zmpGroundReactions");

    //// Check ground contact for current state
    //SimTK::Vector inContact = zmpCalculator.checkGroundContact(s);

    //// The returned vector has boolean values related to the contact body points for
    //// specifying if they are/aren't in contact. We can check this and print out
    //// some summary statements here.
    //if (inContact.get(0)) {
    //    std::cout << "Contact body calcn_r in contact with ground at queried state." << std::endl;
    //} else {
    //    std::cout << "Contact body calcn_r NOT in contact with ground at queried state." << std::endl;
    //}
    //if (inContact.get(1)) {
    //    std::cout << "Contact body calcn_l in contact with ground at queried state." << std::endl;
    //} else {
    //    std::cout << "Contact body calcn_l NOT in contact with ground at queried state." << std::endl;
    //}

    return 0;

}
