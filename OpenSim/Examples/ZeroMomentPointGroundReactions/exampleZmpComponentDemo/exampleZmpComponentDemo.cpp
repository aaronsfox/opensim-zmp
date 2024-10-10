/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleZmpComponentDemo.cpp   						      *
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
#include "OpenSim/Simulation/SimbodyEngine/ZeroMomentPointContactPointConstraint.h"
#include "OpenSim/Moco/MocoTrajectory.h"

#include "OpenSim/Simulation/SimbodyEngine/RollingOnSurfaceConstraint.h"

//=============================================================================
//=============================================================================
/**
 * This example demonstrates how to create and connect the Zero Moment Point 
 * Ground Reactions component to a model, and specify associated properties.
 * This includes adding relevant contact bodies and contact points on these
 * bodies for the model. It also adds the ZeroMomentPointContactForce objects
 * that can eventually apply the estimated ground reactions. Lastly, the 
 * ZeroMomentPointContactPointConstraints are added to the model.The example also
 * runs through some associated functions of the component. 
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
        new ZeroMomentPointContactBody("calcn_r_contact", "calcn_r", "distance");
    ZeroMomentPointContactBody* contactBodyLeft = 
        new ZeroMomentPointContactBody("calcn_l_contact", "calcn_l", "distance");

    // Set the desired distance and velocity thresholds for contact checking
    double distThreshold = 0.04;
    double velThreshold = 1.50;

    // The model has a number of markers added to each foot to reference for
    // adding the contact points. These are added here with the specified
    // distance and velocity threshold checking limits.

    // TODO: this needs to change now as the body is now connected via socket...

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
            osimModel.getMarkerSet().get(contactName + "_ground").get_location(),
            distThreshold, velThreshold
            );
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
            osimModel.getMarkerSet().get(contactName + "_ground").get_location(),
            distThreshold, velThreshold);
    }

    // Add the contact bodies to the ZMP ground reactions component
    zmpGroundReactions->upd_ZeroMomentPointContactBodySet().adoptAndAppend(
            contactBodyRight);
    zmpGroundReactions->upd_ZeroMomentPointContactBodySet().adoptAndAppend(
            contactBodyLeft);

    //=============================================================================
    //  Demonstrate use of checking for number of contact bodies added
    //=============================================================================

    // Get number of contact bodies in the component
    int nCB = zmpGroundReactions->get_ZeroMomentPointContactBodySet().getSize();

    // Print the number of contact bodies in model
    std::cout << "Number of ZMP contact bodies in component: " + std::to_string(nCB) << std::endl;

    //=============================================================================
    //  Demonstrate the outputs of the model component
    //=============================================================================

    // Get the model outputs
    std::vector<std::string> componentOutputs = zmpGroundReactions->getOutputNames();

    // Print out component outputs
    for (int ii = 0; ii < componentOutputs.size(); ii++) {        
        std::cout << "Component output: " + componentOutputs.at(ii) << std::endl;
    }
        
    //=============================================================================
    //  Add the component to the model
    //=============================================================================

    // Append the component to the model
    // Ensure connections for component socket is finalized
    osimModel.addComponent(zmpGroundReactions);

    //=============================================================================
    //  Create and append ZeroMomentPointContactPointConstraint components
    //=============================================================================

    // The same arrays of contact points from earlier can be used to add the constraints

    // Right side
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);

        // Create the constraint
        ZeroMomentPointContactPointConstraint* constraint =
                new ZeroMomentPointContactPointConstraint(
                        "constraintPoint_" + contactName,
                        osimModel.getBodySet().get("calcn_r"),
                        osimModel.getGround(),
                        osimModel.getMarkerSet()
                                .get(contactName + "_ground")
                                .get_location());

        // Add the constraint
        osimModel.addConstraint(constraint);

    }

    // Left side
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {

        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);

        // Create the constraint
        ZeroMomentPointContactPointConstraint* constraint =
                new ZeroMomentPointContactPointConstraint(
                        "constraintPoint_" + contactName,
                        osimModel.getBodySet().get("calcn_l"),
                        osimModel.getGround(),
                        osimModel.getMarkerSet()
                                .get(contactName + "_ground")
                                .get_location());

        // Add the constraint
        osimModel.addConstraint(constraint);

    }    

    //=============================================================================
    //  Demonstrate creating and connecting the ZeroMomentPointContactForce
    //=============================================================================

    // New model connections need to be finalised before connecting forces
    osimModel.finalizeConnections();

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
    std::cout << "Added right side ZeroMomentPointContactForce to model" << std::endl;
    osimModel.addForce(zmpContactForce_l);
    std::cout << "Added left side ZeroMomentPointContactForce to model" << std::endl;

    //=============================================================================
    //  Print the model to file for viewing
    //=============================================================================

    // Finalize model connections
    osimModel.finalizeConnections();

    // Print model to file
    osimModel.print("subject01_run3_withZMP.osim");

    //=============================================================================
    //  Demonstrate some methods associated with ZeroMomentPointContactForce
    //=============================================================================

    // Get one of the ZMP contact force objects recording labels
    OpenSim::Array<std::string> labels = osimModel.getForceSet().get(
        "zmpContactForce_r").getRecordLabels();

    // Get number of reporting labels    
    int nLabels = labels.size();

    // Loop through and print out labels
    for (int ii = 0; ii < nLabels; ii++) { 
        std::cout << labels.get(ii) << std::endl;    
    }

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

    // Get the Moco solution to test with this example
    // NOTE: no joints can be locked otherwise nan's end up in states trajectory
    MocoTrajectory trajectory = MocoTrajectory("subject01_run3_trackingSolution.sto");
    StatesTrajectory statesTraj = trajectory.exportToStatesTrajectory(osimModel);

    // Set the state index from the trajectory to query
    // This index corresponds to t ~ 0.636 from the data where the right foot
    // is in pretty flat contact with the ground
    int i = 17;

    // Get the desired state
    SimTK::State s = statesTraj[i];

    // Initialise the model system for use in function
    osimModel.initSystem();

    // Get back the component from the model for calculations
    const auto& zmpCalculator =
            osimModel.getComponent<ZeroMomentPointGroundReactions>(
                    "zmpGroundReactions");

    // Check ground contact for current state
    SimTK::Vector inContact = zmpCalculator.checkGroundContact(s);

    // The returned vector has boolean values related to the contact body points for
    // specifying if they are/aren't in contact. We can check this and print out
    // some summary statements here.
    if (inContact.get(0)) {
        std::cout << "Contact body calcn_r in contact with ground at queried state." << std::endl;
    } else {
        std::cout << "Contact body calcn_r NOT in contact with ground at queried state." << std::endl;
    }
    if (inContact.get(1)) {
        std::cout << "Contact body calcn_l in contact with ground at queried state." << std::endl;
    } else {
        std::cout << "Contact body calcn_l NOT in contact with ground at queried state." << std::endl;
    }

    return 0;

}
