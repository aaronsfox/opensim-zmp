/* -------------------------------------------------------------------------- *
 *         OpenSim:  exampleZeroMomentPoint1_createAndConnectComponents.cpp   *
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

/* -------------------------------------------------------------------------- *
 *                                                                            *
 * This example demonstrates how to create and connect the zero moment point  *
 * ground reactions model component to a model, and specify associated        *
 * properties. Note that this example would not work in a real context, as    *
 * model is completely blank and therefore the bodies referred to in the      *
 * component to not exist. Other examples in this set provide a more          *
 * example of how this component could be used.                               *
 *                                                                            *
 * -------------------------------------------------------------------------- */

/** TODO: 
    > Consider including the force component once created?
*/

int main() {
	
    using namespace OpenSim;
    
    // Create a dummy blank model to allow connecting the components
    Model osimModel;

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

    // Set the distance checking threshold
    zmpGroundReactions->setDistanceThreshold(0.02);

    // Set the force checking threshold
    zmpGroundReactions->setForceThreshold(50.0);
    
    //=============================================================================
    //  Create and append ZeroMomentContactBody components
    //=============================================================================

    // Note that these bodies and points are completely made up and aren't indicative
    // of anything in the model. Given the model is blank the bodies being referred
    // to don't exist, so names typical of model feet bodies are used as examples.

    // Add the right calcaneus as a contact body
    zmpGroundReactions->addContactBodyZMP("calcn_r_contact", "calcn_r");

    // Add a couple of random points to the contact body as the checking points
    // for ground contact. Note that points can be specified on different bodies.
    zmpGroundReactions->upd_zmp_contact_bodies(0).addContactBodyPoint(
            "contact_point_r_1", "calcn_r", SimTK::Vec3(0.1, 0.2, 0.3));
    zmpGroundReactions->upd_zmp_contact_bodies(0).addContactBodyPoint(
            "contact_point_r_2", "toes_r", SimTK::Vec3(0.3, 0.2, 0.1));

    // Add the left calcaneus as a contact body
    zmpGroundReactions->addContactBodyZMP("calcn_l_contact", "calcn_l");

    // Add a couple of random points to the contact body as the checking points
    // for ground contact. Note that points can be specified on different
    // bodies.
    zmpGroundReactions->upd_zmp_contact_bodies(1).addContactBodyPoint(
            "contact_point_l_1", "calcn_l", SimTK::Vec3(0.1, 0.2, 0.3));
    zmpGroundReactions->upd_zmp_contact_bodies(1).addContactBodyPoint(
            "contact_point_l_2", "toes_l", SimTK::Vec3(0.3, 0.2, 0.1));

    //=============================================================================
    //  Demonstrate use of checking for number of contact bodies added
    //=============================================================================

    // Get number of contact bodies in the component
    int nCB = zmpGroundReactions->getNumContactBodiesZMP();

    // Print the number of contact bodies in model
    std::cout << "Number of ZMP contact bodies in component: " + std::to_string(nCB) << std::endl;
    
    //=============================================================================
    //  Add the component to the model
    //=============================================================================

    // Append the component to the model
    osimModel.addComponent(zmpGroundReactions);

    // Finalize model connections
    osimModel.finalizeConnections();

    // Print model to file
    osimModel.print("osimModel_ZMP.osim");

    return 0;

}
