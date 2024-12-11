/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointGroundReactions.cpp                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Aaron Fox                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "ZeroMomentPointGroundReactions.h"
#include "ZeroMomentPointContactBody.h"
#include "ZeroMomentPointContactBodySet.h"
#include "ZeroMomentPointContactPoint.h"
#include "ZeroMomentPointContactPointSet.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include "OpenSim/Common/IO.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;
//=============================================================================
//=============================================================================
/**
 * The ZeroMomentPointGroundReactions is a model component used to calculate
 * estimates of ground reactions (i.e. forces, moments and point of application)
 * from a model at a particular state. This allows a prediction of the ground
 * reactions from a motion where experimental measures were unavailable. The
 * component requires the bodies that are expected to come into contact with the
 * ground to be specified, alongside points on the contact bodies (or nearby
 * bodies) that are checked for specifying when ground contact has occurred. The
 * component can also be connected to the ZeroMomentPointContactForce class so
 * that the estimated ground reactions can be applied as external forces to the
 * model during dynamic simulations.
 *
 * Ground reactions are estimated via the Zero Moment Point method when ground
 * contact is identified. Ground contact is identified by first checking if the
 * contact points meet the criteria for ground contact - either based on
 * distance from the ground plane (which is assumed to be the XZ plane) or by
 * both distance to the ground plane and meeting a velocity threshold. A force
 * threshold is also specified for if ground contact has occurred, whereby if
 * the vertical force (assumed to be in the y-direction) is greater than the
 * force threshold then ground contact is occurring.
 *
 * The Zero Moment Point method is outlined in Xiang et al. (2009) and an
 * implementation to predicting GRFs during gait explored by Dijkstra &
 * Gutierrez-Farewik (2015). The method for checking ground contact with points
 * on a body is similar to the method implemented in Karcnik (2003).
 *
 * Xiang et al. (2009): https://doi.org/10.1002/nme.2575
 * Dijkstra & Gutierrez-Farewik (2015):
 * https://doi.org/10.1016/j.jbiomech.2015.08.027 Karcnik  (2003):
 * https://doi.org/10.1007/BF02345310).
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ZeroMomentPointGroundReactions::ZeroMomentPointGroundReactions() {
    
    // Construct with default properties
    constructProperties();

}

/** Constructor specifying free joint name with other defaults. */
ZeroMomentPointGroundReactions::ZeroMomentPointGroundReactions(
        const std::string& freeJointName) {

    // Construct with default properties
    constructProperties();

    // Set the free joint name
    set_free_joint_name(freeJointName);

}

/** Constructor specifying free joint name and force threshold. */
ZeroMomentPointGroundReactions::ZeroMomentPointGroundReactions(
    const std::string& freeJointName, const double& forceThreshold) {

    // Construct with default properties
    constructProperties();

    // Set the free joint name
    set_free_joint_name(freeJointName);

    // Set the force threshold
    set_force_threshold(forceThreshold);

}

/** Construct default properties */
void ZeroMomentPointGroundReactions::constructProperties() {

    // Contact body set
    ZeroMomentPointContactBodySet contactBodySet;
    contactBodySet.setName(
            IO::Lowercase(contactBodySet.getConcreteClassName()));
    constructProperty_ZeroMomentPointContactBodySet(contactBodySet);

    // Standard properties
    constructProperty_free_joint_name("ground_pelvis");
    constructProperty_force_threshold(20.0);
}

/** Finalize properties */
void ZeroMomentPointGroundReactions::extendFinalizeFromProperties() {}

/** Connection to model */
void ZeroMomentPointGroundReactions::extendConnectToModel(Model& model) {
    
    // Base class first
    Super::extendConnectToModel(model);

}

/** Topology and creating contact body map */
void ZeroMomentPointGroundReactions::extendRealizeTopology(
        SimTK::State& state) const {
    
    // Base class first
    Super::extendRealizeTopology(state);

    // Clear contact body map variable
    m_contactBodyIndices.clear();

    // Assign indices in map
    for (int ii = 0; ii < get_ZeroMomentPointContactBodySet().getSize(); ++ii) {
        const ZeroMomentPointContactBody& contactBody = get_ZeroMomentPointContactBodySet().get(ii);
        m_contactBodyIndices[contactBody.getName()] = ii;        
    }

}

/** Add component to system */
void ZeroMomentPointGroundReactions::extendAddToSystem(
        SimTK::MultibodySystem& system) const {

    // Base class first
    Super::extendAddToSystem(system);

    // Set cache variables
    /*this->_groundReactionsCV = addCacheVariable("ground_reactions", 
        SimTK::Vector((int)m_contactBodyIndices.size() * 9, 0.0),
        SimTK::Stage::Dynamics);*/

}

//=============================================================================
//  METHODS
//=============================================================================

/** Specify a contact body that should be considered when reviewing if
the model has come into contact with the ground plane. Uses default
value of velocity for contact checking method.*/
void ZeroMomentPointGroundReactions::addContactBody(
    const std::string& name, const std::string& bodyName) {

    // Create the contact body with default properties
    /*ZeroMomentPointContactBody& cb =
            ZeroMomentPointContactBody();*/
    ZeroMomentPointContactBody* cb =
            new ZeroMomentPointContactBody();

    // Set the name
    cb->setName(name);

    // Set the body name
    cb->set_body_name(bodyName);

    // Append to the contact body set
    //upd_ZeroMomentPointContactBodySet().cloneAndAppend(cb);
    updZeroMomentPointContactBodySet().adoptAndAppend(cb);

    // TODO: needed somewhere?
    //// Finalize properties
    //finalizeFromProperties();
    //prependComponentPathToConnecteePath(cb);

};

/** Specify a contact body that should be considered when reviewing if
the model has come into contact with the ground plane, while also
specifying the contact checking method.*/
void ZeroMomentPointGroundReactions::addContactBody(
        const std::string& name, const std::string& bodyName,
        const std::string& contactCheckingMethod) {

    // TODO: checks in place for contact checking method strings

    // Create the contact body with default properties
    /*ZeroMomentPointContactBody& cb = 
        ZeroMomentPointContactBody();*/
    ZeroMomentPointContactBody* cb = 
        new ZeroMomentPointContactBody();

    // Set the name
    cb->setName(name);

    // Set the body name
    cb->set_body_name(bodyName);

    // Set the contact checking method
    cb->set_zmp_contact_checking_method(contactCheckingMethod);

    // Append to the contact body set
    //upd_ZeroMomentPointContactBodySet().cloneAndAppend(cb);
    updZeroMomentPointContactBodySet().adoptAndAppend(cb);

    // TODO: needed somewhere?
    //// Finalize properties
    //finalizeFromProperties();
    //prependComponentPathToConnecteePath(cb);

};

/** Set the free joint name in the component. */
void ZeroMomentPointGroundReactions::setFreeJointName(
        const std::string& free_joint_name) {
    set_free_joint_name(free_joint_name);
};

/** Get and set the force threshold for considering when ground contact
has occurred. */
void ZeroMomentPointGroundReactions::setForceThreshold(
        const double& force_threshold) {
    set_force_threshold(force_threshold);
}

//=============================================================================
//  CALCULATIONS
//=============================================================================

/** Check ground contact of each contact body specified. Returns a vector
of 1 or 0 for each contact body specifying if contact has or has not
occurred. */
SimTK::Vector ZeroMomentPointGroundReactions::checkGroundContact(
        const SimTK::State& s) const {

    /* This function takes the state and checks if the contact points allocated
    to a body are in contact with the ground based on a distance and velocity
    threshold. If any points meet the criteria then the body is said to be in
    contact with the ground. However, if useVelocity is set to FALSE then only
    the distance threshold is considered (i.e. velocity ignored). 
    
    This approach reflects that outlined in Karcnik (2003):
    https://doi.org/10.1007/BF02345310 */

    // Get the model
    const Model& model = getModel();

    // Realize to acceleration stage
    model.getMultibodySystem().realize(s, SimTK::Stage::Acceleration);

    // Get the number of contact bodies
    const int nCB = get_ZeroMomentPointContactBodySet().getSize();

    // Create a vector to store whether the array is vs. isn't in ground contact
    SimTK::Vector bodyInContact = SimTK::Vector(nCB);

    // Loop through contact bodies and check for contact
    for (int iCB = 0; iCB < (nCB); iCB++) {

        // Get the contact checking method for the current body
        std::string checkingMethod = get_ZeroMomentPointContactBodySet()
                                             .get(iCB)
                                             .get_zmp_contact_checking_method();

        // Get the number of contact body points linked to this contact body
        int nCBP = get_ZeroMomentPointContactBodySet()
                           .get(iCB).get_ZeroMomentPointContactPointSet().getSize();

        // Create a vector to store if points are in contact
        SimTK::Vector pointInContact = SimTK::Vector(nCBP);

        // Loop through contact body points for the current contact body
        for (int iCBP = 0; iCBP < (nCBP); iCBP++) {

            // Get the body name, location and thresholds for the
            // current contact point
            ZeroMomentPointContactPoint& contactPoint =
                    get_ZeroMomentPointContactBodySet()
                            .get(iCB)
                            .get_ZeroMomentPointContactPointSet()
                            .get(iCBP);
            std::string bodyName = contactPoint.getParentFrame().getName();
            SimTK::Vec3 pointLoc = contactPoint.get_location();
            double distanceThreshold = contactPoint.get_distance_threshold();
            double velocityThreshold = contactPoint.get_velocity_threshold();

            // Check if point is in contact based on proposed checking method
            if (checkingMethod == "velocity") {

                std::cout << "TODO: distance and velocity contact checking"
                          << std::endl;

            } else {

                // Calculate the position of the current contact point in the
                // ground and it's vertical level (y-axis). Check if this
                // vertical level is within the the specified distance
                // threshold.
                if (model.getBodySet()
                                .get(bodyName)
                                .findStationLocationInGround(s, pointLoc)
                                .get(1) < distanceThreshold) {

                    // Specify that the point is in contact
                    pointInContact.set(iCBP, 1);

                } else {

                    // Specify that the point is not in contact
                    pointInContact.set(iCBP, 0);

                }
            }
        }

        // If any of the points are in contact then the body can be set as
        // in contact
        if (pointInContact.sum() > 0) {

            // At least one of the points is in contact with the ground
            bodyInContact.set(iCB, TRUE);

        } else {

            // The contact body point is not in contact with the ground
            bodyInContact.set(iCB, FALSE);

        }

    }

    // Return the vector of if the bodies are in contact
    return bodyInContact;

}

/** The below functions calculate the Zero Moment Point of the model based
on the Model state and any other provided inputs. It identifies the ground
reaction forces, moments and centre of pressure for each contact body
listed in the component. The output is returned as a Vector which size is
based on the number of contact bodies. Each component of the Vector contains
a spatial vector that contains the separate force, moment and point
components.

    i.e.

        FX, FY, FZ, PX, PY, PZ, MX, MY, MZ

which is repeated for the number of contact bodies in the component.*/

// TODO: consider using Vector of SpatialVec for output from ground reactions...

/** Calculate ground reactions from state. This is a simpler function that
uses the state to get udot and then feeds back to the other function. */
SimTK::Vector ZeroMomentPointGroundReactions::getGroundReactions(
        const SimTK::State& s) const {

    // Get the model
    const auto& model = getModel();

    // Realize to appropriate stage
    model.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

    // Get accelerations from state
    // TODO: is this the right way to do it? Comes out as zeros sometimes?
    SimTK::Vector udot = s.getUDot();

    // Feed state and udot into detailed function to return ground reactions
    SimTK::Vector groundReactions = getGroundReactions(s, udot);

    return groundReactions;

}

/** Calculate ground reactions with state and udot accelerations vector.*/
SimTK::Vector ZeroMomentPointGroundReactions::getGroundReactions(
        const SimTK::State& s, const SimTK::Vector& udot) const {

    ///* TODO: mapping if q != u in index (i.e.tree vs.model)... */
    ///     - Not sure it matters as calcEquivalentSpatial force seems to work...
    ///* TODO: getting the MY calculation correct - currently staying as zero...
    ///*/

    // Get the model
    const auto& model = getModel();

    // Get the free body name and associated joint
    const std::string freeJointName = get_free_joint_name();
    const Joint& freeJoint = model.getJointSet().get(freeJointName);

    // Get the number of contact bodies
    const int nCB = get_ZeroMomentPointContactBodySet().getSize();

    // Get the desired force threshold property
    const double forceThreshold = get_force_threshold();

    // Create the vector to fill with the calculated ground reactions
    // Size is based on the number of contact bodies * 3x3 (F, M and P)
    SimTK::Vector groundReactionsVec = SimTK::Vector(9 * nCB);

    // Set all zeros as default in ground reactions
    for (int iCol = 0; iCol < (9 * nCB); iCol++) {
        groundReactionsVec.set(iCol, 0.0);
    }

    // Initialise an inverse dynamics solver with model
    InverseDynamicsSolver ivdSolver(model);

    // Realize to appropriate stage
    model.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

    // Check the contact bodies for ground contact
    SimTK::Vector inContact(nCB);
    inContact = checkGroundContact(s);

    // Determine whether contact has occurred
    bool contactOccurring = FALSE;
    if (inContact.sum() > 0) { contactOccurring = TRUE; }

    // Proceed through subsequent calculations only if contact occurring
    if (contactOccurring) {

        // Solve inverse dynamics given current states and udot
        // The output vector contains the generalised coordinate forces
        // to generate the accelerations based on the current state.
        // Note that these aren't necessarily in the order of the
        // coordinate set, but rather the multibody tree order.
        SimTK::Vector genForceTraj = ivdSolver.solve(s, udot);

        // Calculate the equivalent body force at the free joint in the model
        SimTK::SpatialVec equivalentBodyForceAtJoint =
                freeJoint.calcEquivalentSpatialForce(s, genForceTraj);

        // Extract the body and torque components in ground frame
        SimTK::Vec3 freeBodyTorque = equivalentBodyForceAtJoint.get(0);
        SimTK::Vec3 freeBodyForce = equivalentBodyForceAtJoint.get(1);

        // Run a secondary check to see if free body vertical force (assumes
        // y-axis) if greater than the desired force threshold
        if (freeBodyForce.get(1) > forceThreshold) {

            // Get the position of the free body in the ground frame
            // This is based on getting the child frame of the free joint
            // and hence assumes that the ground should always be the parent.
            // Getting the frames position in the ground should represent it's
            // translational coordinates, as it appears robust to frame
            // translation in the model. The frames origin (i.e. 0,0,0) can be
            // used as the station. An adaptation to this could be to use the
            // centre of mass of the body frame to estimate of the ground
            // reactions.
            SimTK::Vec3 rp =
                    freeJoint.getChildFrame().findStationLocationInGround(
                            s, SimTK::Vec3(0, 0, 0));             

            // Calculate moment at origin
            // Formulas used here come from Xiang et al. 2009:
            // https://doi.org/10.1002/nme.2575
            SimTK::Vec3 rp_fbf = SimTK::cross(rp, freeBodyForce);
            SimTK::Vec3 groundM =
                    SimTK::Vec3(freeBodyTorque.get(0) + rp_fbf.get(0),
                            freeBodyTorque.get(1) + rp_fbf.get(1),
                            freeBodyTorque.get(2) + rp_fbf.get(2));

            // Calculate X & Z cZMP, noting that yZMP is set as 0
            // Given this (and some other calculations) the standard OpenSim
            // coordinate system must be used.
            // Formula used here come from Xiang et al. 2009:
            // https://doi.org/10.1002/nme.2575
            SimTK::Vec3 zmpCOP =
                    SimTK::Vec3(groundM.get(2) / freeBodyForce.get(1), 0,
                        -groundM.get(0) / freeBodyForce.get(1));

            // Calculate the resultant active moment at ZMP along the y-axis
            // TODO: This formula still doesn't seem quite right?
            double myZMP = groundM.get(1) +
                           (freeBodyForce.get(0) * zmpCOP.get(2)) -
                           (freeBodyForce.get(2) * zmpCOP.get(1));

            // Check for unilateral contact
            if (inContact.sum() == 1) {

                // If unilateral contact is flagged, then the force is simply
                // allocated to the body that is deemed in contact with the
                // ground.

                // Figure out which body index is contacting the ground
                int contactInd;
                for (int bb = 0; bb < nCB; bb++) {
                    if (inContact.get(bb) == 1) { contactInd = bb; }
                }

                // Set the values at the appropriate indices in the ground
                // reactions vector Note the MX (index = 6*nCB) and MZ (index =
                // 8 * nCB) remain set as zero
                groundReactionsVec.set(
                        contactInd * 9 + 0, freeBodyForce.get(0));
                groundReactionsVec.set(
                        contactInd * 9 + 1, freeBodyForce.get(1));
                groundReactionsVec.set(
                        contactInd * 9 + 2, freeBodyForce.get(2));
                groundReactionsVec.set(contactInd * 9 + 3, zmpCOP.get(0));
                groundReactionsVec.set(contactInd * 9 + 4, zmpCOP.get(1));
                groundReactionsVec.set(contactInd * 9 + 5, zmpCOP.get(2));
                /*groundReactionsVec.set(contactInd * 9 + 7, myZMP);*/

            } else {

                // TODO: dealing with bilateral contact
                // There can probably only be a max of 2 contact bodies given
                // method used
                std::cout << "TODO: calculations with bilateral contact"
                          << std::endl;
            }
        }
    }

    return groundReactionsVec;
}

/** These functions calculate the Zero Moment Point of the model based
on a series of states from a predefined motion. It identifies the ground
reaction forces, moments and centre of pressure for each contact body
listed in the component across the states provided. The output is returned
as a Storage with the number of columns based on the number of contact
bodies and the separate force, moment and point components.

i.e.

    FXn, FYn, FZn, PXn, PYn, PZn, MXn, MYn, MZn

where n is repeated for the number of contact bodies specified.*/

/** Calculate ground reactions from a provided states trajectory and
accelerations table. */
Storage ZeroMomentPointGroundReactions::getGroundReactionsFromMotion(
        const StatesTrajectory& states, const TimeSeriesTable& udot) const {

    // Get the model
    const Model& model = getModel();

    // Create the table to store ZMP results in
    Storage zmpResults;

    // Set the name in the ZMP storage
    zmpResults.setName("ZMP Estimated Ground Reactions");

    // Define number of times from states trajectory
    int nt = states.getSize();

    // Get number of contact bodies in the component
    const int nCB = get_ZeroMomentPointContactBodySet().getSize();

    // Set the contact body names for labelling
    Array<std::string> contactBodyNames;
    for (int iCB = 0; iCB < nCB; iCB++) {
        std::string labelBody = get_ZeroMomentPointContactBodySet()
            .get(iCB)
            .get_body_name();
        contactBodyNames.append(labelBody);
    }

    // Create the columns for storing ground reactions based on body names
    // Ordering is Fx, Fy, Fz, Px, Py, Pz, Mx, My, Mz
    Array<std::string> zmpLabels("time", 9 * nCB);
    for (int iCB = 0; iCB < nCB; iCB++) {
        zmpLabels.set(iCB * 9 + 1, contactBodyNames.get(iCB) + "_force_vx");
        zmpLabels.set(iCB * 9 + 2, contactBodyNames.get(iCB) + "_force_vy");
        zmpLabels.set(iCB * 9 + 3, contactBodyNames.get(iCB) + "_force_vz");
        zmpLabels.set(iCB * 9 + 4, contactBodyNames.get(iCB) + "_force_px");
        zmpLabels.set(iCB * 9 + 5, contactBodyNames.get(iCB) + "_force_py");
        zmpLabels.set(iCB * 9 + 6, contactBodyNames.get(iCB) + "_force_pz");
        zmpLabels.set(iCB * 9 + 7, contactBodyNames.get(iCB) + "_torque_x");
        zmpLabels.set(iCB * 9 + 8, contactBodyNames.get(iCB) + "_torque_y");
        zmpLabels.set(iCB * 9 + 9, contactBodyNames.get(iCB) + "_torque_z");
    }

    // Set the column labels in table
    zmpResults.setColumnLabels(zmpLabels);

    // Loop through times to calculate ground reactions at each state
    for (int i = 0; i < nt; i++) {

        // Get the current state
        SimTK::State s = states[i];

        // Get number of coordinates from state
        int nq = s.getNQ();

        // Get accelerations from the Moco trajectory at the current state
        SimTK::Vector s_udot(nq);
        for (int k = 0; k < nq; k++) {
            s_udot.set(k, udot.getRowAtIndex(i).getAnyElt(0, k));
        }

        // Calculate ground reactions from state and udot via convenience function
        SimTK::Vector groundReactions = getGroundReactions(s, s_udot);

        // Create a state vector with the time and ZMP values
        StateVector zmpStateVec = StateVector(s.getTime(), groundReactions);

        // Append the state vector to the storage object
        zmpResults.append(zmpStateVec);
    
    }

    return zmpResults;

}

//=============================================================================
//  OUTPUTS
//=============================================================================

/** The below function allows getting the ground reactions from the component as
a model output. The output takes the form of SimTK::Vectors that specifies the
forces, points and moments in the same way as the calculation functions:

i.e.

    FXn, FYn, FZn, PXn, PYn, PZn, MXn, MYn, MZn

where n is repeated for the number of contact bodies specified.*/




/** Get the forces estimated for the specified contact body. */
SimTK::Vec3 ZeroMomentPointGroundReactions::getContactBodyForces(
    const SimTK::State& s, const std::string& bodyName) const 
{

    /*if (isCacheVariableValid(s, _groundReactionsCV)) {
        return getCacheVariableValue(s, _groundReactionsCV);
    }*/

    

    //// Run the calculation function with the input state
    //SimTK::Vector groundReactionsOut = getGroundReactions(s);

    // Get the contact body index for the specified body object
    const int& bodyInd = m_contactBodyIndices.at(bodyName);

    // TODO: just default values at the moment
    SimTK::Vec3 forces = SimTK::Vec3(0);

    return forces;

}

/** Get the moments estimated for the specified contact body. */
SimTK::Vec3 ZeroMomentPointGroundReactions::getContactBodyMoments(
        const SimTK::State& s, const std::string& bodyName) const {

    //// Run the calculation function with the input state
    // SimTK::Vector groundReactionsOut = getGroundReactions(s);

    // TODO: just default values at the moment
    SimTK::Vec3 moments = SimTK::Vec3(0);

    return moments;
}

/** Get the point of application estimated for the specified contact body. */
SimTK::Vec3 ZeroMomentPointGroundReactions::getContactBodyPoint(
        const SimTK::State& s, const std::string& bodyName) const {

    //// Run the calculation function with the input state
    // SimTK::Vector groundReactionsOut = getGroundReactions(s);

    // TODO: just default values at the moment
    SimTK::Vec3 point = SimTK::Vec3(0);

    return point;
}