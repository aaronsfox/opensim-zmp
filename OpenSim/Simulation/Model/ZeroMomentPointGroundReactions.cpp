/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointGroundReactions.cpp                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

// TODO: any other inclusions?

#include "ZeroMomentPointGroundReactions.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

using namespace OpenSim;


/* -------------------------------------------------------------------------- *
 * ZeroMomentPointGroundReactions                                             *
 * -------------------------------------------------------------------------- *
 * Add instructional content related to the ZeroMomentPointGroundReactions    *
 * class...                                                                   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

//=============================================================================
//  METHODS: ZeroMomentPointGroundReactions
//=============================================================================

// Default constructor
ZeroMomentPointGroundReactions::ZeroMomentPointGroundReactions() {
    constructProperties();

}

// TODO: add any other general constructors?

// Get number of contact bodies
int ZeroMomentPointGroundReactions::getNumContactBodiesZMP() const {
    return getProperty_zmp_contact_bodies().size();
}

// Add a contact body specified with a name label and the contact body name
void ZeroMomentPointGroundReactions::addContactBodyZMP(
    const std::string& name, const std::string& body_name) {

    append_zmp_contact_bodies(ZeroMomentPointContactBody());

    // Get updated parameters for contact body
    auto& cb =
            upd_zmp_contact_bodies(getProperty_zmp_contact_bodies().size() - 1);
    
    // Set the name as specified
    cb.setName(name);

    // Set the body name
    cb.set_body_name(body_name);

};

// TODO: add any other addContactBody constructors...with checkpoints included?

// Set the free joint name in the component
void ZeroMomentPointGroundReactions::setFreeJointName(
        const std::string& free_joint_name) {

    set_free_joint_name(free_joint_name);

};

// Set the distance threshold for a checkpoint to be considered
// in contact with the ground plane.
void ZeroMomentPointGroundReactions::setDistanceThreshold(
        const double& distance_threshold) {

    set_distance_threshold(distance_threshold);

}

// Set the force threshold for considering when ground contact
// has occurred
void ZeroMomentPointGroundReactions::setForceThreshold(
        const double& force_threshold) {

    set_force_threshold(force_threshold);
}

// Constructor properties
void ZeroMomentPointGroundReactions::constructProperties() {
    
    constructProperty_zmp_contact_bodies();

    // Free joint that connects the model to the ground
    constructProperty_free_joint_name("ground_pelvis");

    // Distance threshold to check ground contact with body (default = 0.01m)
    constructProperty_distance_threshold(0.01);

    // Vertical force threshold specifying ground contact (default = 20N)
    constructProperty_force_threshold(20.0);

}

// TODO: needed?
void ZeroMomentPointGroundReactions::extendFinalizeFromProperties() {

}

//=============================================================================
//  FUNCTIONS: ZeroMomentPointGroundReactions
//=============================================================================

// Calculating ground reactions when only the state is provided
SimTK::Vector ZeroMomentPointGroundReactions::calcGroundReactions(
        const SimTK::State& s) const {

    /*This function calculates the Zero Moment Point of the model based on
    the Model state. It identifies the ground reaction forces, moments
    and centre of pressure for each contact body connected to the component.
    The output is returned as a Vector which size is based on the number of
    contact bodies and the separate force, moment and point components, i.e.:
       
    FXn, FYn, FZn, MXn, MYn, MZn, PXn, PYn, PZn --- todo: this order needs to change!
    
    where n is repeated for the number of contact bodies specified.*/

    // TODO:
    //    > Am I Getting variables appropriately?
    //    > This is a more incomplete version of this function...

    // Get the model
    const auto& model = getModel();

    // Get the free body name and associated joint
    const std::string freeJointName = get_free_joint_name();
    const Joint& freeJoint = model.getJointSet().get(freeJointName);

    // Get the number of contact bodies
    const int nCB = getProperty_zmp_contact_bodies().size();

    // Get the desired force threshold property
    const double forceThreshold = get_force_threshold();

    // Create the vector to fill with the calculated ground reactions
    // Size is based on the number of contact bodies * 3x3 (F, M and P)
    SimTK::Vector groundReactionsVec = SimTK::Vector(9 * nCB);

    // OpenSim::Coordinates represent degrees of freedom for a model.
    // Each Coordinate's value and speed maps to an index
    // in the model's underlying SimTK::State (value to a slot in the
    // State's q, and speed to a slot in the State's u).
    // So we need to map each OpenSim::Coordinate value and speed to the
    // corresponding SimTK::State's q and u indices, respectively.
    auto coords = model.getCoordinatesInMultibodyTreeOrder();
    int nq = s.getNQ();
    int nu = s.getNU();
    int nCoords = (int)coords.size(); // TODO: needed?
    // int intUnusedSlot = -1; // TODO: needed?

    // TODO: does the mapCoordinateToQ vector from ID Tool need to be created here?

    // Initialise an inverse dynamics solver with model
    InverseDynamicsSolver ivdSolver(model);

    // Realize to appropriate stage
    model.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

    // Compute accelerations for current state
    // NOTE: unsure whether this produces the desired accelerations?
    // Different results using this versus an entire MocoTrajectory?
    SimTK::Vector udot = model.getMatterSubsystem().getUDot(s);

    // Solve inverse dynamics given current states and udot
    // The output vector contains the generalised coordinate forces
    // to generate the accelerations based on the current state.
    // Note that these aren't necessarily in the order of the
    // coordinate set, but rather the multibody tree order.
    SimTK::Vector genForceTraj = ivdSolver.solve(s, udot);

    ///* TODO: mapping if q != u in index(i.e.tree vs.model)... */

    // Calculate the equivalent body force at the free joint in the model
    SimTK::SpatialVec equivalentBodyForceAtJoint = freeJoint.calcEquivalentSpatialForce(s, genForceTraj);

    // Extract the body and torque components
    SimTK::Vec3 freeBodyTorque = equivalentBodyForceAtJoint.get(0);
    SimTK::Vec3 freeBodyForce = equivalentBodyForceAtJoint.get(1);

    // First determine whether generic ground contact is occurring
    // based on force threshold
    if (freeBodyForce.get(1) > forceThreshold) {
        
        // Get the position of the free body in the ground frame
        // This is based on getting the child frame of the free joint
        // and hence assumes that the ground should always be the parent.
        // I think getting the frames position in the ground should
        // represent it's translational coordinates, as it appears this
        // is robust to any frame translation in the model. Similarly,
        // using the frames origin (i.e. 0,0,0) as the station seems to
        // also be appropriate for simply getting the translational values.
        SimTK::Vec3 rp = freeJoint.getChildFrame().findStationLocationInGround(
                s, SimTK::Vec3(0, 0, 0));

        // Take the cross product of free body position and force vector to get
        // moment at origin in ground
        SimTK::Vec3 groundM = SimTK::Vec3((rp.get(1) * freeBodyForce.get(2)) - (rp.get(2) * freeBodyForce.get(1)),
            -((rp.get(0) * freeBodyForce.get(2)) - (rp.get(2) * freeBodyForce.get(0))),
            (rp.get(0) * freeBodyForce.get(1)) - (rp.get(1) * freeBodyForce.get(0)));

        // Calculate X & Z cZMP, noting that yZMP is set as 0
        // Given this (and some other calculations) the standard OpenSim
        // coordinate system must be used.
        // Formulas used here come from Xiang et al. 2009:
        // https://doi.org/10.1002/nme.2575
        SimTK::Vec3 zmpCOP = SimTK::Vec3(groundM.get(2) / freeBodyForce.get(1),
            0,
            -groundM.get(0) / freeBodyForce.get(1));

        // Calculate the resultant active moment at ZMP along the y-axis
        // TODO : this still seems wrong --- off by factor of 10?
        // Check Xiang et al.
        double myZMP = groundM.get(1) + (freeBodyForce.get(0) * zmpCOP.get(2)) -
                       (freeBodyForce.get(2) * zmpCOP.get(0));

        // Check the contact bodies for contact with the ground
        /*for (int i = 0; i < 5; i++) { cout << i << "\n"; }*/

        //const int nCB = getProperty_contact_body().size();

        //getProperty_contact_body().get(0)




    } // else allocate zeros in vector...

}

// Calculating ground reactions when udot is provided alongside the accelerations
// TODO: currently no checks related to udot matching model

SimTK::Vector ZeroMomentPointGroundReactions::calcGroundReactions(
        const SimTK::State& s, const SimTK::Vector& udot, bool unilateralContact) const {

    // TODO: determine whether contact bodies are in contact with ground...
    //
    // Probably need marker distance AND vertical velocity
    // (i.e. during walking trial markers cna be penetrating ground while
    // moving...)
    //
    // Subsequent calculation steps are fairly dependent on this...
    // Xiang et al. denotes contact points on the foot that seemingly can't
    // penetrate the ground (e.g. some sort of constraint on these?)
    // When the vertical height (y-axis) of contact points = 0, ground contact
    // is present The ZMP is constrained to remain within these boundaries ---
    // but that doesn't seem feasible in a basic calculation...maybe in a
    // dynamic simulation though...

    /*This function calculates the Zero Moment Point of the model based on
    the Model state. It identifies the ground reaction forces, moments
    and centre of pressure for each contact body connected to the component.
    The output is returned as a Vector which size is based on the number of
    contact bodies and the separate force, moment and point components, i.e.:

    FXn, FYn, FZn, MXn, MYn, MZn, PXn, PYn, PZn

    where n is repeated for the number of contact bodies specified.*/

    // TODO:
    //    > Am I Getting variables appropriately?

    // Get the model
    const auto& model = getModel();

    // Get the free body name and associated joint
    const std::string freeJointName = get_free_joint_name();
    const Joint& freeJoint = model.getJointSet().get(freeJointName);

    // Get the number of contact bodies
    const int nCB = getProperty_zmp_contact_bodies().size();

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

    // Solve inverse dynamics given current states and udot
    // The output vector contains the generalised coordinate forces
    // to generate the accelerations based on the current state.
    // Note that these aren't necessarily in the order of the
    // coordinate set, but rather the multibody tree order.
    SimTK::Vector genForceTraj = ivdSolver.solve(s, udot);
    /*std::cout << "Printing ID forces";
    std::cout << genForceTraj;*/

    ///* TODO: mapping if q != u in index(i.e.tree vs.model)... */

    // Calculate the equivalent body force at the free joint in the model
    SimTK::SpatialVec equivalentBodyForceAtJoint =
            freeJoint.calcEquivalentSpatialForce(s, genForceTraj);

    // Extract the body and torque components
    SimTK::Vec3 freeBodyTorque = equivalentBodyForceAtJoint.get(0);
    SimTK::Vec3 freeBodyForce = equivalentBodyForceAtJoint.get(1);

    /*std::cout << freeBodyForce;
    std::cout << freeBodyTorque;*/

    // First determine whether generic ground contact is occurring
    // based on force threshold
    if (freeBodyForce.get(1) > forceThreshold) {

        // Get the position of the free body in the ground frame
        // This is based on getting the child frame of the free joint
        // and hence assumes that the ground should always be the parent.
        // I think getting the frames position in the ground should
        // represent it's translational coordinates, as it appears this
        // is robust to any frame translation in the model. Similarly,
        // using the frames origin (i.e. 0,0,0) as the station seems to
        // also be appropriate for simply getting the translational values.
        
		///* TODO: Would body centre of mass be the better station to get? */
		
		SimTK::Vec3 rp = freeJoint.getChildFrame().findStationLocationInGround(
                s, SimTK::Vec3(0, 0, 0));

        // Take the cross product of free body position and force vector to get
        // moment at origin in ground
        SimTK::Vec3 groundM =
                SimTK::Vec3((rp.get(1) * freeBodyForce.get(2)) -
                                    (rp.get(2) * freeBodyForce.get(1)),
                        -((rp.get(0) * freeBodyForce.get(2)) -
                                (rp.get(2) * freeBodyForce.get(0))),
                        (rp.get(0) * freeBodyForce.get(1)) -
                                (rp.get(1) * freeBodyForce.get(0)));

        // Calculate X & Z cZMP, noting that yZMP is set as 0
        // Given this (and some other calculations) the standard OpenSim
        // coordinate system must be used.
        // Formulas used here come from Xiang et al. 2009:
        // https://doi.org/10.1002/nme.2575
        SimTK::Vec3 zmpCOP = SimTK::Vec3(groundM.get(2) / freeBodyForce.get(1),
                0, -groundM.get(0) / freeBodyForce.get(1));

        // Calculate the resultant active moment at ZMP along the y-axis
        // TODO : this still seems wrong --- off by factor of 10?
        // Check Xiang et al.
        double myZMP = ( groundM.get(1) + (freeBodyForce.get(0) * zmpCOP.get(2)) -
                       (freeBodyForce.get(2) * zmpCOP.get(0)) ) / 1000;

        // Check the contact bodies for contact with the ground
        /*for (int i = 0; i < 5; i++) { cout << i << "\n"; }*/

        // Check for unilateral contact flag
        if (unilateralContact) {

            // If unilateral contact is flagged, then the force is simply
            // allocated to the body that is closer to the predicted COP

            // Another option is to use whichever one is closer to the ground

            // NOTE: this is currently assuming there are always two bodies!

            // Get the first body and calculate distance to COP

            // Get the body position
            std::string body1_name = get_zmp_contact_bodies(0).get_body_name();
            SimTK::Vec3 body1Pos = model.getBodySet().get(body1_name).findStationLocationInGround(s, SimTK::Vec3(0, 0, 0));

            //// Calculate distances from body to predicted ZMP
            //double xSqr_1 = (body1Pos.get(0) - zmpCOP.get(0)) * (body1Pos.get(0) - zmpCOP.get(0));
            //double ySqr_1 = (body1Pos.get(1) - zmpCOP.get(1)) * (body1Pos.get(1) - zmpCOP.get(1));
            //double zSqr_1 = (body1Pos.get(2) - zmpCOP.get(2)) * (body1Pos.get(2) - zmpCOP.get(2));
            //double body1Dist = sqrt(xSqr_1 + ySqr_1 + zSqr_1);
            double body1Level = body1Pos.get(1);

            // Get the second body and calculate distance to COP

            // Get the body position
            std::string body2_name = get_zmp_contact_bodies(1).get_body_name();
            SimTK::Vec3 body2Pos = model.getBodySet().get(body2_name).findStationLocationInGround(s, SimTK::Vec3(0, 0, 0));

            //// Calculate distances from body to predicted ZMP
            //double xSqr_2 = (body2Pos.get(0) - zmpCOP.get(0)) * (body2Pos.get(0) - zmpCOP.get(0));
            //double ySqr_2 = (body2Pos.get(1) - zmpCOP.get(1)) * (body2Pos.get(1) - zmpCOP.get(1));
            //double zSqr_2 = (body2Pos.get(2) - zmpCOP.get(2)) * (body2Pos.get(2) - zmpCOP.get(2));
            //double body2Dist = sqrt(xSqr_2 + ySqr_2 + zSqr_2);
            double body2Level = body2Pos.get(1);

            // Allocate the forces to appropriate body in vector
            // TODO: this needs to be cleaned up --- i.e. auto column values
            //if (body1Dist < body2Dist) {
            if (body1Level < body2Level) {

                // Forces
                groundReactionsVec.set(0, freeBodyForce.get(0));
                groundReactionsVec.set(1, freeBodyForce.get(1));
                groundReactionsVec.set(2, freeBodyForce.get(2));
                // Torques (note zero moments for X and Y)
                groundReactionsVec.set(12, 0.0);
                groundReactionsVec.set(13, myZMP);
                groundReactionsVec.set(14, 0.0);
                // COP
                groundReactionsVec.set(3, zmpCOP.get(0));
                groundReactionsVec.set(4, zmpCOP.get(1));
                groundReactionsVec.set(5, zmpCOP.get(2));

            //} else if (body2Dist < body1Dist) {
            } else if (body2Level < body1Level) {

                // Forces
                groundReactionsVec.set(6, freeBodyForce.get(0));
                groundReactionsVec.set(7, freeBodyForce.get(1));
                groundReactionsVec.set(8, freeBodyForce.get(2));
                // Torques (note zero moments for X and Y)
                groundReactionsVec.set(15, 0.0);
                groundReactionsVec.set(16, myZMP);
                groundReactionsVec.set(17, 0.0);
                // COP
                groundReactionsVec.set(9, zmpCOP.get(0));
                groundReactionsVec.set(10, zmpCOP.get(1));
                groundReactionsVec.set(11, zmpCOP.get(2));
            
            }

        } // else --- TODO: check for which body is contacting...

    }

    return groundReactionsVec;

}

/* -------------------------------------------------------------------------- *
 * ZeroMomentPointContactBody                                                 *
 * -------------------------------------------------------------------------- *
 * Add instructional content related to the ZeroMomentPointContactBody        *
 * class...                                                                   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

//=============================================================================
//  METHODS: ZeroMomentPointContactBody
//=============================================================================

ZeroMomentPointContactBody::ZeroMomentPointContactBody() {
    constructProperties();
}

void ZeroMomentPointContactBody::constructProperties() {

    constructProperty_body_name("NONE");
    constructProperty_zmp_contact_body_points();

}

void ZeroMomentPointContactBody::addContactBodyPoint(
    const std::string& point_name,
    const std::string& body_name,
    const SimTK::Vec3& point_location) {

    // Append provided value to list of contact body points
    append_zmp_contact_body_points(ZeroMomentPointContactBodyPoint());

    // Get updated parameters for the contact body point
    auto& cbp = upd_zmp_contact_body_points(
            getProperty_zmp_contact_body_points().size() - 1);

    // Set the name as specified
    cbp.setName(point_name);

    // Set the body name
    cbp.set_body_name(body_name);

    // Set the point location
    cbp.set_location(point_location);

}


/* -------------------------------------------------------------------------- *
 * ZeroMomentPointContactBodyPoint                                            *
 * -------------------------------------------------------------------------- *
 * Add instructional content related to the ZeroMomentPointContactBodyPoint   *
 * class...                                                                   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * -------------------------------------------------------------------------- */

//=============================================================================
//  METHODS: ZeroMomentPointContactBody
//=============================================================================

ZeroMomentPointContactBodyPoint::ZeroMomentPointContactBodyPoint() {
    constructProperties();
}

void ZeroMomentPointContactBodyPoint::constructProperties() {

    constructProperty_body_name("NONE");
    constructProperty_location(SimTK::Vec3(0,0,0));

}