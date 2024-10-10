/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointContactPointConstraint.cpp                         *
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


#include "ZeroMomentPointContactPointConstraint.h"
#include "simbody/internal/SimbodyMatterSubsystem.h"
#include "simbody/internal/Constraint_PointInPlane.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

// TODO:
    // > Can this just be a much simpler constraint that the distance from ground
    //   in vertical plane must be positive?
    //      >> Review how constraint is structured in Xiang et atl. for guidance

//=============================================================================
//=============================================================================
/**
 * The ZeroMomentPointContactPointConstraint is used to specify constraints on 
 * the contact body to avoid ground penetration in a simulation. The underlying
 * Constraints in Simbody are: PointInPlane to oppose penetration into the ground
 * (unilaterally). This constraint is a much more basic version of the simbody
 * RollingOnSurfaceConstraint as it removes the torque and slip constraints.
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ZeroMomentPointContactPointConstraint::ZeroMomentPointContactPointConstraint() {
    
    // Construct with default properties
    setNull();
    constructProperties();

}

/** Construct specifying point name, contact and surface body frames,
and location */
ZeroMomentPointContactPointConstraint::ZeroMomentPointContactPointConstraint(
        const std::string& pointName, 
        const PhysicalFrame& contactBody,
        const PhysicalFrame& surfaceBody,
        const SimTK::Vec3& location) {

    // Construct default properties
    setNull();
    constructProperties();
    
    // Set point name
    setName(pointName);
    
    // Connect body sockets
    connectSocket_contact_body(contactBody);
    connectSocket_surface_body(surfaceBody);

    // Set with specified location
    set_location(location);

}

/** Construct default properties */
void ZeroMomentPointContactPointConstraint::constructProperties() {

    setName("contact_point_constraint");

    // Point properties
    constructProperty_location(Vec3(0, 0, 0));

    // Surface parameters
    constructProperty_surface_normal(Vec3(0, 1.0, 0));
    constructProperty_surface_height(0.0);

}

/** Set the data members of this constraint to their null values.*/
void ZeroMomentPointContactPointConstraint::setNull() {

    setAuthors("Aaron Fox");
    _defaultUnilateralConditions = std::vector<bool>(1, false);
    _contactFrame.reset(nullptr);
    _surfaceFrame.reset(nullptr);

}

/** Extend ModelComponent interface */
void ZeroMomentPointContactPointConstraint::extendConnectToModel(
    Model& aModel) {

    // Base class first
    Super::extendConnectToModel(aModel);

    // Get frame connectees
    _contactFrame = updSocket<PhysicalFrame>("contact_body").getConnectee();
    _surfaceFrame = updSocket<PhysicalFrame>("surface_body").getConnectee();

}

/** Create the SimTK::Constraints: which implements this constraint. */
void ZeroMomentPointContactPointConstraint::extendAddToSystem(
        SimTK::MultibodySystem& system) const {

    // Get underlying mobilized bodies
    SimTK::MobilizedBody contact = _contactFrame->getMobilizedBody();
    SimTK::MobilizedBody surface = _surfaceFrame->getMobilizedBody();

    // Constrain the contact body to the surface
    // TODO: using location point here instead of a Vec3 of zeros --- correct?
    SimTK::Constraint::PointInPlane contactY(surface,
            SimTK::UnitVec3(get_surface_normal()), get_surface_height(),
            contact, get_location());

    // Beyond the const Component get the index so we can access the
    // SimTK::Constraint later
    ZeroMomentPointContactPointConstraint* mutableThis =
            const_cast<ZeroMomentPointContactPointConstraint*>(this);

    // Make sure that there is nothing in the list of constraint indices
    mutableThis->_indices.clear();

    // Get the index so we can access the SimTK::Constraint later
    mutableThis->_indices.push_back(contactY.getConstraintIndex());

    mutableThis->_numConstraintEquations = (int)_indices.size();

    // For compound constraints, the bodies and/or mobilities involved must be
    // characterized by the first "master" constraint, which dictates the
    // behavior of the other constraints. For example, enabling and disabling.
    // This enables a compound constraint to be treated like a single constraint
    // for the purpose of enabling/disabling and getting output
    assignConstraintIndex(_indices[0]);

}

/** Populate the SimTK::State: with defaults for the constraint. */
void ZeroMomentPointContactPointConstraint::extendInitStateFromProperties(
        SimTK::State& state) const {

    // Base class first
    Super::extendInitStateFromProperties(state);

    // All constraints treated the same as default behavior at initialization
    for (int i = 0; i < _numConstraintEquations; i++) {
        SimTK::Constraint& simConstraint =
                updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        // initialize the status of the constraint
        if (_defaultUnilateralConditions[i]) {
            simConstraint.enable(state);
        } else {
            simConstraint.disable(state);
        }
    }
}

/** Given an existing SimTK::State set defaults for the constraint. */
void ZeroMomentPointContactPointConstraint::extendSetPropertiesFromState(
        const SimTK::State& state) {

    // Base class first
    Super::extendSetPropertiesFromState(state);

    set_isEnforced(isEnforced(state));
    for (int i = 0; i < _numConstraintEquations; i++) {
        SimTK::Constraint& simConstraint =
                updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        // initialize the status of the constraint
        _defaultUnilateralConditions[i] = !simConstraint.isDisabled(state);
    }
}

//=============================================================================
//  METHODS
//=============================================================================

/** Set contact body by its name */
void ZeroMomentPointContactPointConstraint::setContactBodyByName(
        const std::string& aBodyName) {
    updSocket<PhysicalFrame>("contact_body").setConnecteePath(aBodyName);
}

/** Set surface body by its name */
void ZeroMomentPointContactPointConstraint::setSurfaceBodyByName(
        const std::string& aBodyName) {
    updSocket<PhysicalFrame>("surface_body").setConnecteePath(aBodyName);
}

/** Set contact point location */
void ZeroMomentPointContactPointConstraint::setContactPointLocation(
        const SimTK::Vec3& pointLocation) {
    set_location(pointLocation);
}

/** Get whether or not the constraint is enforced. */
bool ZeroMomentPointContactPointConstraint::isEnforced(
        const SimTK::State& state) const {

    // The parent constraint in is the plane constraint, so check its value
    return !updSystem()
                    .updMatterSubsystem()
                    .updConstraint(_indices[0])
                    .isDisabled(state);
}

/** Set whether or not the constraint is enforced.*/
bool ZeroMomentPointContactPointConstraint::setIsEnforced(
        SimTK::State& state, bool isEnforced) {

    // All constraints treated the same as default behavior i.e. at
    // initialization
    std::vector<bool> shouldBeOn(_numConstraintEquations, isEnforced);

    // If dynamics has been realized, then this is an attempt to enforce/disable
    //  the constraint during a computation and not an initialization, in which
    // case we must check the unilateral conditions for each constraint
    if (state.getSystemStage() > Stage::Dynamics)
        shouldBeOn = unilateralConditionsSatisfied(state);

    return setIsEnforced(state, isEnforced, shouldBeOn);
}

// This method allows finer granularity over the subconstraints according
// to imposed behavior
bool ZeroMomentPointContactPointConstraint::setIsEnforced(
        SimTK::State& state, bool isEnforced, std::vector<bool> shouldBeOn) {
    
    for (int i = 0; i < _numConstraintEquations; i++) {
        SimTK::Constraint& simConstraint =
                updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        bool isConstraintOn = !simConstraint.isDisabled(state);

        // Check if we already have the correct enabling of the constraint then
        // do nothing
        if (shouldBeOn[i] == isConstraintOn) continue;

        // Otherwise we have to change the status of the constraint
        if (shouldBeOn[i]) {
            simConstraint.enable(state);
        } else {
            simConstraint.disable(state);
        }
    }

    // Update the property accordingly
    set_isEnforced(isEnforced);

    // Return whether or not constraint is in the state the caller wanted
    // The first constraint is the "master" so its state is what we care about
    return isEnforced != updSystem()
                                 .updMatterSubsystem()
                                 .updConstraint(_indices[0])
                                 .isDisabled(state);
}

/** Test whether unilateral conditions are being satisfied.
Note: system must be realized to at least Stage::Dynamics */
std::vector<bool> ZeroMomentPointContactPointConstraint::unilateralConditionsSatisfied(
        const SimTK::State& state) {

    std::vector<bool> conditionsSatisfied(1, false);
    int mp, mv, ma;
    SimTK::Vector lambda;

    // The reaction forces necessary for resolving the unilateral conditions
    double normalForce = 0;

    // Get the individual underlying constraints
    SimTK::Constraint& contactY =
            updSystem().updMatterSubsystem().updConstraint(_indices[0]);

    // Constraint conditions only matter if the constraint is enabled.
    if (!contactY.isDisabled(state)) {

        // Obtain the surface constraint force in the normal direction.
        contactY.getNumConstraintEquationsInUse(state, mp, mv, ma);
        lambda = contactY.getMultipliersAsVector(state);
        normalForce = -lambda[0];

    }

    // All internal constraints depend on the normal force: no force no
    // constraint That is what makes this constraint unilateral
    if (normalForce > 0.0) {

        conditionsSatisfied[0] = true;

    }

    // Cache the conditions until the next reevaluation
    _defaultUnilateralConditions = conditionsSatisfied;

    return conditionsSatisfied;
}

/** Set whether constraint is enforced but use cached values
for unilateral conditions instead of automatic reevaluation */
bool ZeroMomentPointContactPointConstraint::
        setIsEnforcedWithCachedUnilateralConditions(
        bool isEnforced, SimTK::State& state) {

    return setIsEnforced(state, isEnforced, _defaultUnilateralConditions);
}

//=========================================================================
// FORCES
//=========================================================================

/** Calculate constraint forces.*/
void ZeroMomentPointContactPointConstraint::calcConstraintForces(
        const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor,
        SimTK::Vector& mobilityForces) const {

    SimTK::Vector_<SimTK::SpatialVec> bfs;
    SimTK::Vector mfs;

    for (int i = 0; i < _numConstraintEquations; i++) {

        SimTK::Constraint& simConstraint =
                updSystem().updMatterSubsystem().updConstraint(_indices[i]);
        int ncb = simConstraint.getNumConstrainedBodies();
        int mp, mv, ma;

        if (!simConstraint.isDisabled(state)) {
            simConstraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
            SimTK::Vector multipliers =
                    simConstraint.getMultipliersAsVector(state);
            simConstraint.calcConstraintForcesFromMultipliers(
                    state, multipliers, bfs, mfs);

            int sbi = -1;
            int rbi = -1;

            for (int j = 0; j < ncb; j++) {
                if (_surfaceFrame->getMobilizedBodyIndex() ==
                        simConstraint
                                .getMobilizedBodyFromConstrainedBody(
                                        ConstrainedBodyIndex(j))
                                .getMobilizedBodyIndex())
                    sbi = j;
                if (_contactFrame->getMobilizedBodyIndex() ==
                        simConstraint
                                .getMobilizedBodyFromConstrainedBody(
                                        ConstrainedBodyIndex(j))
                                .getMobilizedBodyIndex())
                    rbi = j;
            }

            /*
            cout << "Constraint " << i << "  forces:" << endl;
            cout << " Surf body index: " << sbi << " Expressed in: " << anc <<
            endl; cout << " Roll body index: " << rbi << " Expressed in: " <<
            anc << endl; bfs.dump(" Constraint body forces:");
            */
            bodyForcesInAncestor[0] += bfs[sbi];
            bodyForcesInAncestor[1] += bfs[rbi];
            mobilityForces += mfs;

        }
    }
}