/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointContactBody.cpp                                    *
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

// TODO: any other inclusions?

#include "ZeroMomentPointContactBody.h"
#include "ZeroMomentPointContactPointSet.h"
#include "ZeroMomentPointContactPoint.h"

// #include <SimTKcommon/internal/State.h>
// #include <OpenSim/Common/Component.h>
// #include "OpenSim/Common/IO.h"
// #include <OpenSim/Simulation/Model/Model.h>
// #include <OpenSim/Simulation/InverseDynamicsSolver.h>

using namespace OpenSim;

//=============================================================================
//=============================================================================
/**
 * The ZeroMomentPointContactBody is used to specify the expected bodies in
 * the model that will come into contact with the ground, and therefore are
 * checked for ground contact during a simulated motion. The object sits in
 * a ZeroMomentPointContactBodySet within the over-arching
 * ZeroMomentPointGroundReactions model component. 
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ZeroMomentPointContactBody::ZeroMomentPointContactBody() {
    
    // Construct with default properties
    constructProperties();

}

/** Construct with object name */
ZeroMomentPointContactBody::ZeroMomentPointContactBody(
        const std::string& name) {

    // Construct with default properties
    constructProperties();

    // Set the object name
    setName(name);

}

/** Construct with object name, body name and contact checking method */
ZeroMomentPointContactBody::ZeroMomentPointContactBody(
        const std::string& name,     
        const std::string& bodyName, 
        const std::string& contactCheckingMethod) {

    // Construct with default properties
    constructProperties();

    // Set the object name
    setName(name);

    // Set the body name
    set_body_name(bodyName);

    // TODO: checks in place for valid strings for contact checking

    // Set the contact checking method
    set_zmp_contact_checking_method(contactCheckingMethod);

}

/** Construct default properties */
void ZeroMomentPointContactBody::constructProperties() {

    // Standard properties
    constructProperty_body_name("unassigned");
    constructProperty_zmp_contact_checking_method("velocity");

    // TODO: some potential issues with component set naming?

    // Contact point set
    ZeroMomentPointContactPointSet contactPointSet;
    contactPointSet.setName(
                IO::Lowercase(contactPointSet.getConcreteClassName()));
    constructProperty_ZeroMomentPointContactPointSet(contactPointSet);

}

//=============================================================================
//  METHODS
//=============================================================================

// TODO: add contact point methods need to be updated to reflect body socket use...

/** Add a contact point for reviewing ground contact with default
distance and velocity parameters */
void ZeroMomentPointContactBody::addContactPoint(
    const std::string& pointName,
    const PhysicalFrame& body,
    const SimTK::Vec3& pointLocation) {

    // Create the contact point with the specified properties
    ZeroMomentPointContactPoint* cp =
            new ZeroMomentPointContactPoint(pointName, body, pointLocation);

    // Append to the contact point set
    // upd_ZeroMomentPointContactPointSet().cloneAndAppend(cp);
    updZeroMomentPointContactPointSet().adoptAndAppend(cp);

    // TODO: needed somewhere?
    //// Finalize properties
    //finalizeFromProperties();
    //prependComponentPathToConnecteePath(cp);

}

/** Add a contact point for reviewing ground contact with specified
distance and default velocity parameter */
void ZeroMomentPointContactBody::addContactPoint(
        const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& pointLocation,
        const double& distanceThreshold) {

    // Create the contact point with the specified properties
    /*ZeroMomentPointContactPoint& cp =
            ZeroMomentPointContactPoint(pointName, bodyName, pointLocation);*/
    ZeroMomentPointContactPoint* cp =
            new ZeroMomentPointContactPoint(pointName, body, pointLocation);

    // Set the distance threshold
    cp->set_distance_threshold(distanceThreshold);

    // Append to the contact point set
    //upd_ZeroMomentPointContactPointSet().cloneAndAppend(cp);
    updZeroMomentPointContactPointSet().adoptAndAppend(cp);

    // TODO: needed somewhere?
    //// Finalize properties
    //finalizeFromProperties();
    //prependComponentPathToConnecteePath(cp);

}

/** Add a contact point for reviewing ground contact with specified
distance and velocity parameters */
void ZeroMomentPointContactBody::addContactPoint(
        const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& pointLocation,
        const double& distanceThreshold,
        const double& velocityThreshold) {

    // Create the contact point with the specified properties
    /*ZeroMomentPointContactPoint& cp =
            ZeroMomentPointContactPoint(pointName, bodyName, pointLocation);*/
    ZeroMomentPointContactPoint* cp =
            new ZeroMomentPointContactPoint(pointName, body, pointLocation);

    // Set the distance threshold
    cp->set_distance_threshold(distanceThreshold);

    // Set the velocity threshold
    cp->set_velocity_threshold(velocityThreshold);

    // Append to the contact point set
    //upd_ZeroMomentPointContactPointSet().cloneAndAppend(cp);
    updZeroMomentPointContactPointSet().adoptAndAppend(cp);

    // TODO: needed somewhere?
    //// Finalize properties
    //finalizeFromProperties();
    //prependComponentPathToConnecteePath(cp);

}

/** Setting contact body name */
void ZeroMomentPointContactBody::setBodyName(
    const std::string& bodyName) {

    // Set the body name
    set_body_name(bodyName);

}

/** Setting contact checking method */
void ZeroMomentPointContactBody::setContactCheckingMethod(
    const std::string& contactCheckingMethod) {

    // TODO: checks in place for valid strings for contact checking

    // Set the contact checking method
    set_zmp_contact_checking_method(contactCheckingMethod);

}