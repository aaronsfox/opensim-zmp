/* -------------------------------------------------------------------------- *
 * OpenSim: ContactPoint.cpp                                                  *
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

#include "ContactPoint.h"

using namespace OpenSim;

//=============================================================================
//=============================================================================
/**
 * A contact point is used to specify points on a body that can be queried for
 * contact against another frame dependent on the use case. The contact points
 * sit within a ContactPointSet which is a ModelComponentSet.
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ContactPoint::ContactPoint() {
    
    // Construct with default properties
    constructProperties();    

}

/** Construct specifying point name, body frame and location */
ContactPoint::ContactPoint(
        const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& location) {

    // Construct default properties
    constructProperties();
    
    // Set point name
    setName(pointName);

    // Set with specified location
    set_location(location);

    // Connect to socket physical frame via body name
    connectSocket_parent_frame(body);

}

/** Construct specifying point name, body frame, location and thresholds */
ContactPoint::ContactPoint(
        const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& location,
        const double& distanceThreshold,
        const double& velocityThreshold) {

    // Construct default properties
    constructProperties();

    // Set point name
    setName(pointName);

    // Set with specified location
    set_location(location);

    // Set distance threshold
    set_distance_threshold(distanceThreshold);

    // Set velocity threshold
    set_velocity_threshold(velocityThreshold);

    // Connect to socket physical frame via body name
    connectSocket_parent_frame(body);

}

/** Construct specifying point name, body frame, location, thresholds
and contact checking method. */
ContactPoint::ContactPoint(
    const std::string& pointName,
    const PhysicalFrame& body,
    const SimTK::Vec3& location,
    const double& distanceThreshold,
    const double& velocityThreshold,
    const std::string& contactCheckingMethod) {

    // Construct default properties
    constructProperties();

    // Set point name
    setName(pointName);

    // Set with specified location
    set_location(location);

    // Set distance threshold
    set_distance_threshold(distanceThreshold);

    // Set velocity threshold
    set_velocity_threshold(velocityThreshold);

    // Set contact checking method
    // TODO: raise error here if not an appropriate option?
    set_contact_checking_method(contactCheckingMethod);

    // Connect to socket physical frame via body name
    connectSocket_parent_frame(body);
}

/** Construct default properties */
void ContactPoint::constructProperties() {

    setName("contact_point");
    constructProperty_distance_threshold(0.08);
    constructProperty_velocity_threshold(1.5);
    constructProperty_contact_checking_method("distance");

}

//=============================================================================
//  METHODS
//=============================================================================

/** Set the distance threshold. */
void ContactPoint::setDistanceThreshold(
        const double& distanceThreshold) {
    set_distance_threshold(distanceThreshold);
}

/** Set the velocity threshold. */
void ContactPoint::setVelocityThreshold(
        const double& velocityThreshold) {
    set_velocity_threshold(velocityThreshold);
}

/** Set the contact checking method. */
void ContactPoint::setContactCheckingMethod(
    const std::string& contactCheckingMethod) {
    set_contact_checking_method(contactCheckingMethod);
}

/** Convenience method to get the 'parent_frame' Socket's connectee_name */
const std::string& ContactPoint::getParentFrameName() const { 
    return getSocket<PhysicalFrame>("parent_frame").getConnecteePath(); }