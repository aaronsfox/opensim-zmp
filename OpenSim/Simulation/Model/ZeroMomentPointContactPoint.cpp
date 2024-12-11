/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointContactPoint.cpp                                *
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

#include "ZeroMomentPointContactPoint.h"

using namespace OpenSim;
//=============================================================================
//=============================================================================
/**
 * The ZeroMomentPointContactPoint is used to specify points on the contact body
 * that are used to check for ground contact, using an approach similat to 
 * Karcnik (2003) (see https://doi.org/10.1007/BF02345310). The contact points
 * sit within a ZeroMomentPointContactPointSet within the ZeroMomentPointContactBody.
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ZeroMomentPointContactPoint::ZeroMomentPointContactPoint() {
    
    // Construct with default properties
    constructProperties();    

}

/** Construct specifying point name, body frame and location */
ZeroMomentPointContactPoint::ZeroMomentPointContactPoint(
        const std::string& pointName, const PhysicalFrame& body,
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
ZeroMomentPointContactPoint::ZeroMomentPointContactPoint(
        const std::string& pointName, const PhysicalFrame& body,
        const SimTK::Vec3& location,
        const double& distanceThreshold,
        const double& velocityThreshold) {

    // Construct default properties
    constructProperties();

    // Set point name
    setName(pointName);

    // Set with specified location
    set_location(location);

    // Connect to socket physical frame via body name
    connectSocket_parent_frame(body);

    // Set distance threshold
    set_distance_threshold(distanceThreshold);

    // Set velocity threshold
    set_velocity_threshold(velocityThreshold);

}

/** Construct default properties */
void ZeroMomentPointContactPoint::constructProperties() {

    setName("contact_point");
    constructProperty_distance_threshold(0.08);
    constructProperty_velocity_threshold(1.5);

}

//=============================================================================
//  METHODS
//=============================================================================

/** Set the distance threshold. */
void ZeroMomentPointContactPoint::setDistanceThreshold(
        const double& distanceThreshold) {
    set_distance_threshold(distanceThreshold);
}

/** Set the velocitythreshold. */
void ZeroMomentPointContactPoint::setVelocityThreshold(
        const double& velocityThreshold) {
    set_velocity_threshold(velocityThreshold);
}