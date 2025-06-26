#ifndef OPENSIM_CONTACTPOINT_H
#define OPENSIM_CONTACTPOINT_H

/* -------------------------------------------------------------------------- *
 * OpenSim: ContactPoint.h                                                    *
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

#include "Station.h"
#include "PhysicalFrame.h"

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * A contact point is used to specify points on a body that can be queried for 
 * contact against another frame dependent on the use case. The contact points
 * will typically sit within a ContactPointSet.
 *
 * @authors Aaron Fox
 * @version 1.0
 */
class OSIMSIMULATION_API ContactPoint: public Station {
    OpenSim_DECLARE_CONCRETE_OBJECT(ContactPoint, Station);

public:
    
	//=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(distance_threshold, double,
            "The vertical distance threshold to check whether the contact point has "
            "come into contact with the ground (in meters) (defaults to 0.08).");

    OpenSim_DECLARE_PROPERTY(velocity_threshold, double,
            "The velocity threshold to check whether the contact point has "
            "come into contact with the ground (in meters per second) (defaults "
            "to 1.5).");

    OpenSim_DECLARE_PROPERTY(contact_checking_method, std::string,
            "The ground contact checking method applied to this contact point. "
            "Options here are strings of distance (only distance to ground is "
            "checked) and velocity (distance and velocity are checked) (defaults "
            "to distance).");

	//=========================================================================
    // CONSTRUCTORS
    //=========================================================================

	/** Default constructor */
    ContactPoint();

    /** Construct specifying point name, body frame and location */
    ContactPoint(
            const std::string& pointName,
            const PhysicalFrame& body,
            const SimTK::Vec3& location);

    /** Construct specifying point name, body frame, location and thresholds */
    ContactPoint(
            const std::string& pointName,
            const PhysicalFrame& body,
            const SimTK::Vec3& location,
            const double& distanceThreshold,
            const double& velocityThreshold);

    /** Construct specifying point name, body frame, location, thresholds 
    and contact checking method. */
    ContactPoint(
        const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& location,
        const double& distanceThreshold,
        const double& velocityThreshold,
        const std::string& contactCheckingMethod);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Get and set the distance threshold. */
    const double& getDistanceThreshold() const { 
        return get_distance_threshold(); }
    void setDistanceThreshold(const double& distanceThreshold);

    /** Get and set the velocity threshold. */
    const double& getVelocityThreshold() const { 
        return get_velocity_threshold(); }
    void setVelocityThreshold(const double& velocityThreshold);

    /** Get and set the contact checking method. */
    const std::string& getContactCheckingMethod() const { 
        return get_contact_checking_method(); }
    void setContactCheckingMethod(const std::string& contactCheckingMethod);

    /** Convenience method to get the 'parent_frame' Socket's connectee_name */
    const std::string& getParentFrameName() const;

private:

    void constructProperties();

};

} // namespace OpenSim

#endif // OPENSIM_CONTACTPOINT_H