#ifndef OPENSIM_ZEROMOMENTPOINTCONTACTPOINT_H
#define OPENSIM_ZEROMOMENTPOINTCONTACTPOINT_H

/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointContactPoint.h                                     *
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

#include <OpenSim/Simulation/Model/Station.h>

namespace OpenSim {
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
class OSIMSIMULATION_API ZeroMomentPointContactPoint: public Station {
    OpenSim_DECLARE_CONCRETE_OBJECT(ZeroMomentPointContactPoint, Station);

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

	//=========================================================================
    // CONSTRUCTORS
    //=========================================================================

	/** Default constructor */
    ZeroMomentPointContactPoint();

    /** Construct specifying point name, body frame and location */
    ZeroMomentPointContactPoint(
            const std::string& pointName,
            const PhysicalFrame& body,
            const SimTK::Vec3& location);

    /** Construct specifying point name, body frame, location and thresholds */
    ZeroMomentPointContactPoint(
            const std::string& pointName,
            const PhysicalFrame& body,
            const SimTK::Vec3& location,
            const double& distanceThreshold,
            const double& velocityThreshold);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Get and set the distance threshold. */
    const double& getDistanceThreshold() const { return get_distance_threshold(); }
    void setDistanceThreshold(const double& distanceThreshold);

    /** Get and set the velocity threshold. */
    const double& getVelocityThreshold() const { return get_velocity_threshold(); }
    void setVelocityThreshold(const double& velocityThreshold);

    // TODO: methods to get and edit points?

private:

    void constructProperties();

};

} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINTCONTACTPOINT_H
