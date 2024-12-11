#ifndef OPENSIM_ZEROMOMENTPOINTCONTACTBODY_H
#define OPENSIM_ZEROMOMENTPOINTCONTACTBODY_H

/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointContactBody.h                                      *
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

// TODO: what to include?

// #include <OpenSim/Moco/osimMocoDLL.h>
// #include <unordered_map>
// #include "OpenSim/Moco/MocoTrajectory.h"
// #include <OpenSim/Common/Object.h>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointContactPoint.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointContactPointSet.h>

namespace OpenSim {
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

class OSIMSIMULATION_API ZeroMomentPointContactBody : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ZeroMomentPointContactBody, ModelComponent);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================

    // TODO: body should be a socket...

    OpenSim_DECLARE_PROPERTY(body_name, std::string,
            "The name of the contact body to which the "
            "ZeroMomentPointGroundReactions "
            " is connected to. Each body is checked for gound contact when "
            "calculating "
            "ZMP ground reactions.");

    OpenSim_DECLARE_PROPERTY(zmp_contact_checking_method, std::string,
            "The ground contact checking method applied to contact points "
            "on this body. Options here are strings of distance (only distance "
            "to ground is checked) and velocity (distance and velocity checked) "
            "(defaults to velocity).");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ZeroMomentPointContactPointSet,
            "The series of points (as locations on bodies) associated with the "
            "contact body "
            "which will be queried to determine whether the points are in "
            "contact with the "
            "ground.");

    //=========================================================================
    // OUTPUTS
    //=========================================================================

    // TODO: consider option to get individual contact body outputs to help out
    // with the ContactForce application?

	//=========================================================================
    // CONSTRUCTORS
    //=========================================================================
	
	/** Default constructor */
    ZeroMomentPointContactBody();

    /** Construct with object name */
    ZeroMomentPointContactBody(const std::string& name);

    /** Construct with object name, body name and contact checking method */
    ZeroMomentPointContactBody(
        const std::string& name, 
        const std::string& bodyName,
        const std::string& contactCheckingMethod);
	
    //=========================================================================
    // METHODS
    //=========================================================================

    /** Method for returning contact point set */
    ZeroMomentPointContactPointSet& updZeroMomentPointContactPointSet() {
        return upd_ZeroMomentPointContactPointSet();
    }

    /** Add a contact point for reviewing ground contact with default
    distance and velocity parameters */
    void addContactPoint(const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& pointLocation);

    /** Add a contact point for reviewing ground contact with specified
    distance and default velocity parameter */
    void addContactPoint(const std::string& pointName,
        const PhysicalFrame& body,
        const SimTK::Vec3& pointLocation,
        const double& distanceThreshold);

    /** Add a contact point for reviewing ground contact with specified
    distance and velocity parameters */
    void addContactPoint(const std::string& pointName,
        const std::string& bodyName,
        const SimTK::Vec3& pointLocation,
        const double& distanceThreshold,
        const double& velocityThreshold);

    /** Getting and setting contact body name */
    const std::string& getBodyName() const {
        return get_body_name();
    }
    void setBodyName(const std::string& bodyName);

    /** Getting and setting contact checking method */
    const std::string& getContactCheckingMethod() const { 
        return get_zmp_contact_checking_method(); 
    }
    void setContactCheckingMethod(const std::string& contactCheckingMethod);

    //=========================================================================
    // OUTPUTS
    //=========================================================================

    ///** Get the ground reaction forces applied on the body. */
    //SimTK::Vec3 getReactionForcesOnBody(const SimTK::State& s);

    ///** Get the ground reaction moments applied on the body. */
    //SimTK::Vec3 getReactionMomentsOnBody(const SimTK::State& s);

    ///** Get the ground reactions point of application on the body. */
    //SimTK::Vec3 getReactionPointOnBody(const SimTK::State& s);

private:

    void constructProperties();

};

} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINTCONTACTBODY_H
