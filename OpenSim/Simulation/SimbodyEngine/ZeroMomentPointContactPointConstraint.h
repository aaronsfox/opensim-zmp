#ifndef OPENSIM_ZEROMOMENTPOINTCONTACTPOINTCONSTRAINT_H
#define OPENSIM_ZEROMOMENTPOINTCONTACTPOINTCONSTRAINT_H

/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointContactPointConstraint.h                           *
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

#include "UnilateralConstraint.h"
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * The ZeroMomentPointContactPointConstraint is used to specify constraints on
 * the contact body to avoid ground penetration in a simulation. The underlying
 * Constraints in Simbody are: PointInPlane to oppose penetration into the
 * ground (unilaterally). This constraint is a much more basic version of the
 * simbody RollingOnSurfaceConstraint as it removes the torque and slip
 * constraints.
 *
 * @authors Aaron Fox
 * @version 1.0
 */
class OSIMSIMULATION_API ZeroMomentPointContactPointConstraint: public UnilateralConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(ZeroMomentPointContactPointConstraint,
									UnilateralConstraint);

public:
    
	//=========================================================================
    // PROPERTIES
    //=========================================================================

	OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
            "The point in the contact bodies reference system corresponding to "
            "the location on the body.");

	OpenSim_DECLARE_PROPERTY(surface_normal, SimTK::Vec3,
        "Surface normal direction in the surface body.");
		
	OpenSim_DECLARE_PROPERTY(surface_height, double,
        "Surface height in the direction of the normal in the surface body.");
			
	//=========================================================================
    // SOCKETS
    //=========================================================================

    OpenSim_DECLARE_SOCKET(contact_body, PhysicalFrame,
            "A frame fixed to the body that the location point is expressed in. "
            "Note that this can be different to the contact body that the "
            "contact point is linked to.");
			
	OpenSim_DECLARE_SOCKET(surface_body, PhysicalFrame,
            "A frame fixed to the surface body.");

	//=========================================================================
    // CONSTRUCTORS
    //=========================================================================

    // TODO: something weird going on with the connections...

	/** Default constructor */
    ZeroMomentPointContactPointConstraint();

    /** Construct specifying point name, contact and surface body names,
    and location */
    ZeroMomentPointContactPointConstraint(
            const std::string& pointName,
            const PhysicalFrame& contactBody,
            const PhysicalFrame& surfaceBody,
            const SimTK::Vec3& location);

    //=========================================================================
    // METHODS
    //=========================================================================

	/** Set contact body by its name */
    void setContactBodyByName(const std::string& aBodyName);
	
    /** Set surface body by its name */
    void setSurfaceBodyByName(const std::string& aBodyName);

    /** Set location point */
    void setContactPointLocation(const SimTK::Vec3& pointLocation);
	
	/**
    * Get whether or not the constraint is enforced.
    * Simbody multibody system instance is realized every time the isEnforced
    * changes, BUT multiple sets to the same value have no cost.
    *
    * @param state  the state of the system that includes the constraint status
    */
    bool isEnforced(const SimTK::State& state) const override;
	
	/**
    * Set whether or not the constraint is enforced.
    * Since the constraint is composed of multiple constraints, this method can
    * disable all the constraints, but enabling is not guaranteed. For example,
    * if the unilateral conditions are violated the constraint will be disabled.
    *
    * @param state      the state to set whether the constraint is enforced or 
                        not.
    * @param isEnforced if true the constraint is enforced.
    */
    bool setIsEnforced(SimTK::State& state, bool isEnforced) override;
	
	// This method allows finer granularity over the subconstraints according
    // to imposed behavior
    bool setIsEnforced(SimTK::State& state,
                       bool isEnforced,
                       std::vector<bool> shouldBeOn);

    /** Test whether unilateral conditions are being satisfied.
        Note: system must be realized to at least Stage::Dynamics */
    std::vector<bool> unilateralConditionsSatisfied(
            const SimTK::State& state) override;

    //=========================================================================
    // FORCES
    //=========================================================================
					   
	/**
    * Ask the constraint for the forces it is imposing on the 
    * system. Simbody multibody system must be realized to at least 
    * Stage::Dynamics. Returns: the bodyForces on those bodies being 
    * constrained (constrainedBodies) a SpatialVec (6 components) describing 
    * resulting torque and force mobilityForces acting along constrained 
    * mobilities
    *
    * @param state  State of model
    * @param bodyForcesInAncestor   Vector of SpatialVecs contain constraint 
    *                               forces
    * @param mobilityForces     Vector of forces that act along the constrained
    *                           mobilities associated with this constraint
    */
    void calcConstraintForces(const SimTK::State& state,
                       SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor,
                       SimTK::Vector& mobilityForces) const override;
					   
	/** Set whether constraint is enforced but use cached values 
    for unilateral conditions instead of automatic reevaluation */
    bool setIsEnforcedWithCachedUnilateralConditions(bool isEnforced,
                                                     SimTK::State& state);
													 
protected:

	/** Extend ModelComponent interface */
    void extendConnectToModel(Model& aModel) override;

    /** Create the SimTK::Constraints: which implements this constraint. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
	
    /** Populate the SimTK::State: with defaults for the constraint. */
    void extendInitStateFromProperties(SimTK::State& state) const override;
	
    /** Given an existing SimTK::State set defaults for the constraint. */
    void extendSetPropertiesFromState(const SimTK::State& state) override;

private:

    /** Set default pointer properties */
    void setNull();

	/** Construct with default properties */
    void constructProperties();
	
	/** Get the indices of underlying constraints to access from Simbody */
    SimTK::ResetOnCopy<std::vector<SimTK::ConstraintIndex>> _indices;

    /**  This cache acts a temporary hold for the constraint conditions when 
    time has not changed */
    std::vector<bool> _defaultUnilateralConditions;
	
	// References to the PhysicalFrames of the constraint
    SimTK::ReferencePtr<const PhysicalFrame> _contactFrame;
    SimTK::ReferencePtr<const PhysicalFrame> _surfaceFrame;

};

} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINTCONTACTPOINTCONSTRAINT_H
