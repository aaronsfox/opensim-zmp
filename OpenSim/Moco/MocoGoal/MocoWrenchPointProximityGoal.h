#ifndef OPENSIM_MOCOWRENCHPOINTPROXIMITYGOAL_H
#define OPENSIM_MOCOWRENCHPOINTPROXIMITYGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoWrenchPointProximityGoal.h                                    *
 * -------------------------------------------------------------------------- */

#include "MocoGoal.h"

#include <OpenSim/Actuators/BodyWrenchSmoothContactActuator.h>

// TODO: add comments

namespace OpenSim {

/**
 * Penalizes the squared per-axis distance (in the actuator's body frame)
 * between a BodyWrenchSmoothContactActuator's applied point of force
 * application and that actuator's smooth contact-point centroid, weighted
 * by the actuator's contact indicator. Unlike an equivalent hard
 * MocoPathConstraint, this requires no relaxation term, and the penalty
 * naturally vanishes as the contact indicator approaches zero, since
 * it is directly multiplied into the integrand.
 *
 *   integrand(t) = C(t) * sum_i weight_i * diff_i(t)^2
 *
 * where diff = p_body - centroid_body.
 *
 * @author Aaron Fox
 *
 @ingroup mocogoal */
class OSIMMOCO_API MocoWrenchPointProximityGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoWrenchPointProximityGoal, MocoGoal);

public:

	//=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(actuator_path, std::string,
        "The absolute component path of the BodyWrenchSmoothContactActuator "
        "this goal penalises the point of application for.");

    OpenSim_DECLARE_PROPERTY(axis_weights, SimTK::Vec3,
        "Per-axis weights (in the actuator's body frame) applied to the "
        "squared distance before summing. Default (1.0, 1.0, 1.0).");
		
	//=========================================================================
    // CONSTRUCTION
    //=========================================================================

    MocoWrenchPointProximityGoal();
    MocoWrenchPointProximityGoal(const std::string& name);
    MocoWrenchPointProximityGoal(const std::string& name, double weight);
	
	//=========================================================================
    // METHODS
    //=========================================================================

    void setActuatorPath(const std::string& actuatorPath) {
        set_actuator_path(actuatorPath);
    }
    std::string getActuatorPath() const { return get_actuator_path(); }

    void setAxisWeights(const SimTK::Vec3& weights) {
        set_axis_weights(weights);
    }
    SimTK::Vec3 getAxisWeights() const { return get_axis_weights(); }


protected:

    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

private:

    void constructProperties();
    
	mutable SimTK::ReferencePtr<const BodyWrenchSmoothContactActuator>
            _actuator;
	
};

} // namespace OpenSim

#endif // OPENSIM_MOCOWRENCHPOINTPROXIMITYGOAL_H
