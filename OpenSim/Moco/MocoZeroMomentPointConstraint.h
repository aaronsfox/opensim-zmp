#ifndef OPENSIM_MOCOZEROMOMENTPOINTCONSTRAINT_H
#define OPENSIM_MOCOZEROMOMENTPOINTCONSTRAINT_H

/* -------------------------------------------------------------------------- *
 *                     MocoZeroMomentPointConstraint.h                        *
 * -------------------------------------------------------------------------- */

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

#include <OpenSim/Actuators/BodyWrenchActuator.h>

namespace OpenSim {

class MocoProblemInfo;
class ZeroMomentPointEstimator;
class ZeroMomentPointForce;

//============================================================================//
//                    MocoZeroMomentPointConstraint                           //
//============================================================================//
/**
 * The MocoZeroMomentPointConstraint can be used within the MocoStudy tool to
 * enforce that external ground reactions applied via a BodyWrenchActuator are
 * consistent with the ZMP method. These actuators apply a force and torque at
 * a given point, and based on rearranging the equations Xiang et al. we can
 * constrain these values to be consistent with the Zero Moment Point method.
 * 
 * Xiang et al. (2009): https://doi.org/10.1002/nme.2575
 * 
 * @author Aaron Fox
 */

class OSIMMOCO_API MocoZeroMomentPointConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoZeroMomentPointConstraint, MocoPathConstraint);

public:

    //=========================================================================
    // PROPERTIES
    //=========================================================================

	OpenSim_DECLARE_PROPERTY(zmp_estimator_path, std::string,
        "The absolute component path of the ZeroMomentEstimator supplying "
		"estimated forces, torques and COP.");

    OpenSim_DECLARE_LIST_PROPERTY(body_wrench_paths, std::string,
        "The absolute component paths of the ZeroMomentPointForce actuators "
		"to track.");

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
	/** Default constructor. */
	MocoZeroMomentPointConstraint();

    /** Constructor with object name and ZMP estimator path listed. */
    MocoZeroMomentPointConstraint(const std::string& name,
        const std::string& zmpEstimatorPath);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Set ZMP estimator path. */
    void setZeroMomentPointEstimatorPath(
            const std::string& zmpEstimatorPath) {
        set_zmp_estimator_path(zmpEstimatorPath);
    };

    /** Get ZMP estimator path. */
    void getZeroMomentPointEstimatorPath() {
        get_zmp_estimator_path();
    };

    /** Append body wrench actuator object path. */
    void addBodyWrenchActuatorPath(const std::string& bodyWrenchPath) {
        append_body_wrench_paths(bodyWrenchPath);
    };

protected:

    /** Discovers component layouts and dynamically sizes constraint equations. */
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;

	/** Computes residual error bounds between Moco controls and physics outputs. */
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:

    // Property construction
    void constructProperties();

    // Cached points
   
    // ZMP estimator
	mutable SimTK::ResetOnCopy<SimTK::ReferencePtr<ZeroMomentPointEstimator>> _estimator{nullptr};

    // Force components
    mutable std::vector<const BodyWrenchActuator*> _forceComponents;

    // Cache matrix matching the layout design pattern for the ZMP forces
    struct ForceControlIndices {
        int fx_idx, fy_idx, fz_idx;
        int cop_x_idx, cop_y_idx, cop_z_idx;
        int tx_idx, ty_idx, tz_idx;
    };

    // Cache for storing control indices
    mutable std::vector<ForceControlIndices> m_controlIndices;

    // Cache the matched contact side names associated with the force components
    // This is based on the contact body name matching the ZMP estimator contact sides
    mutable std::vector<std::string> m_matchedSideNames;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOZEROMOMENTPOINTCONSTRAINT_H
