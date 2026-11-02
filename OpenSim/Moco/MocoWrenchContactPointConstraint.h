#ifndef OPENSIM_MOCOWRENCHCONTACTPOINTCONSTRAINT_H
#define OPENSIM_MOCOWRENCHCONTACTPOINTCONSTRAINT_H

/* -------------------------------------------------------------------------- *
 *                     MocoWrenchContactPointConstraint.h                     *
 * -------------------------------------------------------------------------- */

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

#include <OpenSim/Actuators/BodyWrenchSmoothContactActuator.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>

#include <string>
#include <vector>

// TODO: this should be simpler and just allow the user to enter the centroid/body
// point location as a Vec3 --- notes around Xiang et al. are also probably now
// unnecessary...

// TODO: this should be renamed something like MocoContactPointProximityConstraint

namespace OpenSim {

class MocoProblemInfo;
class BodyWrenchSmoothContactActuator;
class ContactPoint;

//============================================================================//
//                    MocoWrenchContactPointConstraint                        //
//============================================================================//
/**
 *
 *   TODO: completely re-write this...recommendations for Big-M is 5-10x max distance...?
 *
 * MocoWrenchContactPointConstraint enforces that the point of application
 * (COP) of a BodyWrenchActuator remains inside the Foot Support Polygon (FSP)
 * based on the location of contact points and there determination of ground
 * contact using smooth continuous formulations.
 *
 * The support polygon is defined directly by a user-supplied, ordered list
 * of ContactPoint component paths (contact_point_paths) rather than being
 * discovered from a ZeroMomentPointEstimator/ContactSide. The listed points
 * must be given in sequential order around the polygon perimeter (either
 * clockwise or counter-clockwise, but consistently) so that consecutive
 * entries define the polygon's edges.
 * 
 * This formulation predominantly replicates the FSP approach of Xiang et al.
 * 
 * Xiang et al. (2009): https://doi.org/10.1002/nme.2575
 * 
 * @author Aaron Fox
 
 * Path constraint enforcing that a BodyWrenchActuator's applied point of     *
 * force application stays within a per-axis distance (expressed in the      *
 * actuator's body frame) of that actuator's smooth contact-point centroid,  *
 * whenever the actuator's contact indicator is active. Uses a big-M         *
 * relaxation so the constraint is vacuous when the body is not in contact   *
 * with the ground.                                                          *
 *                                                                            *
 * For each body-frame axis i:                                              *
 *   error_upper_i =  diff[i] - max_distance[i] - big_m * (1 - C)  <= 0      *
 *   error_lower_i = -diff[i] - max_distance[i] - big_m * (1 - C)  <= 0      *
 * where diff = p_body - centroid_body.                                     *
 */

class OSIMMOCO_API MocoWrenchContactPointConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoWrenchContactPointConstraint, MocoPathConstraint);

public:

    //=========================================================================
    // PROPERTIES
    //=========================================================================

	OpenSim_DECLARE_PROPERTY(actuator_path, std::string,
        "The absolute component path of the BodyWrenchSmoothContactActuator "
		"object that this constraint controls the point application for.");

    OpenSim_DECLARE_PROPERTY(distance_tolerance, SimTK::Vec3,
            "Max allowed per-axis distance (m) between the applied point and "
            "the contact centroid, expressed in the actuator's body frame, "
            "enforced only when the actuator is in contact. "
            "Defaults to (0.05, 0.01, 0.05).");

    OpenSim_DECLARE_PROPERTY(big_m, double,
            "Big-M relaxation constant (m) used to deactivate this constraint "
            "when the contact indicator is near zero. Should be larger than "
            "the largest plausible per-axis separation during swing. "
            "Default 0.2.");

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
	/** Default constructor. */
	MocoWrenchContactPointConstraint();

    /** Constructor with object name and body wrench actuator path listed. */
    MocoWrenchContactPointConstraint(
            const std::string& name, const std::string& actuatorPath);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Set body wrench actuator object path. */
    void setActuatorPath(const std::string& actuatorPath) {
        set_actuator_path(actuatorPath);
    };
	
	/** Get body wrench actuator path. */
    std::string getActuatorPath() const {
        return get_actuator_path();
    };

    /** Set distance tolerance parameter. */
    void setDistanceTolerance(const SimTK::Vec3& tolerance) {
        set_distance_tolerance(tolerance);
    };

    /** Get distance tolerance parameter. */
    SimTK::Vec3 getDistanceTolerance() const { 
        return get_distance_tolerance(); 
    };

    /** Set the big-M parameter. */
    void setBigM(const double& m) {
        set_big_m(m);
    };

    /** Get the big-M parameter. */
    double getBigM() const {
        return get_big_m();
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

    //=========================================================================
    // CACHED MEMBERS (set in initializeOnModelImpl)
    //=========================================================================
   
    // Body wrench actuator and its contact body name
    mutable SimTK::ResetOnCopy<SimTK::ReferencePtr<BodyWrenchSmoothContactActuator>>
            _actuator{nullptr};

    //// Cache the resolved physical body frame pointer
    //mutable SimTK::ResetOnCopy<SimTK::ReferencePtr<const PhysicalFrame>>
    //        _bodyFrame{nullptr};

    ////// Cached pointers to the ordered contact point coordinates forming the
    ////// support polygon hull, resolved once from contact_point_coords.
    ////mutable std::vector<SimTK::ReferencePtr<const SimTK::Vec3>>
    ////        m_contactPoints;

    //// Sign (+1 or -1) applied to every edge's raw signed-distance
    //// calculation so that a positive result consistently means "outside
    //// the polygon", regardless of whether contact_point_paths happened to
    //// be specified clockwise or counter-clockwise. Determined once in
    //// initializeOnModelImpl by checking which sign places the polygon's
    //// own centroid on the "inside" side of its first edge.
    ////
    //// This sign is computed from the model's working/default state
    //// at initialization and assumes the contact points' winding order (as
    //// seen looking along -Y, down at the ground plane) does not flip over
    //// the course of the simulation. This holds for ordinary foot-flat/roll
    //// behavior. It would only break under an extreme (>90 degree) foot
    //// inversion/eversion that flips which side of the foot faces the
    //// ground.
    //mutable double m_hullOrientationSign{1.0};

    //// Cache matrix matching the layout design pattern for the actuator forces
    ////struct ControlIndices {
    ////    int cop_x_idx, cop_y_idx, cop_z_idx;
    ////};
    //struct ControlIndices {
    //    int fx_idx, fy_idx, fz_idx;
    //    int cop_x_idx, cop_y_idx, cop_z_idx;
    //    int tx_idx, ty_idx, tz_idx;
    //};
    //
    //// Cache for storing control indices
    //mutable ControlIndices m_controlIndices;

    //// Cache for body mass normalisation
    //mutable double m_bodyWeight{1.0}; 

    ////// Smooth helper functions
    ////static double softReLU(double x, double k);
    ////static double computeSmoothContactWeight(
    ////        double point_y, double dist_thresh, double trans_width);

    //// Signed perpendicular distance (in the ground XZ plane) from point q
    //// to the infinite line through edge (p1 -> p2), positive if q is to
    //// the "left" of the directed edge. This is a raw/uncalibrated sign --
    //// callers must multiply by m_hullOrientationSign to get a value where
    //// positive consistently means "outside the polygon".
    //static double computeRawEdgeDistance(
    //        const SimTK::Vec3& p1, const SimTK::Vec3& p2, const SimTK::Vec3& q);

};

} // namespace OpenSim

#endif // OPENSIM_MOCOWRENCHCONTACTPOINTCONSTRAINT_H
