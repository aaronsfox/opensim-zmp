#ifndef OPENSIM_MOCO_CONTACT_POINT_CONSTRAINT_H
#define OPENSIM_MOCO_CONTACT_POINT_CONSTRAINT_H
/* -------------------------------------------------------------------------- *
 *                 OpenSim: MocoContactPointConstraint.h                      *
 * -------------------------------------------------------------------------- *
 * A MocoPathConstraint that prevents ContactPoint objects from penetrating   *
 * the ground plane in a Moco direct-collocation problem.                     *
 *                                                                            *
 * ContactPoints are specified by their absolute component paths. This makes  *
 * the constraint independent of any specific parent component.
 *                                                                            *
 * For each specified ContactPoint, the constraint enforces:                  *
 *   point_height(t) >= ground_plane_height  for all t                        *
 *                                                                            *
 * Note that the y-axis is assumed as vertical consistent with standard       *
 * OpenSim coordinate systems.                                                *
 *                                                                            *
 * This is a purely kinematic constraint at Stage::Position. It applies no    *
 * forces.                                                                    *
 *                                                                            *
 * Set enforce_path_constraint_midpoints=true in MocoCasADiSolver when using  *
 * Hermite-Simpson collocation to prevent penetration between mesh points.    *
 * -------------------------------------------------------------------------- */

#include "MocoConstraint.h"

#include <OpenSim/Simulation/Model/ContactPoint.h>

#include <vector>
#include <string>

namespace OpenSim {

//============================================================================//
//                       MocoContactPointConstraint                           //
//============================================================================//
/**
 * Prevents specified ContactPoints from penetrating a ground plane.
 *
 * ### Properties
 * - `contact_point_paths` : list<string>
 *     Absolute component path to each ContactPoint to constrain.
 *     e.g. "/zmp_grf_estimator/right_side/contact_point_RCAL"
 *     At least one path must be provided.
 * - `ground_plane_height` : double (default 0.0 m)
 *     Vertical coordinate of the ground plane. Constraint enforces
 *     point_height >= ground_plane_height.
 *
 * ### Constraint equations
 * One inequality per ContactPoint:
 *   c_i(q) = point_vertical_coord_i(q) - ground_plane_height >= 0
 *
 * ### Usage
 * @code
 * auto* con = new MocoContactPointConstraint();
 * con->setName("no_penetration_right");
 * con->set_ground_plane_height(0.0);
 * con->addContactPointPath(
 *     "/zmp_grf_estimator/right_side/contact_point_RCAL");
 * con->addContactPointPath(
 *     "/zmp_grf_estimator/right_side/contact_point_RTOE");
 * problem.addPathConstraint(con);
 * @endcode
 *
 * @author Aaron Fox
 */

class OSIMMOCO_API MocoContactPointConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactPointConstraint,
                                    MocoPathConstraint);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================
    OpenSim_DECLARE_LIST_PROPERTY(contact_point_paths, std::string,
        "Absolute component path to each ContactPoint to constrain. "
        "At least one path must be provided.");

    OpenSim_DECLARE_PROPERTY(ground_plane_height, double,
        "Vertical coordinate (m) of the ground plane. "
        "Constraint enforces: point_height >= ground_plane_height. "
        "Default 0.0 m.");

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    MocoContactPointConstraint();
    MocoContactPointConstraint(const MocoContactPointConstraint&) = default;
    virtual ~MocoContactPointConstraint() = default;

    //=========================================================================
    // CONVENIENCE METHODS
    //=========================================================================

    /** Appends an absolute ContactPoint path to the constraint. */
    void addContactPointPath(const std::string& absolutePath) {
        append_contact_point_paths(absolutePath);
    }

    void setGroundPlaneHeight(double height) {
        set_ground_plane_height(height);
    }

    int getNumContactPointPaths() const {
        return getProperty_contact_point_paths().size();
    }

protected:
    //=========================================================================
    // MOCOPATHCONSTRAINT INTERFACE
    //=========================================================================

    /** Validates properties, resolves ContactPoint paths in the model,
     *  and sets constraint bounds (one per point, [0, +inf)). */
    void initializeOnModelImpl(const Model&       model,
                               const MocoProblemInfo& problem) const override;

    /** Evaluates the vertical height of each ContactPoint above
     *  ground_plane_height and writes results to errors.
     *  Non-negative error means the constraint is satisfied.
     *  Requires Stage::Position. */
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state,
            SimTK::Vector&      errors) const override;

private:
    void constructProperties();

    /** Runtime record of a resolved ContactPoint. */
    struct ResolvedPoint {
        std::string absolutePath;
        std::string displayName;  // last path element for readability
    };

    mutable std::vector<ResolvedPoint> _resolvedPoints;
};

} // namespace OpenSim

#endif // OPENSIM_MOCO_CONTACT_POINT_CONSTRAINT_H
