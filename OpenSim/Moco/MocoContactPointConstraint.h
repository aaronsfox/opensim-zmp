#ifndef OPENSIM_MOCO_CONTACT_POINT_CONSTRAINT_H_
#define OPENSIM_MOCO_CONTACT_POINT_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  MocoContactPointConstraint.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2026 Stanford University and the Authors                     *
 * Author(s): Aaron Fox                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MocoConstraint.h"
#include "osimMocoDLL.h"
#include <string>
#include <vector>

#include <OpenSim/Simulation/Model/ContactPoint.h>

namespace OpenSim {

/** This path constraint prevents ContactPoint%s from penetrating the ground
plane. For each constrained ContactPoint, it enforces

\f[ h_i(q) \geq -\delta \f]

where \f$ h_i \f$ is the height of the point above the ground plane (see
ContactPoint::calcHeightAboveGround()) and \f$ \delta \f$ is the
`penetration_tolerance` (default 0). The ground plane is the horizontal plane
at the `ground_height` of the ZeroMomentPointEstimator that owns the point's
ContactSide (y = 0 for a ContactPoint that is not owned by one), so this
constraint, contact detection and the Zero Moment Point all use the same
ground. The constraint depends only on the model's position (it applies no
forces).

### Which points are constrained
- The points at `contact_point_paths`, and all points of the ContactSide%s at
  `contact_side_paths`.
- If both are empty (the default), every ContactPoint in the model.
A point listed more than once is constrained once. The constraint has one
equation per point, labelled with the point's name.

### Usage
@code
// Every ContactPoint in the model.
auto* noPenetration =
        problem.addPathConstraint<MocoContactPointConstraint>();
noPenetration->setName("no_penetration");

// Or only the right foot.
auto* right = problem.addPathConstraint<MocoContactPointConstraint>();
right->setName("no_penetration_right");
right->addContactSidePath("/zmp_estimator/right");
@endcode

### Recommendations
- **Place the ContactPoints on the sole.** The constraint keeps the points,
  not the foot geometry, above the ground. Points above the sole let the sole
  penetrate the ground, and points below it hold the foot above the ground.
  A static trial (or the stance phases of the motion) can be used to place
  each point on the ground plane.
- **Start from a feasible guess.** Feet in Inverse Kinematics results often
  penetrate the ground by a few millimeters, which violates this constraint
  at the initial guess. Raise the pelvis (e.g., `pelvis_ty`) in the guess by
  the deepest penetration, and/or solve first with a small
  `penetration_tolerance` (e.g., 0.005 m) and then re-solve with 0 from that
  solution.
- **Enforce the constraint between mesh points.** Path constraints are
  enforced at mesh points; with Hermite-Simpson or Legendre-Gauss
  transcription, also set `enforce_path_constraint_mesh_interior_points` to
  true in MocoCasADiSolver.
@ingroup mocopathcon */
class OSIMMOCO_API MocoContactPointConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoContactPointConstraint, MocoPathConstraint);

public:
    MocoContactPointConstraint();

    /** Constrain the ContactPoint at this path (e.g.,
    "/zmp_estimator/right/RCAL"). */
    void addContactPointPath(const std::string& path) {
        append_contact_point_paths(path);
    }
    /** Constrain all ContactPoints of the ContactSide at this path (e.g.,
    "/zmp_estimator/right"). */
    void addContactSidePath(const std::string& path) {
        append_contact_side_paths(path);
    }
    /** Remove all point and side paths, so that every ContactPoint in the
    model is constrained. */
    void clearPaths() {
        updProperty_contact_point_paths().clear();
        updProperty_contact_side_paths().clear();
    }

    /** Allowed penetration (m) below the ground plane; non-negative. */
    void setPenetrationTolerance(double tolerance) {
        set_penetration_tolerance(tolerance);
    }
    double getPenetrationTolerance() const {
        return get_penetration_tolerance();
    }

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(contact_point_paths, std::string,
            "Paths to ContactPoints to constrain.");
    OpenSim_DECLARE_LIST_PROPERTY(contact_side_paths, std::string,
            "Paths to ContactSides whose ContactPoints are constrained. If "
            "both contact_point_paths and contact_side_paths are empty, every "
            "ContactPoint in the model is constrained.");
    OpenSim_DECLARE_PROPERTY(penetration_tolerance, double,
            "Allowed penetration (m) of a point below the ground plane "
            "(non-negative). Default 0.");

    void constructProperties();

    // Resolved in initializeOnModelImpl().
    mutable std::vector<SimTK::ReferencePtr<const ContactPoint>> m_points;
};

} // namespace OpenSim

#endif // OPENSIM_MOCO_CONTACT_POINT_CONSTRAINT_H_
