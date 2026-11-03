#ifndef OPENSIM_MOCO_CONTACT_PENALTY_GOAL_H_
#define OPENSIM_MOCO_CONTACT_PENALTY_GOAL_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  MocoContactPenaltyGoal.h                       *
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

#include "MocoGoal.h"
#include <string>
#include <vector>

#include <OpenSim/Actuators/ContactSideActuator.h>

namespace OpenSim {

/** Penalizes ContactSideActuator load that acts where the foot is not in
contact: above the contact band, or (for the 'velocity'
contact_checking_method) while sliding. It complements
MocoContactPointConstraint's relaxed complementarity constraint, which keeps
the problem solvable but allows a small load above the band; this goal drives
that load toward zero within the same solve.

The cost is the time average of
\f[ \sum_{a} \sum_{i} \lambda_i \left[ w_h\,
        \frac{\mathrm{softplus}_s(h_i - z_{t,i})}{L_h}
      + w_v\, \frac{|\mathbf{v}_{h,i}|}{L_v} \right] \f]
over the contact locations \f$ i \f$ of each ContactSideActuator \f$ a \f$
(see ContactSideActuator::calcContactLocations()), where
\f$ \lambda_i \f$ is the location's load (fraction of maximum_normal_force),
\f$ h_i \f$ its height, \f$ z_{t,i} \f$ its contact band
(`distance_threshold`), \f$ |\mathbf{v}_{h,i}| \f$ its horizontal speed,
\f$ \mathrm{softplus}_s(x) = s \ln(1 + e^{x/s}) \f$ a smooth
\f$ \max(x, 0) \f$, \f$ w_h \f$ and \f$ w_v \f$ the `height_weight` and
`sliding_weight`, and \f$ L_h \f$ and \f$ L_v \f$ the `height_scale` and
`speed_scale`. The sliding term applies only to locations whose
ContactPoints use the 'velocity' contact_checking_method.

Both terms are smooth and keep the gradient with respect to height and speed
proportional to the load, so, like the complementarity constraint, they work
from an initial guess with the feet in the air. Being penalties, they reduce
but do not eliminate the load above the band and sliding; increase the
goal's weight for stricter contact.

The goal applies to the ContactSideActuators at `actuator_paths`, or to all
ContactSideActuators in the model if none are given.
@ingroup mocogoal */
class OSIMMOCO_API MocoContactPenaltyGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactPenaltyGoal, MocoGoal);

public:
    MocoContactPenaltyGoal();
    MocoContactPenaltyGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoContactPenaltyGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Penalize this ContactSideActuator (by default, all of them). */
    void addActuatorPath(const std::string& path) {
        append_actuator_paths(path);
    }
    void setHeightWeight(double weight) { set_height_weight(weight); }
    void setSlidingWeight(double weight) { set_sliding_weight(weight); }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(actuator_paths, std::string,
            "Paths to ContactSideActuators to penalize. If empty, all "
            "ContactSideActuators in the model.");
    OpenSim_DECLARE_PROPERTY(height_weight, double,
            "Weight of the term for load above the contact band. Default 1.");
    OpenSim_DECLARE_PROPERTY(sliding_weight, double,
            "Weight of the term for load while sliding ('velocity' "
            "contact_checking_method only). Default 1.");
    OpenSim_DECLARE_PROPERTY(height_scale, double,
            "Height (m) that normalizes the height term. Default 0.01 m.");
    OpenSim_DECLARE_PROPERTY(speed_scale, double,
            "Speed (m/s) that normalizes the sliding term. Default 0.1 m/s.");
    OpenSim_DECLARE_PROPERTY(height_smoothing, double,
            "Smoothing width s (m) of the smooth max(height - band, 0). "
            "Default 0.001 m.");

    void constructProperties();

    mutable std::vector<SimTK::ReferencePtr<const ContactSideActuator>>
            m_actuators;
};

} // namespace OpenSim

#endif // OPENSIM_MOCO_CONTACT_PENALTY_GOAL_H_
