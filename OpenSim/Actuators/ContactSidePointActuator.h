#ifndef OPENSIM_CONTACT_SIDE_POINT_ACTUATOR_H_
#define OPENSIM_CONTACT_SIDE_POINT_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ContactSidePointActuator.h                      *
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

#include "ContactSideActuator.h"

namespace OpenSim {

/** A ContactSideActuator whose normal load is shared among the side's
ContactPoint%s. See ContactSideActuator for the contact gate, friction, free
moment and outputs.

### Controls
For a side with \f$ n \f$ ContactPoints there are \f$ n + 4 \f$ controls:
| Index | Control | Bounds |
| ----- | ------- | ------ |
| 0 | total normal load \f$ u_n \f$ | [0, 1] |
| 1 to n | distribution weight \f$ w_i \f$ of point i | [0, 1] |
| n+1 | friction \f$ u_x \f$ | [-1, 1] |
| n+2 | friction \f$ u_z \f$ | [-1, 1] |
| n+3 | free moment \f$ u_\tau \f$ | [-1, 1] |

### Loads
Point \f$ i \f$ carries the share
\f$ s_i = (w_i + \delta) / \sum_j (w_j + \delta) \f$ of the total load
(\f$ \delta = 10^{-3} \f$ keeps the shares defined when all weights are
zero), so its normal load is \f$ \lambda_i = g_i u_n s_i \f$, where
\f$ g_i \f$ is its contact gate (1 with 'constraint' enforcement), and it
receives the force
\f[ \mathbf{f}_i = F_{max} \lambda_i\,(\mu u_x,\ 1,\ \mu u_z) \f]
(in Ground) at its projection onto the ground plane, applied to the point's
parent frame. The free moment \f$ r F_y u_\tau \f$ about the vertical axis is
applied to the contact body, where \f$ F_y = F_{max} \sum_i \lambda_i \le
F_{max} \f$: `maximum_normal_force` is the maximum for the side, as for
ContactSideWrenchActuator.

The center of pressure is the \f$ \lambda \f$-weighted mean of the points'
ground projections. Because the weights are non-negative and only points in
contact can carry load, the center of pressure always lies in the support
polygon (the convex hull of the loaded points' projections), without any
constraint, and the support polygon changes during stance as points come into
and out of contact. The load shares are not unique for a given center of
pressure, so include an effort term on the controls; the weights only matter
relative to each other.

@author Aaron Fox */
class OSIMACTUATORS_API ContactSidePointActuator : public ContactSideActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ContactSidePointActuator, ContactSideActuator);

public:
    ContactSidePointActuator();
    /** Create an actuator for `side`. */
    explicit ContactSidePointActuator(const ContactSide& side);

    /** n + 4, where n is the number of the side's ContactPoints. The
    contact_side socket must be connected. */
    int numControls() const override;
    std::vector<std::pair<double, double>> getControlBounds() const override;
    std::vector<std::string> getControlLabels() const override;

    /** Normal load \f$ \lambda_i \f$ of each ContactPoint (fraction of
    maximum_normal_force). The system must be realized to
    SimTK::Stage::Velocity. */
    std::vector<double> calcPointLoads(const SimTK::State& state) const;

    /** One location per ContactPoint, with load \f$ \lambda_i \f$. */
    std::vector<ContactLocation> calcContactLocations(
            const SimTK::State& state) const override;
    std::vector<ContactLocation> getContactLocationInfo() const override;

protected:
    void calcLoads(const SimTK::State& state, std::vector<PointLoad>& loads,
            SimTK::Vec3& freeMoment) const override;
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_SIDE_POINT_ACTUATOR_H_
