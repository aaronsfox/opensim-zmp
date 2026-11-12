#ifndef OPENSIM_CONTACT_SIDE_WRENCH_ACTUATOR_H_
#define OPENSIM_CONTACT_SIDE_WRENCH_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ContactSideWrenchActuator.h                     *
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

/** A ContactSideActuator that applies one force at a center of pressure
chosen by its controls, plus a free moment. See ContactSideActuator for the
friction, free moment and outputs.

### Controls
| Index | Control | Bounds |
| ----- | ------- | ------ |
| 0 | normal force \f$ u_n \f$ | [0, 1] |
| 1 | friction \f$ u_x \f$ | [-1, 1] |
| 2 | friction \f$ u_z \f$ | [-1, 1] |
| 3 | center of pressure, fore-aft \f$ a \f$ | [0, 1] |
| 4 | center of pressure, medio-lateral \f$ b \f$ | [0, 1] |
| 5 | free moment \f$ u_\tau \f$ | [-1, 1] |

### Center of pressure
The center of pressure is a material point \f$ \mathbf{c} \f$ of the contact
body's sole: in the contact body frame,
\f$ c_x = x_{min} + a (x_{max} - x_{min}) \f$,
\f$ c_z = z_{min} + b (z_{max} - z_{min}) \f$, and \f$ c_y \f$ is the mean
height of the ContactPoints, where \f$ [x_{min}, x_{max}] \f$ and
\f$ [z_{min}, z_{max}] \f$ are the ranges of the ContactPoint locations in the
contact body frame. The x and z axes of the contact body are assumed to be
the foot's fore-aft and medio-lateral axes, and the sole is assumed to be
flat, so the ContactPoints should be at similar heights in the contact body
frame. All ContactPoints must be fixed to the contact body and use the same
contact_checking_method.

### Loads
The normal force is \f$ F_y = F_{max}\, g(\mathbf{c})\, u_n \f$, where
\f$ g(\mathbf{c}) \f$ is the contact gate evaluated at the center of pressure
material point itself, using the ContactPoints' method and mean thresholds.
The force \f$ F_y (\mu u_x, 1, \mu u_z) \f$ (in Ground) is applied to the
contact body at the projection of \f$ \mathbf{c} \f$ onto the ground plane,
with the free moment \f$ r F_y u_\tau \f$ about the vertical axis. Because the
gate is evaluated at the center of pressure, the load can only act where the
sole touches the ground, without any constraint.

Compared with ContactSidePointActuator, this has fewer controls, but its
center of pressure ranges over the rectangle spanned by the ContactPoints
rather than their convex hull.

### Intended use: tracking problems, 'gate' enforcement
This actuator uses 'gate' contact_enforcement by default and is intended for
tracking problems, where an initial guess with the feet near the ground in
stance is available from the experimental data (e.g., Inverse Kinematics).
From such a guess, the gate formulation solves reliably (e.g., the landing
test in testMocoContactSideActuators). With 'constraint' enforcement, or
from a guess with the feet in the air, it rarely converges: its single
complementarity condition is evaluated at the center of pressure, whose
height depends on the center of pressure controls as well as the motion (so
the optimizer can satisfy it by moving the center of pressure rather than
the foot), and the center of pressure controls are undetermined when there
is no load. For predictive problems, or without a good guess, use
ContactSidePointActuator with 'constraint' enforcement. Both actuators apply
a single wrench per side (ContactSidePointActuator's point forces share one
direction), so they represent the same ground reactions.

@author Aaron Fox */
class OSIMACTUATORS_API ContactSideWrenchActuator : public ContactSideActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ContactSideWrenchActuator, ContactSideActuator);

public:
    ContactSideWrenchActuator();
    /** Create an actuator for `side`. */
    explicit ContactSideWrenchActuator(const ContactSide& side);

    int numControls() const override { return 6; }
    std::vector<std::pair<double, double>> getControlBounds() const override;
    std::vector<std::string> getControlLabels() const override;
    std::array<int, 3> getShearControlIndices() const override;

    /** The center of pressure material point in the contact body frame, for
    the given fore-aft and medio-lateral controls (each 0 to 1). */
    SimTK::Vec3 calcCenterOfPressureInBody(
            double foreAft, double medioLateral) const;

    /** The contact gate at the center of pressure. The system must be
    realized to SimTK::Stage::Velocity. */
    double calcCenterOfPressureGate(const SimTK::State& state) const;

    /** One location, "center_of_pressure": the center of pressure material
    point, with load \f$ g(\mathbf{c})\, u_n \f$. */
    std::vector<ContactLocation> calcContactLocations(
            const SimTK::State& state) const override;
    std::vector<ContactLocation> getContactLocationInfo() const override;

protected:
    void calcLoads(const SimTK::State& state, std::vector<PointLoad>& loads,
            SimTK::Vec3& freeMoment) const override;
    void extendConnectToModel(Model& model) override;

private:
    // Sole geometry in the contact body frame and contact settings, set in
    // extendConnectToModel().
    SimTK::Vec3 m_minimum{0};
    SimTK::Vec3 m_maximum{0};
    double m_soleHeight = 0;
    ContactPoint::ContactCheckingMethod m_method =
            ContactPoint::ContactCheckingMethod::Distance;
    double m_distanceThreshold = 0;
    double m_velocityThreshold = 0;
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_SIDE_WRENCH_ACTUATOR_H_
