#ifndef OPENSIM_CONTACT_SIDE_ACTUATOR_H_
#define OPENSIM_CONTACT_SIDE_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ContactSideActuator.h                        *
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

#include "osimActuatorsDLL.h"
#include <string>
#include <utility>
#include <vector>

#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ContactSide.h>

namespace OpenSim {

/** Base class for actuators that apply a ground reaction load (force, center
of pressure and free moment) to a ContactSide, with controls that an optimal
control problem (e.g., in Moco) chooses. The load is only available where the
side's ContactPoint%s touch the ground, through a smooth contact gate, so
contact timing and the support polygon follow from the model's motion rather
than from data.

The load always satisfies the Zero Moment Point relations: it is applied on
the ground plane, so its moment about the center of pressure has no
horizontal component. With no other external loads (e.g., no pelvis residual
actuators), the dynamics then make the center of pressure equal the Zero
Moment Point of the motion.

### Contact enforcement
`contact_enforcement` selects how load is restricted to points that touch
the ground:
- 'constraint' (default; recommended for predictive problems): the controls
  set the load directly, and MocoContactPointConstraint (with
  `enforce_contact_complementarity`) adds the relaxed complementarity
  constraint \f$ \lambda_i (h_i - z_{t,i}) \le \epsilon \f$ for each
  contact location (see calcContactLocations()), where \f$ \lambda_i \f$
  is the location's load as a fraction of maximum_normal_force and
  \f$ z_{t,i} \f$ a contact band (`distance_threshold`). The constraint's
  gradient with respect to height is \f$ \lambda_i \f$, so an optimizer
  that wants load at a point in the air is pulled toward the ground, even
  from an initial guess with the feet in the air. Add
  MocoContactPenaltyGoal to remove the small load the relaxation allows
  above the band and (for the 'velocity' method) to penalize sliding.
- 'gate': the load is multiplied by a smooth contact gate (below), so no
  constraint is needed. Far above the ground the gate is nearly 0 and nearly
  flat, so an optimizer gets no indication that lowering the foot makes load
  available: start from a guess with the feet near the ground where they
  should be in contact.

### Contact gate ('gate' enforcement)
Each ContactPoint has a gate value between 0 and 1 that limits the load it
can carry, using the point's `contact_checking_method` and thresholds:
- 'distance': \f$ g = \sigma\left((z_t - h)/w_h\right) \f$, where \f$ h \f$
  is the point's height above the ground plane (ContactPoint::
  calcHeightAboveGround()), \f$ z_t \f$ its `distance_threshold` and
  \f$ w_h \f$ the actuator's `distance_smoothing`.
- 'velocity': the distance gate multiplied by
  \f$ \sigma\left((v_t - |\mathbf{v}_h|)/w_v\right) \f$, where
  \f$ |\mathbf{v}_h| \f$ is the point's horizontal speed in Ground,
  \f$ v_t \f$ its `velocity_threshold` and \f$ w_v \f$ the actuator's
  `velocity_smoothing`. Loaded points cannot slide. Unlike ContactPoint's
  contact detection (Karcnik 2003), which uses the 3D speed, only the
  horizontal speed is gated: a point moving into the ground must be able to
  carry load to stop the foot at impact, and vertical motion is already
  limited by the distance gate and MocoContactPointConstraint.
\f$ \sigma(x) = 1/(1 + e^{-x}) \f$ is the logistic function. The gate is a
smoothed form of the complementarity condition "load only where the point
touches the ground": a point's load is bounded by its gate, which is near 1 on
the ground and falls to near 0 within a few smoothing widths above the
threshold. Smaller smoothing widths are more exact but make the optimal
control problem stiffer. ContactPoint hysteresis properties are not used.

Use with MocoContactPointConstraint to prevent ground penetration.

### Load dynamics (optional)
With a positive `load_time_constant` \f$ \tau \f$, the normal load
\f$ \lambda \f$ (fraction of maximum_normal_force, before the gate) is a
state variable, 'normal_load', following the normal control \f$ u_n \f$:
\f[ \dot{\lambda} = (u_n - \lambda) / \tau \f]
This smooths the ground reaction in time, standing in for the compliance of
the heel pad and shoe: the load cannot rise or fall faster than \f$ \tau \f$
allows, which discourages impulsive (single mesh point) loads and bouncing
solutions. It also lowers and delays the impact peak (in the landing test,
462 N without dynamics and 289 N with a 20 ms time constant). Lift-off
requires the load to decay first, so the time constant also delays
lift-off. The load stays in [0, 1] if its initial value is. In Moco, the
state is `<actuator path>/normal_load`, with default bounds [0, 1] (from the
`statebounds_normal_load` output); set its initial value if it is known (e.g.,
0 when the foot starts in the air). Values of 10-30 ms
are typical of the rise time of the vertical ground reaction at heel strike.

With a positive `load_release_time_constant` \f$ \tau_r \f$, the load rises
with \f$ \tau \f$ and falls with \f$ \tau_r \f$, blended smoothly (as in
De Groote et al.'s (2016) activation dynamics):
\f[ \dot{\lambda} = (u_n - \lambda) \left[ \frac{f}{\tau}
        + \frac{1 - f}{\tau_r} \right], \quad
    f = \tfrac{1}{2} + \tfrac{1}{2} \tanh(10 (u_n - \lambda)) \f]
A short \f$ \tau_r \f$ (e.g., 2-5 ms) keeps the smoothing of loading at
impact but lets the load drop quickly at lift-off. With a long release time
constant, the decay of the load, rather than the controls, shapes the end of
stance, and the complementarity constraint then holds the loaded points within
the contact band until the load has decayed.

### Friction and free moment
The tangential force is bounded by a friction pyramid,
\f$ |F_x|, |F_z| \le \mu F_y \f$ (`friction_coefficient`), and the free
moment by \f$ |\tau_y| \le r F_y \f$ (`free_moment_radius`).

### Controls
Concrete classes define the controls (see ContactSideWrenchActuator and
ContactSidePointActuator); getControlBounds() and getControlLabels() describe
them. In Moco, control `i` is named `<actuator path>_<i>` and has no bounds
unless set, e.g.:
@code
const auto bounds = actuator.getControlBounds();
for (int i = 0; i < actuator.numControls(); ++i) {
    problem.setControlInfo(actuator.getAbsolutePathString() + "_" +
            std::to_string(i), {bounds[i].first, bounds[i].second});
}
@endcode

### Outputs
- `ground_reaction_force` (Vec3): total force, in Ground (N).
- `center_of_pressure` (Vec3): on the ground plane, in Ground (m); zero when
  the normal force is zero.
- `free_moment` (Vec3): moment about the center of pressure, in Ground (N-m);
  only the y component is non-zero.
These match the outputs of ZeroMomentPointEstimator and the columns of
createZMPLoadsFromStates().

@author Aaron Fox */
class OSIMACTUATORS_API ContactSideActuator : public Actuator {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ContactSideActuator, Actuator);

public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_PROPERTY(maximum_normal_force, double,
            "Maximum normal (vertical) force (N). Controls are scaled by this; "
            "2-3 times body weight per side is typical. Default 2000 N.");
    OpenSim_DECLARE_PROPERTY(friction_coefficient, double,
            "Friction coefficient bounding the tangential force (friction "
            "pyramid). Default 0.8.");
    OpenSim_DECLARE_PROPERTY(free_moment_radius, double,
            "Radius (m) bounding the free moment: |free moment| <= radius * "
            "normal force. Default 0.02 m.");
    OpenSim_DECLARE_PROPERTY(distance_smoothing, double,
            "Width (m) of the smooth transition of the distance gate. Default "
            "0.002 m.");
    OpenSim_DECLARE_PROPERTY(velocity_smoothing, double,
            "Width (m/s) of the smooth transition of the velocity gate "
            "('velocity' contact_checking_method only). Default 0.005 m/s.");
    OpenSim_DECLARE_PROPERTY(contact_enforcement, std::string,
            "How load is restricted to points touching the ground: "
            "'constraint' (controls set the load; use "
            "MocoContactPointConstraint with "
            "enforce_contact_complementarity) or 'gate' (load multiplied by a "
            "smooth contact gate). Default 'constraint' ('gate' for "
            "ContactSideWrenchActuator).");
    OpenSim_DECLARE_PROPERTY(load_time_constant, double,
            "Time constant (s) of first-order dynamics of the normal load: "
            "the normal control sets a target that the load follows (state "
            "variable 'normal_load'). Smooths the ground reaction in time. "
            "Non-positive: no dynamics, the control is the load. Default -1.");
    OpenSim_DECLARE_PROPERTY(load_release_time_constant, double,
            "Time constant (s) of the load dynamics while the load decreases "
            "(control below the load), if positive; otherwise "
            "load_time_constant is used for both. A short release time "
            "constant smooths loading at impact without delaying lift-off. "
            "Default -1.");

    //==========================================================================
    // SOCKETS
    //==========================================================================
    OpenSim_DECLARE_SOCKET(contact_side, ContactSide,
            "The ContactSide that this actuator applies a ground reaction to.");

    //==========================================================================
    // OUTPUTS
    //==========================================================================
    OpenSim_DECLARE_OUTPUT(ground_reaction_force, SimTK::Vec3,
            calcGroundReactionForce, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(center_of_pressure, SimTK::Vec3,
            calcCenterOfPressure, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(
            free_moment, SimTK::Vec3, calcFreeMoment, SimTK::Stage::Velocity);
    /** Bounds [0, 1] of the 'normal_load' state variable (load dynamics
    only), used by Moco as the state's default bounds. */
    OpenSim_DECLARE_OUTPUT(statebounds_normal_load, SimTK::Vec2,
            getBoundsNormalLoad, SimTK::Stage::Model);

    //==========================================================================
    // METHODS
    //==========================================================================
    const ContactSide& getContactSide() const;
    void setContactSide(const ContactSide& side);

    /** Lower and upper bounds of each control, in control order. */
    virtual std::vector<std::pair<double, double>> getControlBounds() const = 0;
    /** A short description of each control, in control order (e.g.,
    "normal", "friction_x"). */
    virtual std::vector<std::string> getControlLabels() const = 0;

    /** Gate value (0 to 1) of each of the side's ContactPoints; all 1 with
    'constraint' enforcement. The system must be realized to
    SimTK::Stage::Velocity. */
    std::vector<double> calcPointGates(const SimTK::State& state) const;

    /** Whether the load is multiplied by the contact gate ('gate'
    enforcement). Valid after finalizeFromProperties(). */
    bool usesGate() const { return m_usesGate; }

    /** Whether the normal load has first-order dynamics (positive
    `load_time_constant`). */
    bool hasLoadDynamics() const { return get_load_time_constant() > 0; }
    /** Normal load, as a fraction of maximum_normal_force, before the
    contact gate: the 'normal_load' state variable with load dynamics,
    otherwise the normal control. */
    double getNormalLoad(const SimTK::State& state) const;
    /** Set the 'normal_load' state variable (load dynamics only). */
    void setNormalLoad(SimTK::State& state, double load) const;
    /** Bounds of the 'normal_load' state variable: [0, 1]. */
    SimTK::Vec2 getBoundsNormalLoad(const SimTK::State&) const {
        return {0, 1};
    }

    /** A location where load is applied, for contact complementarity
    constraints. */
    struct ContactLocation {
        std::string name;       ///< E.g., the ContactPoint's name.
        double load;            ///< Normal load / maximum_normal_force (>= 0).
        double height;          ///< Height above the ground plane (m).
        double horizontalSpeed; ///< Horizontal speed in Ground (m/s).
        bool checksSliding;     ///< 'velocity' contact_checking_method.
        double contactHeight;   ///< Contact band (distance_threshold, m).
        double velocityThreshold; ///< velocity_threshold (m/s).
    };
    /** Load, height and horizontal speed of each location where this
    actuator applies load (each ContactPoint for ContactSidePointActuator; the
    center of pressure for ContactSideWrenchActuator). The system must be
    realized to SimTK::Stage::Velocity. */
    virtual std::vector<ContactLocation> calcContactLocations(
            const SimTK::State& state) const = 0;
    /** The contact locations' names and checksSliding flags, in the order of
    calcContactLocations() (loads, heights and speeds are zero); no state is
    needed. The contact_side socket must be connected. */
    virtual std::vector<ContactLocation> getContactLocationInfo() const = 0;

    /** @name Ground reaction (system realized to SimTK::Stage::Velocity) */
    // @{
    SimTK::Vec3 calcGroundReactionForce(const SimTK::State& state) const;
    SimTK::Vec3 calcCenterOfPressure(const SimTK::State& state) const;
    SimTK::Vec3 calcFreeMoment(const SimTK::State& state) const;
    // @}

    /** Power delivered to the model (W). */
    double getPower(const SimTK::State& state) const override;

protected:
    ContactSideActuator();

    /** A force applied at a point on the ground plane. */
    struct PointLoad {
        const PhysicalFrame* frame; ///< Frame the force is applied to.
        SimTK::Vec3 pointInFrame;   ///< Application point, in `frame`.
        SimTK::Vec3 pointInGround;  ///< Application point, in Ground.
        SimTK::Vec3 force;          ///< Force, in Ground.
    };

    /** Compute the point forces and the free moment (in Ground, applied to
    the contact body) from the controls and the state. All application points
    must be on the ground plane. */
    virtual void calcLoads(const SimTK::State& state,
            std::vector<PointLoad>& loads, SimTK::Vec3& freeMoment) const = 0;

    /** Gate of one point (see class description). */
    double calcGate(const SimTK::State& state, const ContactPoint& point) const;
    /** Gate of a material point of `frame` at `pointInFrame`, using the given
    method and thresholds. */
    double calcGate(const SimTK::State& state, const PhysicalFrame& frame,
            const SimTK::Vec3& pointInFrame,
            ContactPoint::ContactCheckingMethod method,
            double distanceThreshold, double velocityThreshold) const;

    /** Project a point in Ground onto the ground plane. */
    SimTK::Vec3 projectToGround(const SimTK::Vec3& pointInGround) const;

    /** Height above the ground plane and horizontal speed of a material point
    of `frame` at `pointInFrame`. The horizontal speed is smoothed near zero
    so that it is differentiable. */
    void calcHeightAndHorizontalSpeed(const SimTK::State& state,
            const PhysicalFrame& frame, const SimTK::Vec3& pointInFrame,
            double& height, double& horizontalSpeed) const;

    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& state) const override;
    void computeStateVariableDerivatives(
            const SimTK::State& state) const override;

private:
    void implProduceForces(const SimTK::State& state,
            ForceConsumer& forceConsumer) const override;
    void constructProperties();

    bool m_usesGate = false;
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_SIDE_ACTUATOR_H_
