#ifndef OPENSIM_ZERO_MOMENT_POINT_ESTIMATOR_H_
#define OPENSIM_ZERO_MOMENT_POINT_ESTIMATOR_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ZeroMomentPointEstimator.h                     *
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

#include <memory>

#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

namespace OpenSim {

//=============================================================================
//                          ZeroMomentPointEstimator
//=============================================================================
/** Estimates the ground reaction force (GRF), center of pressure (CoP) and
free moment for up to two contact sides from model kinematics, using the Zero
Moment Point (ZMP) method (Xiang et al. 2009; Dijkstra & Gutierrez-Farewik
2015).

### Method
1. The whole-body external wrench required by the motion is the residual of
   inverse dynamics at the model's root (free) joint, computed with gravity as
   the only applied force. Forces in the model (muscles, actuators, contact
   elements, ExternalLoads) are deliberately excluded so that the estimate
   depends only on the motion and the model's inertial properties.
2. The residual is converted to the equivalent spatial force on the root
   body at the joint's child frame and shifted to the Ground origin (Xiang et
   al. 2009, eq. 16).
3. The ZMP is the point on the ground plane (y = h, where h is
   `ground_height`) where the horizontal moment of this wrench is zero
   (Xiang et al. 2009, eq. 19, generalized to h != 0):
   \f[ x_{ZMP} = \frac{M_z + h F_x}{F_y}, \qquad
       z_{ZMP} = \frac{h F_z - M_x}{F_y} \f]
   The free moment is the vertical moment that remains about the ZMP:
   \f$ \tau_y = M_y - (z_{ZMP} F_x - x_{ZMP} F_z) \f$. (Xiang et al. 2009,
   eq. 20 has the opposite sign on the bracketed terms; the expression used
   here is the one consistent with eq. 19.)

Ground contact is detected with ContactSide / ContactPoint subcomponents.

- **No side in contact:** all outputs are zero and not valid.
- **One side in contact (unilateral):** the whole-body GRF and free moment
  are assigned to that side, and its CoP is the ZMP.
- **Both sides in contact (bilateral):** the split between sides is
  statically indeterminate and is not yet implemented. Outputs are zero and
  not valid (createZMPLoadsFromStates() reports these periods).
  TODO: add a bilateral method (e.g., the linear distance split of Xiang et
  al. 2009, eq. 22, or the approach of Dijkstra & Gutierrez-Farewik 2015).

Outputs are marked valid only when the vertical GRF is at least
`contact_force_threshold`.

### Accelerations
The estimate uses the generalized accelerations in the state
(`state.getUDot()`), so the system must be realized to
SimTK::Stage::Acceleration. To estimate GRFs from experimental kinematics,
prescribe the motion of every coordinate with a PositionMotion; the state's
accelerations are then the second derivatives of the coordinate splines:
@code{.cpp}
auto positionMotion = PositionMotion::createFromTable(model, coordinates);
model.addComponent(positionMotion.release());
SimTK::State& state = model.initSystem();
state.setTime(time);
model.getSystem().prescribe(state); // Set q and u from the PositionMotion.
model.realizeAcceleration(state);
SimTK::Vec3 grf = estimator.getGroundReactionForce(state, "right");
@endcode
Equivalently, record the `ground_reaction_force`, `center_of_pressure` and
`free_moment` outputs with a TableReporter while stepping through time.
createZMPLoadsFromKinematics() does this for a table of coordinate values.
When the accelerations come from data instead (e.g., a states table with
accelerations), pass them to calcZMPResult(), or use
createZMPLoadsFromStates().
Filter the coordinates before creating the PositionMotion, since accelerations
amplify noise. In a forward simulation without prescribed motion, the
accelerations are those produced by the model's forces, so the estimate
reproduces the external forces acting on the model (useful for validation
against a contact model).

### Assumptions
- Ground is a horizontal plane with Ground's y axis pointing up.
- All external forces act through the ground contacts.
- `free_joint_name` is the joint between Ground and the root of the model's
  body tree, with all other bodies descending from it.

### Properties
- `contact_sides`: list of ContactSide (one or two).
- `free_joint_name`: name of the root joint (default "ground_pelvis"). Its
  parent frame must be Ground (or a frame fixed to Ground).
- `contact_force_threshold`: minimum vertical GRF (N) for valid outputs
  (default 20 N).
- `ground_height`: height of the ground plane along the Ground y axis (m),
  default 0. Used for the ZMP and for contact detection by all ContactPoints.

### Outputs
List outputs have one channel per ContactSide, named after the side.
- `ground_reaction_force` (Vec3, Stage::Acceleration): GRF in Ground (N).
- `center_of_pressure` (Vec3, Stage::Acceleration): CoP in Ground (m), on the
  ground plane.
- `free_moment` (Vec3, Stage::Acceleration): free moment about the CoP,
  expressed in Ground (N-m); only the y component is non-zero.
- `is_in_contact` (bool, Stage::Velocity): whether the side is in contact.
- `is_valid` (bool, Stage::Acceleration): whether a GRF was assigned to the
  side and its vertical component is at least `contact_force_threshold`.
- `contact_mode` (int, Stage::Velocity): 0 = NoContact, 1 = Unilateral,
  2 = Bilateral.

### References
- Xiang Y, Arora JS, Rahmatalla S, Abdel-Malek K (2009). Optimization-based
  dynamic human walking prediction: One step formulation. Int J Numer Meth
  Engng 79:667-695. https://doi.org/10.1002/nme.2575
- Dijkstra EJ, Gutierrez-Farewik EM (2015). Computation of ground reaction
  force using Zero Moment Point. J Biomech 48:3776-3781.
  https://doi.org/10.1016/j.jbiomech.2015.08.027
- Karcnik T (2003). Using motion analysis data for foot-floor contact
  detection. Med Biol Eng Comput 41:509-512.
  https://doi.org/10.1007/BF02345310

@author Aaron Fox */
class OSIMSIMULATION_API ZeroMomentPointEstimator : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ZeroMomentPointEstimator, ModelComponent);

public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_LIST_PROPERTY(contact_sides, ContactSide,
            "Limb sides used for contact detection and GRF assignment (one or "
            "two).");

    OpenSim_DECLARE_PROPERTY(free_joint_name, std::string,
            "Name of the joint between Ground and the root body, used to "
            "extract the inverse dynamics residual. Default 'ground_pelvis'.");

    OpenSim_DECLARE_PROPERTY(contact_force_threshold, double,
            "Minimum vertical GRF (N) for outputs to be marked valid. "
            "Default 20 N.");

    OpenSim_DECLARE_PROPERTY(ground_height, double,
            "Height (m) of the horizontal ground plane along the Ground y "
            "axis. Used for the ZMP and for contact detection. Default 0.");

    //==========================================================================
    // OUTPUTS
    //==========================================================================
    OpenSim_DECLARE_LIST_OUTPUT(ground_reaction_force, SimTK::Vec3,
            getGroundReactionForce, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(center_of_pressure, SimTK::Vec3,
            getCenterOfPressure, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(free_moment, SimTK::Vec3, getFreeMoment,
            SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(
            is_in_contact, bool, getIsInContact, SimTK::Stage::Velocity);

    OpenSim_DECLARE_LIST_OUTPUT(
            is_valid, bool, getIsValid, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_OUTPUT(
            contact_mode, int, getContactModeAsInt, SimTK::Stage::Velocity);

    //==========================================================================
    // METHODS
    //==========================================================================
    /** How many ContactSides are in contact. */
    enum class ContactMode {
        NoContact = 0,  ///< No side in contact.
        Unilateral = 1, ///< Exactly one side in contact.
        Bilateral = 2   ///< Both sides in contact (double support).
    };

    ZeroMomentPointEstimator();

    /** Create an estimator that extracts the residual at `freeJointName`. */
    explicit ZeroMomentPointEstimator(const std::string& freeJointName);

    /** Create an estimator with the given root joint and vertical force
    threshold (N). */
    ZeroMomentPointEstimator(
            const std::string& freeJointName, double contactForceThreshold);

    /** @name Contact sides */
    // @{
    /** Take ownership of `side` and append it. An unnamed side is named after
    its contact body. Throws if `side` is null, if two sides already exist,
    or if a side with the same name already exists; ownership is only taken
    if no exception is thrown. Returns the index of the new side. */
    int adoptContactSide(ContactSide* side);

    /** Append a copy of `side`, with the same rules as adoptContactSide().
    Returns the index of the new side. */
    int addContactSide(const ContactSide& side);

    int getNumContactSides() const;
    const ContactSide& getContactSide(int index) const;
    const ContactSide& getContactSide(const std::string& name) const;
    ContactSide& updContactSide(int index);

    /** Index of the ContactSide with the given name (the output channel
    name). Throws if there is none. */
    int getContactSideIndex(const std::string& name) const;
    // @}

    /** @name Property accessors */
    // @{
    const std::string& getFreeJointName() const {
        return get_free_joint_name();
    }
    void setFreeJointName(const std::string& freeJointName);

    double getContactForceThreshold() const {
        return get_contact_force_threshold();
    }
    void setContactForceThreshold(double contactForceThreshold);

    double getGroundHeight() const { return get_ground_height(); }
    void setGroundHeight(double groundHeight);
    // @}

    /** @name Output values
    `side` is the name of a ContactSide. The GRF, CoP, free moment and
    validity require the system to be realized to SimTK::Stage::Acceleration;
    contact status and mode require SimTK::Stage::Velocity. */
    // @{
    SimTK::Vec3 getGroundReactionForce(
            const SimTK::State& state, const std::string& side) const;
    SimTK::Vec3 getCenterOfPressure(
            const SimTK::State& state, const std::string& side) const;
    SimTK::Vec3 getFreeMoment(
            const SimTK::State& state, const std::string& side) const;
    bool getIsInContact(
            const SimTK::State& state, const std::string& side) const;
    bool getIsValid(const SimTK::State& state, const std::string& side) const;
    ContactMode getContactMode(const SimTK::State& state) const;
    int getContactModeAsInt(const SimTK::State& state) const;
    // @}

#ifndef SWIG
    /** Result for one ContactSide. */
    struct SideResult {
        SimTK::Vec3 groundReactionForce{0.0};
        SimTK::Vec3 centerOfPressure{0.0};
        SimTK::Vec3 freeMoment{0.0};
        bool isInContact{false};
        bool isValid{false};
    };

    /** Result for all ContactSides. */
    struct ZMPResult {
        SimTK::Array_<SideResult> sides; ///< Parallel to contact_sides.
        ContactMode mode{ContactMode::NoContact};
    };

    /** Compute the result for all sides using the generalized accelerations
    `udot` (in the system's mobility order, size state.getNU()) instead of
    the accelerations in the state. The result is not cached. Use this when
    the accelerations come from data rather than from the model's dynamics or
    a PositionMotion (see createZMPLoadsFromStates()). The system must be
    realized to SimTK::Stage::Velocity. Not available through scripting. */
    ZMPResult calcZMPResult(
            const SimTK::State& state, const SimTK::Vector& udot) const;
#endif

protected:
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    void constructProperties();

    // Return the cached result (computed with state.getUDot()), computing it
    // if the cache is stale.
    const ZMPResult& getZMPResult(const SimTK::State& state) const;

    // Created in extendConnectToModel(); not copied with the component.
    mutable SimTK::ResetOnCopy<std::unique_ptr<InverseDynamicsSolver>>
            _idSolver;
    SimTK::ReferencePtr<const Joint> _freeJoint;
    mutable CacheVariable<ZMPResult> _zmpResultCV;
};

} // namespace OpenSim

#endif // OPENSIM_ZERO_MOMENT_POINT_ESTIMATOR_H_
