#ifndef OPENSIM_CONTACT_POINT_H_
#define OPENSIM_CONTACT_POINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ContactPoint.h                           *
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

#include <OpenSim/Simulation/Model/Station.h>

namespace OpenSim {

//=============================================================================
//                                ContactPoint
//=============================================================================
/** A Station fixed to a PhysicalFrame (typically a foot segment) that
detects kinematic contact with a horizontal ground plane.

The point's parent frame and location are given by the `parent_frame` Socket
and `location` property inherited from Station. Place the location on the
contact surface of the segment (e.g., the plantar surface under the heel or
the head of a metatarsal), since contact is assessed from the height of the
point itself.

The ground plane is horizontal (normal along the Ground y axis) at height
`ground_height`. When the ContactPoint is owned by a ContactSide that is in
turn owned by a ZeroMomentPointEstimator, the estimator's `ground_height`
property is used; otherwise the ground plane is at y = 0.

### Contact checking methods
The `contact_checking_method` property selects the criterion:
- `distance`: the point is in contact when its height above the ground plane
  is below `distance_threshold`, i.e.,
  \f$ y - y_{ground} < z_t \f$.
- `velocity`: the combined distance and velocity criterion of Karcnik (2003).
  The point is in contact when it is both close to the ground plane and
  nearly stationary:
  \f$ y - y_{ground} < z_t \f$ and \f$ |\mathbf{v}| < v_t \f$,
  where \f$ |\mathbf{v}| \f$ is the magnitude of the point's 3D velocity in
  Ground. This matches the kinematic contact conditions used by Xiang et al.
  (2009), where contacting points have zero height and zero velocity in all
  three directions, with "exactly zero" replaced by "close to zero".

### Default thresholds
The defaults \f$ z_t \f$ = 0.008 m and \f$ v_t \f$ = 0.015 m/s are the typical
values reported by Karcnik (2003) for markers placed on the lateral side of
the shoe as low as possible, at the heel and at the metatarsal joint. Karcnik
notes the thresholds are highly set-up and subject dependent and were tuned by
trial and error, so treat the defaults as a starting point. For model-based
points (rather than calibrated markers), the appropriate values depend on how
the point is placed relative to the sole, on kinematic noise and filtering,
and on how well the model's foot tracks the floor during stance.

The `velocity` method is recommended for gait. The `distance` method alone
cannot distinguish a stationary stance foot from a swing foot passing close to
the floor (minimum toe clearance in walking is typically only 1-2 cm).

Contact is assessed independently at each state. Detection on its own can
switch on and off rapidly near the thresholds; trajectory-level processing
(e.g., a minimum contact duration) is outside the scope of this component.

### References
- Karcnik T (2003). Using motion analysis data for foot-floor contact
  detection. Med Biol Eng Comput 41:509-512.
  https://doi.org/10.1007/BF02345310
- Xiang Y, Arora JS, Rahmatalla S, Abdel-Malek K (2009). Optimization-based
  dynamic human walking prediction: One step formulation. Int J Numer Meth
  Engng 79:667-695. https://doi.org/10.1002/nme.2575

@author Aaron Fox */
class OSIMSIMULATION_API ContactPoint : public Station {
    OpenSim_DECLARE_CONCRETE_OBJECT(ContactPoint, Station);

public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_PROPERTY(distance_threshold, double,
            "Height (m) above the ground plane below which the point can be "
            "in contact (z_t). Default 0.008 m (Karcnik 2003).");

    OpenSim_DECLARE_PROPERTY(velocity_threshold, double,
            "Magnitude of the point's velocity in Ground (m/s) below which "
            "the point can be in contact (v_t). Only used by the 'velocity' "
            "method. Default 0.015 m/s (Karcnik 2003).");

    OpenSim_DECLARE_PROPERTY(contact_checking_method, std::string,
            "Contact criterion: 'distance' (height only) or 'velocity' "
            "(height and velocity; Karcnik 2003). Default 'distance'.");

    //==========================================================================
    // OUTPUTS
    //==========================================================================
    OpenSim_DECLARE_OUTPUT(
            is_in_contact, bool, isInContact, SimTK::Stage::Velocity);

    //==========================================================================
    // METHODS
    //==========================================================================
    /** The contact criterion, parsed from `contact_checking_method`. */
    enum class ContactCheckingMethod {
        Distance, ///< Height below distance_threshold.
        Velocity  ///< Height and speed below their thresholds.
    };

    ContactPoint();

    /** Create a ContactPoint fixed to `frame` at `location` (expressed in
    `frame`). Unnamed points are given a name when adopted by a ContactSide.
    */
    ContactPoint(const PhysicalFrame& frame, const SimTK::Vec3& location);

    /** Create a named ContactPoint fixed to `frame` at `location` (expressed
    in `frame`). */
    ContactPoint(const std::string& name, const PhysicalFrame& frame,
            const SimTK::Vec3& location);

    /** @name Accessors */
    // @{
    double getDistanceThreshold() const { return get_distance_threshold(); }
    void setDistanceThreshold(double distanceThreshold);

    double getVelocityThreshold() const { return get_velocity_threshold(); }
    void setVelocityThreshold(double velocityThreshold);

    const std::string& getContactCheckingMethodName() const {
        return get_contact_checking_method();
    }
    /** Set the method by name ('distance' or 'velocity'). The name is
    validated when the model is finalized. */
    void setContactCheckingMethod(const std::string& method);
    void setContactCheckingMethod(ContactCheckingMethod method);
    /** The method in use. Only valid after finalizeFromProperties(). */
    ContactCheckingMethod getContactCheckingMethod() const {
        return _contactCheckingMethod;
    }
    /** The connectee path of the `parent_frame` Socket (e.g.,
    "/bodyset/calcn_r"). */
    const std::string& getParentFrameName() const;
    // @}

    /** Height of the ground plane used for contact checking. This is the
    `ground_height` of the owning ZeroMomentPointEstimator (via the owning
    ContactSide), or 0 if the point is not owned by one. */
    double getGroundHeight() const;

    /** Height of the point above the ground plane (m). Negative when the
    point is below the plane. The system must be realized to
    SimTK::Stage::Position. */
    double calcHeightAboveGround(const SimTK::State& state) const;

    /** Whether the point satisfies the contact criterion. The system must be
    realized to SimTK::Stage::Velocity (Stage::Position is sufficient for the
    'distance' method). */
    bool isInContact(const SimTK::State& state) const;

protected:
    void extendFinalizeFromProperties() override;

private:
    void constructProperties();

    ContactCheckingMethod _contactCheckingMethod =
            ContactCheckingMethod::Distance;
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_POINT_H_
