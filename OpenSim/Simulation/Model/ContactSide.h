#ifndef OPENSIM_CONTACT_SIDE_H_
#define OPENSIM_CONTACT_SIDE_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ContactSide.h                           *
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

#include <vector>

#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//=============================================================================
//                                ContactSide
//=============================================================================
/** A limb side (e.g., the right or left foot) that groups the ContactPoints
used to detect ground contact and identifies the body to which the ground
reaction for that side is applied.

A ContactSide is in contact when ANY of its ContactPoints is in contact. Its
name is used as the output channel name by ZeroMomentPointEstimator (e.g.,
"right" and "left", or "calcn_r" and "calcn_l").

### Properties
- `contact_points`: the ContactPoints owned by this side (at least one).
- `minimum_contact_duration`: see below.

### Sockets
- `contact_body`: the PhysicalFrame to which the ground reaction for this side
  is applied (e.g., /bodyset/calcn_r).

### Outputs
- `is_in_contact` (bool, Stage::Velocity): true if any ContactPoint is in
  contact.

### Support polygon
The support polygon is the convex hull of the active (in-contact) ContactPoint
locations projected onto the ground plane. It is computed in the horizontal
(x, z) plane of Ground; vertices are returned at the ground height and in
counter-clockwise order in the (x, z) coordinate plane. With one active point
the polygon is that point, and with two it is the segment between them.

### Minimum contact duration (optional)
When `minimum_contact_duration` (s) is set, periods of contact or of no
contact that are shorter than it are removed: each such period takes the
contact status of the periods around it (shortest periods first). Periods at
the start or end of a trial are kept, since the trial may cut them short. This
is off by default (-1). Like ContactPoint hysteresis, it needs the whole
trajectory, so it is applied by createZMPLoadsFromStates(), after contact has
been detected for every frame; the `is_in_contact` output assesses each state
on its own and does not use it.

### Naming
Unnamed sides adopted by ZeroMomentPointEstimator::adoptContactSide() are
named after their contact body. Unnamed points adopted by
adoptContactPoint() are named `<side name>_point_<n>`. Adopting a point
whose name matches an existing point throws an exception.

@author Aaron Fox */
class OSIMSIMULATION_API ContactSide : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ContactSide, ModelComponent);

public:
    //==========================================================================
    // PROPERTIES
    //==========================================================================
    OpenSim_DECLARE_LIST_PROPERTY(contact_points, ContactPoint,
            "Contact detection points belonging to this side.");

    OpenSim_DECLARE_PROPERTY(minimum_contact_duration, double,
            "Periods of contact or no contact shorter than this (s) are "
            "removed by createZMPLoadsFromStates(). Default -1 (unused).");

    //==========================================================================
    // SOCKETS
    //==========================================================================
    OpenSim_DECLARE_SOCKET(contact_body, PhysicalFrame,
            "The frame to which the ground reaction for this side is applied "
            "(e.g., /bodyset/calcn_r).");

    //==========================================================================
    // OUTPUTS
    //==========================================================================
    OpenSim_DECLARE_OUTPUT(
            is_in_contact, bool, isInContact, SimTK::Stage::Velocity);

    //==========================================================================
    // METHODS
    //==========================================================================
    ContactSide();

    /** Create a side whose ground reaction is applied to `contactBody`. The
    side is named after the body. */
    explicit ContactSide(const PhysicalFrame& contactBody);

    /** Create a named side whose ground reaction is applied to
    `contactBody`. */
    ContactSide(const std::string& name, const PhysicalFrame& contactBody);

    /** @name Contact points */
    // @{
    /** Take ownership of `point` and append it to this side. If the point is
    unnamed, it is named `<side name>_point_<n>`. Throws if `point` is null
    or if a point with the same name already exists; ownership is only taken
    if no exception is thrown. Returns the index of the new point. */
    int adoptContactPoint(ContactPoint* point);

    /** Append a copy of `point` to this side, with the same naming rules as
    adoptContactPoint(). Returns the index of the new point. */
    int addContactPoint(const ContactPoint& point);

    int getNumContactPoints() const;
    const ContactPoint& getContactPoint(int index) const;
    const ContactPoint& getContactPoint(const std::string& name) const;
    ContactPoint& updContactPoint(int index);
    // @}

    /** Minimum duration (s) of a contact or no-contact period; -1 means
    unused. */
    double getMinimumContactDuration() const {
        return get_minimum_contact_duration();
    }
    void setMinimumContactDuration(double duration);

    /** @name Contact body */
    // @{
    const PhysicalFrame& getContactBody() const;
    void setContactBody(const PhysicalFrame& contactBody);
    // @}

    /** Height of the ground plane: the `ground_height` of the owning
    ZeroMomentPointEstimator, or 0 if this side is not owned by one. */
    double getGroundHeight() const;

    /** True if any ContactPoint is in contact. The system must be realized
    to SimTK::Stage::Velocity. */
    bool isInContact(const SimTK::State& state) const;

    /** @name Support polygon
    These require the system to be realized to SimTK::Stage::Velocity. */
    // @{
    /** Number of ContactPoints that are in contact. */
    int calcNumActiveContactPoints(const SimTK::State& state) const;

    /** Locations in Ground of the ContactPoints that are in contact,
    projected onto the ground plane (y = ground height). */
    std::vector<SimTK::Vec3> calcActiveContactPointLocationsInGround(
            const SimTK::State& state) const;

    /** Mean of the active, ground-projected ContactPoint locations. Returns a
    Vec3 of NaN when no point is in contact. */
    SimTK::Vec3 calcActiveContactPointCentroidInGround(
            const SimTK::State& state) const;

    /** Vertices of the support polygon (see class description). Empty when
    no point is in contact. */
    std::vector<SimTK::Vec3> calcSupportPolygonInGround(
            const SimTK::State& state) const;

    /** Horizontal (x, z) distance from `point` (expressed in Ground) to the
    boundary of the support polygon: negative inside, positive outside. With
    one or two active points (a point or a segment), the distance is never
    negative. Returns NaN when no point is in contact. */
    double calcSignedDistanceToSupportPolygon(
            const SimTK::State& state, const SimTK::Vec3& point) const;

    /** Whether `point` (expressed in Ground) lies inside the support polygon
    or within `tolerance` (m) of it, measured in the horizontal (x, z)
    plane. Returns false when no point is in contact. */
    bool isPointInSupportPolygon(const SimTK::State& state,
            const SimTK::Vec3& point, double tolerance = 0.0) const;
    // @}

protected:
    void extendFinalizeFromProperties() override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_SIDE_H_
