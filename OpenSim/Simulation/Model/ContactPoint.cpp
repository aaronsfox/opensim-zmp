/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactPoint.cpp                          *
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

#include "ContactPoint.h"

#include "ContactSide.h"
#include <format>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTION
//=============================================================================
ContactPoint::ContactPoint() : Station() { constructProperties(); }

ContactPoint::ContactPoint(
        const PhysicalFrame& frame, const SimTK::Vec3& location)
        : Station(frame, location) {
    constructProperties();
}

ContactPoint::ContactPoint(const std::string& name, const PhysicalFrame& frame,
        const SimTK::Vec3& location)
        : Station(frame, location) {
    constructProperties();
    setName(name);
}

void ContactPoint::constructProperties() {
    // Defaults are the typical values reported by Karcnik (2003).
    constructProperty_distance_threshold(0.008);
    constructProperty_velocity_threshold(0.015);
    constructProperty_contact_checking_method("distance");
}

//=============================================================================
// ACCESSORS
//=============================================================================
void ContactPoint::setDistanceThreshold(double distanceThreshold) {
    set_distance_threshold(distanceThreshold);
}

void ContactPoint::setVelocityThreshold(double velocityThreshold) {
    set_velocity_threshold(velocityThreshold);
}

void ContactPoint::setContactCheckingMethod(const std::string& method) {
    set_contact_checking_method(method);
}

void ContactPoint::setContactCheckingMethod(ContactCheckingMethod method) {
    set_contact_checking_method(method == ContactCheckingMethod::Velocity
                                        ? "velocity"
                                        : "distance");
}

const std::string& ContactPoint::getParentFrameName() const {
    return getSocket<PhysicalFrame>("parent_frame").getConnecteePath();
}

double ContactPoint::getGroundHeight() const {
    if (hasOwner()) {
        if (const auto* side = dynamic_cast<const ContactSide*>(&getOwner())) {
            return side->getGroundHeight();
        }
    }
    return 0.0;
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void ContactPoint::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    OPENSIM_THROW_IF_FRMOBJ(get_distance_threshold() < 0.0,
            InvalidPropertyValue, getProperty_distance_threshold().getName(),
            std::format("Expected distance_threshold to be non-negative, but "
                        "got {}.",
                    get_distance_threshold()));

    OPENSIM_THROW_IF_FRMOBJ(get_velocity_threshold() < 0.0,
            InvalidPropertyValue, getProperty_velocity_threshold().getName(),
            std::format("Expected velocity_threshold to be non-negative, but "
                        "got {}.",
                    get_velocity_threshold()));

    const std::string& method = get_contact_checking_method();
    if (method == "distance") {
        _contactCheckingMethod = ContactCheckingMethod::Distance;
    } else if (method == "velocity") {
        _contactCheckingMethod = ContactCheckingMethod::Velocity;
    } else {
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue,
                getProperty_contact_checking_method().getName(),
                std::format("Expected contact_checking_method to be "
                            "'distance' or 'velocity', but got '{}'.",
                        method));
    }
}

//=============================================================================
// CONTACT CHECKING
//=============================================================================
double ContactPoint::calcHeightAboveGround(const SimTK::State& state) const {
    return getLocationInGround(state)[1] - getGroundHeight();
}

bool ContactPoint::isInContact(const SimTK::State& state) const {
    // Karcnik (2003), eq. 5: height below z_t and, for the velocity method,
    // the magnitude of the 3D velocity below v_t.
    const bool closeToGround =
            calcHeightAboveGround(state) < get_distance_threshold();
    if (_contactCheckingMethod == ContactCheckingMethod::Distance) {
        return closeToGround;
    }
    return closeToGround &&
           getVelocityInGround(state).norm() < get_velocity_threshold();
}
