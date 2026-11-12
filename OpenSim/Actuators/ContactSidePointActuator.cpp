/* -------------------------------------------------------------------------- *
 *                 OpenSim:  ContactSidePointActuator.cpp                     *
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

#include "ContactSidePointActuator.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using SimTK::Vec3;

namespace {
// Keeps the shares defined when all distribution weights are zero.
constexpr double weightOffset = 1e-3;
} // namespace

ContactSidePointActuator::ContactSidePointActuator() = default;

ContactSidePointActuator::ContactSidePointActuator(const ContactSide& side) {
    setContactSide(side);
}

int ContactSidePointActuator::numControls() const {
    return getContactSide().getNumContactPoints() + 4;
}

std::vector<std::pair<double, double>>
ContactSidePointActuator::getControlBounds() const {
    const int n = getContactSide().getNumContactPoints();
    std::vector<std::pair<double, double>> bounds(n + 1, {0.0, 1.0});
    bounds.insert(bounds.end(), 3, {-1.0, 1.0});
    return bounds;
}

std::vector<std::string> ContactSidePointActuator::getControlLabels() const {
    const ContactSide& side = getContactSide();
    std::vector<std::string> labels{"normal"};
    for (int i = 0; i < side.getNumContactPoints(); ++i) {
        labels.push_back("weight_" + side.getContactPoint(i).getName());
    }
    labels.insert(labels.end(), {"friction_x", "friction_z", "free_moment"});
    return labels;
}

std::array<int, 3> ContactSidePointActuator::getShearControlIndices() const {
    const int n = getContactSide().getNumContactPoints();
    return {n + 1, n + 2, n + 3};
}

std::vector<double> ContactSidePointActuator::calcPointLoads(
        const SimTK::State& state) const {
    const SimTK::Vector controls = getControls(state);
    std::vector<double> loads = calcPointGates(state);
    const int n = static_cast<int>(loads.size());
    const double normal = getNormalLoad(state);
    double totalWeight = 0;
    for (int i = 0; i < n; ++i) totalWeight += controls[i + 1] + weightOffset;
    for (int i = 0; i < n; ++i) {
        loads[i] *= normal * (controls[i + 1] + weightOffset) / totalWeight;
    }
    return loads;
}

std::vector<ContactSideActuator::ContactLocation>
ContactSidePointActuator::calcContactLocations(
        const SimTK::State& state) const {
    const ContactSide& side = getContactSide();
    const std::vector<double> loads = calcPointLoads(state);
    std::vector<ContactLocation> locations;
    for (int i = 0; i < side.getNumContactPoints(); ++i) {
        const ContactPoint& point = side.getContactPoint(i);
        ContactLocation location;
        location.name = point.getName();
        location.load = loads[i];
        calcHeightAndHorizontalSpeed(state, point.getParentFrame(),
                point.get_location(), location.height,
                location.horizontalSpeed);
        location.contactHeight = point.getDistanceThreshold();
        location.velocityThreshold = point.getVelocityThreshold();
        location.checksSliding = point.getContactCheckingMethod() ==
                                 ContactPoint::ContactCheckingMethod::Velocity;
        locations.push_back(location);
    }
    return locations;
}

std::vector<ContactSideActuator::ContactLocation>
ContactSidePointActuator::getContactLocationInfo() const {
    const ContactSide& side = getContactSide();
    std::vector<ContactLocation> locations;
    for (int i = 0; i < side.getNumContactPoints(); ++i) {
        const ContactPoint& point = side.getContactPoint(i);
        locations.push_back({point.getName(), 0, 0, 0,
                point.getContactCheckingMethodName() == "velocity",
                point.getDistanceThreshold(), point.getVelocityThreshold()});
    }
    return locations;
}

void ContactSidePointActuator::calcLoads(const SimTK::State& state,
        std::vector<PointLoad>& loads, Vec3& freeMoment) const {
    const ContactSide& side = getContactSide();
    const int n = side.getNumContactPoints();
    const double maxForce = get_maximum_normal_force();
    const double mu = get_friction_coefficient();
    const Vec3 shear = getShearRatios(state);
    const Vec3 direction(mu * shear[0], 1.0, mu * shear[1]);
    const std::vector<double> pointLoads = calcPointLoads(state);

    loads.clear();
    double normalForce = 0;
    for (int i = 0; i < n; ++i) {
        const ContactPoint& point = side.getContactPoint(i);
        const double load = pointLoads[i];
        const PhysicalFrame& frame = point.getParentFrame();
        const Vec3 pointInGround =
                projectToGround(point.getLocationInGround(state));
        const Vec3 pointInFrame =
                getModel().getGround().findStationLocationInAnotherFrame(
                        state, pointInGround, frame);
        loads.push_back({&frame, pointInFrame, pointInGround,
                maxForce * load * direction});
        normalForce += maxForce * load;
    }
    freeMoment = Vec3(
            0, get_free_moment_radius() * normalForce * shear[2], 0);
}
