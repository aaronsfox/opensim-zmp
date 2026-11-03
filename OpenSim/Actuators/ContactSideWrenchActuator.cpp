/* -------------------------------------------------------------------------- *
 *                 OpenSim:  ContactSideWrenchActuator.cpp                    *
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

#include "ContactSideWrenchActuator.h"

#include <algorithm>
#include <format>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using SimTK::Vec3;

ContactSideWrenchActuator::ContactSideWrenchActuator() {
    // See the class description: 'gate' is the supported enforcement.
    set_contact_enforcement("gate");
}

ContactSideWrenchActuator::ContactSideWrenchActuator(const ContactSide& side)
        : ContactSideWrenchActuator() {
    setContactSide(side);
}

std::vector<std::pair<double, double>>
ContactSideWrenchActuator::getControlBounds() const {
    return {{0, 1}, {-1, 1}, {-1, 1}, {0, 1}, {0, 1}, {-1, 1}};
}

std::vector<std::string> ContactSideWrenchActuator::getControlLabels() const {
    return {"normal", "friction_x", "friction_z", "cop_fore_aft",
            "cop_medio_lateral", "free_moment"};
}

void ContactSideWrenchActuator::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    const ContactSide& side = getContactSide();
    const PhysicalFrame& body = side.getContactBody();
    const SimTK::Transform X_baseBody = body.findTransformInBaseFrame();
    const Frame& base = body.findBaseFrame();

    const int n = side.getNumContactPoints();
    m_minimum = Vec3(SimTK::Infinity);
    m_maximum = Vec3(-SimTK::Infinity);
    m_soleHeight = 0;
    m_distanceThreshold = 0;
    m_velocityThreshold = 0;
    for (int i = 0; i < n; ++i) {
        const ContactPoint& point = side.getContactPoint(i);
        const PhysicalFrame& frame = point.getParentFrame();
        OPENSIM_THROW_IF_FRMOBJ(&frame.findBaseFrame() != &base, Exception,
                std::format("Expected ContactPoint '{}' to be fixed to the "
                            "contact body '{}', but it is on frame '{}'.",
                        point.getName(), body.getName(), frame.getName()));
        if (i == 0) {
            m_method = point.getContactCheckingMethod();
        } else {
            OPENSIM_THROW_IF_FRMOBJ(
                    point.getContactCheckingMethod() != m_method, Exception,
                    "Expected all ContactPoints of the side to use the same "
                    "contact_checking_method.");
        }
        // Location in the contact body frame.
        const Vec3 location = ~X_baseBody * (frame.findTransformInBaseFrame() *
                                                    point.get_location());
        for (int k = 0; k < 3; ++k) {
            m_minimum[k] = std::min(m_minimum[k], location[k]);
            m_maximum[k] = std::max(m_maximum[k], location[k]);
        }
        m_soleHeight += location[1] / n;
        m_distanceThreshold += point.getDistanceThreshold() / n;
        m_velocityThreshold += point.getVelocityThreshold() / n;
    }
}

Vec3 ContactSideWrenchActuator::calcCenterOfPressureInBody(
        double foreAft, double medioLateral) const {
    return {m_minimum[0] + foreAft * (m_maximum[0] - m_minimum[0]),
            m_soleHeight,
            m_minimum[2] + medioLateral * (m_maximum[2] - m_minimum[2])};
}

double ContactSideWrenchActuator::calcCenterOfPressureGate(
        const SimTK::State& state) const {
    const SimTK::Vector controls = getControls(state);
    return calcGate(state, getContactSide().getContactBody(),
            calcCenterOfPressureInBody(controls[3], controls[4]), m_method,
            m_distanceThreshold, m_velocityThreshold);
}

std::vector<ContactSideActuator::ContactLocation>
ContactSideWrenchActuator::calcContactLocations(
        const SimTK::State& state) const {
    const SimTK::Vector controls = getControls(state);
    ContactLocation location;
    location.name = "center_of_pressure";
    location.load = calcCenterOfPressureGate(state) * getNormalLoad(state);
    calcHeightAndHorizontalSpeed(state, getContactSide().getContactBody(),
            calcCenterOfPressureInBody(controls[3], controls[4]),
            location.height, location.horizontalSpeed);
    location.checksSliding =
            m_method == ContactPoint::ContactCheckingMethod::Velocity;
    location.contactHeight = m_distanceThreshold;
    location.velocityThreshold = m_velocityThreshold;
    return {location};
}

std::vector<ContactSideActuator::ContactLocation>
ContactSideWrenchActuator::getContactLocationInfo() const {
    const ContactSide& side = getContactSide();
    const bool sliding =
            side.getNumContactPoints() > 0 &&
            side.getContactPoint(0).getContactCheckingMethodName() ==
                    "velocity";
    return {{"center_of_pressure", 0, 0, 0, sliding, m_distanceThreshold,
            m_velocityThreshold}};
}

void ContactSideWrenchActuator::calcLoads(const SimTK::State& state,
        std::vector<PointLoad>& loads, Vec3& freeMoment) const {
    const SimTK::Vector controls = getControls(state);
    const PhysicalFrame& body = getContactSide().getContactBody();
    const double mu = get_friction_coefficient();

    const Vec3 copInBody = calcCenterOfPressureInBody(controls[3], controls[4]);
    const double normalForce =
            get_maximum_normal_force() *
            calcGate(state, body, copInBody, m_method, m_distanceThreshold,
                    m_velocityThreshold) *
            getNormalLoad(state);
    const Vec3 pointInGround =
            projectToGround(body.findStationLocationInGround(state, copInBody));
    const Vec3 pointInFrame =
            getModel().getGround().findStationLocationInAnotherFrame(
                    state, pointInGround, body);

    loads.clear();
    loads.push_back({&body, pointInFrame, pointInGround,
            normalForce * Vec3(mu * controls[1], 1.0, mu * controls[2])});
    freeMoment =
            Vec3(0, get_free_moment_radius() * normalForce * controls[5], 0);
}
