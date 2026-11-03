/* -------------------------------------------------------------------------- *
 *                  OpenSim:  MocoContactPenaltyGoal.cpp                      *
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

#include "MocoContactPenaltyGoal.h"

#include <cmath>
#include <format>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

namespace {

// Smooth max(x, 0) with width s, written to avoid overflow.
double softplus(double x, double s) {
    const double y = x / s;
    return s * (std::max(y, 0.0) + std::log1p(std::exp(-std::abs(y))));
}

} // namespace

MocoContactPenaltyGoal::MocoContactPenaltyGoal() { constructProperties(); }

void MocoContactPenaltyGoal::constructProperties() {
    constructProperty_actuator_paths();
    constructProperty_height_weight(1.0);
    constructProperty_sliding_weight(1.0);
    constructProperty_height_scale(0.01);
    constructProperty_speed_scale(0.1);
    constructProperty_height_smoothing(0.001);
}

void MocoContactPenaltyGoal::initializeOnModelImpl(const Model& model) const {
    OPENSIM_THROW_IF_FRMOBJ(get_height_weight() < 0 || get_sliding_weight() < 0,
            Exception,
            "Expected non-negative height_weight and sliding_weight.");
    OPENSIM_THROW_IF_FRMOBJ(!(get_height_scale() > 0) ||
                                    !(get_speed_scale() > 0) ||
                                    !(get_height_smoothing() > 0),
            Exception,
            "Expected positive height_scale, speed_scale and "
            "height_smoothing.");

    m_actuators.clear();
    if (getProperty_actuator_paths().empty()) {
        for (const auto& actuator :
                model.getComponentList<ContactSideActuator>()) {
            m_actuators.emplace_back(&actuator);
        }
    }
    for (int i = 0; i < getProperty_actuator_paths().size(); ++i) {
        const std::string& path = get_actuator_paths(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<ContactSideActuator>(path),
                Exception,
                std::format("No ContactSideActuator at '{}'.", path));
        m_actuators.emplace_back(
                &model.getComponent<ContactSideActuator>(path));
    }
    OPENSIM_THROW_IF_FRMOBJ(m_actuators.empty(), Exception,
            "Expected at least one ContactSideActuator, but found none.");

    // The loads depend on the controls and the speeds.
    setRequirements(1, 1, SimTK::Stage::Velocity);
}

void MocoContactPenaltyGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const double heightWeight = get_height_weight() / get_height_scale();
    const double slidingWeight = get_sliding_weight() / get_speed_scale();
    const double smoothing = get_height_smoothing();
    // The model's controls (and so the loads) are computed when realizing to
    // Velocity.
    getModel().realizeVelocity(input.state);
    integrand = 0;
    for (const auto& actuator : m_actuators) {
        for (const auto& location :
                actuator->calcContactLocations(input.state)) {
            double penalty = heightWeight *
                             softplus(location.height - location.contactHeight,
                                     smoothing);
            if (location.checksSliding) {
                penalty += slidingWeight * location.horizontalSpeed;
            }
            integrand += location.load * penalty;
        }
    }
}

void MocoContactPenaltyGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    // Time average, so the weight does not depend on the duration.
    const double duration = input.final_time - input.initial_time;
    cost[0] = duration > 0 ? input.integral / duration : input.integral;
}

void MocoContactPenaltyGoal::printDescriptionImpl() const {
    std::string names;
    for (const auto& actuator : m_actuators) {
        if (!names.empty()) names += ", ";
        names += actuator->getName();
    }
    log_info("        actuators: {}; height weight {}, sliding weight {}",
            names, get_height_weight(), get_sliding_weight());
}
