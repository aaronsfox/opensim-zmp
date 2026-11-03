/* -------------------------------------------------------------------------- *
 *                 OpenSim:  MocoContactPointConstraint.cpp                   *
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

#include "MocoContactPointConstraint.h"

#include <algorithm>
#include <format>

#include <OpenSim/Actuators/ContactSideActuator.h>
#include <OpenSim/Actuators/ContactSideWrenchActuator.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoContactPointConstraint::MocoContactPointConstraint() {
    constructProperties();
}

void MocoContactPointConstraint::constructProperties() {
    constructProperty_contact_point_paths();
    constructProperty_contact_side_paths();
    constructProperty_penetration_tolerance(0.0);
    constructProperty_enforce_contact_complementarity(false);
    constructProperty_complementarity_tolerance(3e-4);
}

void MocoContactPointConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo&) const {
    OPENSIM_THROW_IF_FRMOBJ(get_penetration_tolerance() < 0, Exception,
            std::format("Expected penetration_tolerance to be non-negative, "
                        "but got {}.",
                    get_penetration_tolerance()));

    // Resolve the points once; evaluating the constraint then needs no path
    // lookups.
    m_points.clear();
    auto add = [&](const ContactPoint& point) {
        const bool listed = std::any_of(m_points.begin(), m_points.end(),
                [&](const auto& p) { return p.get() == &point; });
        if (!listed) m_points.emplace_back(&point);
    };

    const int numPointPaths = getProperty_contact_point_paths().size();
    const int numSidePaths = getProperty_contact_side_paths().size();
    if (numPointPaths == 0 && numSidePaths == 0) {
        for (const auto& point : model.getComponentList<ContactPoint>()) {
            add(point);
        }
        OPENSIM_THROW_IF_FRMOBJ(m_points.empty(), Exception,
                "Expected the model to contain at least one ContactPoint, "
                "but it has none.");
    }
    for (int i = 0; i < numPointPaths; ++i) {
        const std::string& path = get_contact_point_paths(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<ContactPoint>(path),
                Exception, std::format("No ContactPoint at '{}'.", path));
        add(model.getComponent<ContactPoint>(path));
    }
    for (int i = 0; i < numSidePaths; ++i) {
        const std::string& path = get_contact_side_paths(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<ContactSide>(path),
                Exception, std::format("No ContactSide at '{}'.", path));
        const auto& side = model.getComponent<ContactSide>(path);
        for (int j = 0; j < side.getNumContactPoints(); ++j) {
            add(side.getContactPoint(j));
        }
    }

    // One equation per point: height above ground >= -tolerance. Label each
    // equation "<constraint name>_<suffix>".
    std::vector<MocoBounds> bounds(m_points.size(),
            MocoBounds(-get_penetration_tolerance(), SimTK::Infinity));
    std::vector<std::string> suffixes;
    for (const auto& point : m_points) suffixes.push_back(point->getName());

    // Complementarity: for each ContactSideActuator on a side of a
    // constrained point, load * (height - band) <= epsilon at each contact
    // location. (Sliding is penalized by MocoContactPenaltyGoal instead.)
    m_actuators.clear();
    if (get_enforce_contact_complementarity()) {
        OPENSIM_THROW_IF_FRMOBJ(!(get_complementarity_tolerance() > 0),
                Exception,
                "Expected complementarity_tolerance to be positive.");
        for (const auto& actuator :
                model.getComponentList<ContactSideActuator>()) {
            const ContactSide& side = actuator.getContactSide();
            const bool constrained = std::any_of(
                    m_points.begin(), m_points.end(), [&](const auto& point) {
                        return &point->getOwner() == &side;
                    });
            if (!constrained) continue;
            m_actuators.emplace_back(&actuator);
            for (const auto& location : actuator.getContactLocationInfo()) {
                const std::string prefix =
                        actuator.getName() + "_" + location.name;
                bounds.emplace_back(
                        -SimTK::Infinity, get_complementarity_tolerance());
                suffixes.push_back(prefix + "_height");
            }
        }
        OPENSIM_THROW_IF_FRMOBJ(m_actuators.empty(), Exception,
                "enforce_contact_complementarity is true, but no "
                "ContactSideActuator acts on the constrained ContactPoints' "
                "sides.");
        for (const auto& actuator : m_actuators) {
            if (actuator->usesGate()) {
                log_warn(
                        "MocoContactPointConstraint '{}': ContactSideActuator "
                        "'{}' uses 'gate' contact_enforcement; complementarity "
                        "is usually used with 'constraint' enforcement.",
                        getName(), actuator->getName());
            } else if (dynamic_cast<const ContactSideWrenchActuator*>(
                               actuator.get())) {
                log_warn("MocoContactPointConstraint '{}': complementarity "
                         "rarely converges for ContactSideWrenchActuator "
                         "'{}'; use its 'gate' contact_enforcement (with a "
                         "guess near the ground) or "
                         "ContactSidePointActuator.",
                        getName(), actuator->getName());
            }
        }
    }

    MocoConstraintInfo info = getConstraintInfo();
    info.setName(getName());
    info.setBounds(bounds);
    info.setSuffixes(suffixes);
    setNumEquations(static_cast<int>(bounds.size()));
    const_cast<MocoContactPointConstraint*>(this)->setConstraintInfo(info);
}

void MocoContactPointConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    // Moco realizes the state to Stage::Position before calling this.
    int index = 0;
    for (const auto& point : m_points) {
        errors[index++] = point->calcHeightAboveGround(state);
    }
    if (m_actuators.empty()) return;

    // Loads depend on the controls.
    getModel().realizeVelocity(state);
    for (const auto& actuator : m_actuators) {
        for (const auto& location : actuator->calcContactLocations(state)) {
            // Load is only allowed within the contact band.
            errors[index++] =
                    location.load * (location.height - location.contactHeight);
        }
    }
}

void MocoContactPointConstraint::printDescriptionImpl() const {
    std::string names;
    for (const auto& point : m_points) {
        if (!names.empty()) names += ", ";
        names += point->getName();
    }
    log_info("        contact points: {}", names.empty() ? "(all)" : names);
    log_info(
            "        penetration tolerance: {} m", get_penetration_tolerance());
    if (get_enforce_contact_complementarity()) {
        std::string actuators;
        for (const auto& actuator : m_actuators) {
            if (!actuators.empty()) actuators += ", ";
            actuators += actuator->getName();
        }
        log_info("        contact complementarity: actuators {}; tolerance {}",
                actuators, get_complementarity_tolerance());
    }
}
