/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ContactSideActuator.cpp                       *
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

#include <cmath>
#include <format>

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using SimTK::Vec3;

namespace {

// Logistic function, written to avoid overflow for large |x|.
double logistic(double x) { return 0.5 * (1.0 + std::tanh(0.5 * x)); }

// Speed with a small smoothing term, so its derivative exists at zero.
constexpr double speedSmoothing = 1e-4;

} // namespace

ContactSideActuator::ContactSideActuator() { constructProperties(); }

void ContactSideActuator::constructProperties() {
    constructProperty_maximum_normal_force(2000.0);
    constructProperty_friction_coefficient(0.8);
    constructProperty_free_moment_radius(0.02);
    constructProperty_distance_smoothing(0.002);
    constructProperty_velocity_smoothing(0.005);
    constructProperty_contact_enforcement("constraint");
    constructProperty_load_time_constant(-1);
    constructProperty_load_release_time_constant(-1);
}

const ContactSide& ContactSideActuator::getContactSide() const {
    return getConnectee<ContactSide>("contact_side");
}

void ContactSideActuator::setContactSide(const ContactSide& side) {
    connectSocket_contact_side(side);
}

void ContactSideActuator::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    auto checkPositive = [&](const AbstractProperty& property, double value) {
        OPENSIM_THROW_IF_FRMOBJ(!(value > 0), InvalidPropertyValue,
                property.getName(),
                std::format("Expected a positive value, but got {}.", value));
    };
    checkPositive(
            getProperty_maximum_normal_force(), get_maximum_normal_force());
    checkPositive(getProperty_distance_smoothing(), get_distance_smoothing());
    checkPositive(getProperty_velocity_smoothing(), get_velocity_smoothing());
    OPENSIM_THROW_IF_FRMOBJ(get_friction_coefficient() < 0,
            InvalidPropertyValue, getProperty_friction_coefficient().getName(),
            "Expected a non-negative value.");
    OPENSIM_THROW_IF_FRMOBJ(get_free_moment_radius() < 0, InvalidPropertyValue,
            getProperty_free_moment_radius().getName(),
            "Expected a non-negative value.");
    const std::string& enforcement = get_contact_enforcement();
    OPENSIM_THROW_IF_FRMOBJ(
            enforcement != "constraint" && enforcement != "gate",
            InvalidPropertyValue, getProperty_contact_enforcement().getName(),
            std::format("Expected 'constraint' or 'gate', but got '{}'.",
                    enforcement));
    m_usesGate = enforcement == "gate";
}

//=============================================================================
// LOAD DYNAMICS
//=============================================================================
void ContactSideActuator::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (hasLoadDynamics()) {
        addStateVariable("normal_load", SimTK::Stage::Dynamics);
    }
}

void ContactSideActuator::extendInitStateFromProperties(
        SimTK::State& state) const {
    Super::extendInitStateFromProperties(state);
    if (hasLoadDynamics()) setStateVariableValue(state, "normal_load", 0);
}

void ContactSideActuator::computeStateVariableDerivatives(
        const SimTK::State& state) const {
    if (!hasLoadDynamics()) return;
    // The target is the normal control (index 0 for every concrete class).
    const SimTK::Vector controls = getControls(state);
    const double load = getStateVariableValue(state, "normal_load");
    const double difference = controls[0] - load;
    const double rise = get_load_time_constant();
    double rate = 1.0 / rise;
    if (get_load_release_time_constant() > 0) {
        // Smooth switch between rising and falling time constants.
        const double f = 0.5 + 0.5 * std::tanh(10.0 * difference);
        rate = f / rise + (1 - f) / get_load_release_time_constant();
    }
    setStateVariableDerivativeValue(state, "normal_load", difference * rate);
}

double ContactSideActuator::getNormalLoad(const SimTK::State& state) const {
    if (hasLoadDynamics()) {
        return getStateVariableValue(state, "normal_load");
    }
    const SimTK::Vector controls = getControls(state);
    return controls[0];
}

void ContactSideActuator::setNormalLoad(
        SimTK::State& state, double load) const {
    OPENSIM_THROW_IF_FRMOBJ(!hasLoadDynamics(), Exception,
            "The normal load is a state variable only with a positive "
            "load_time_constant.");
    setStateVariableValue(state, "normal_load", load);
}

//=============================================================================
// GATES
//=============================================================================
Vec3 ContactSideActuator::projectToGround(const Vec3& pointInGround) const {
    return {pointInGround[0], getContactSide().getGroundHeight(),
            pointInGround[2]};
}

void ContactSideActuator::calcHeightAndHorizontalSpeed(
        const SimTK::State& state, const PhysicalFrame& frame,
        const Vec3& pointInFrame, double& height,
        double& horizontalSpeed) const {
    height = frame.findStationLocationInGround(state, pointInFrame)[1] -
             getContactSide().getGroundHeight();
    Vec3 velocity = frame.findStationVelocityInGround(state, pointInFrame);
    velocity[1] = 0;
    horizontalSpeed =
            std::sqrt(velocity.normSqr() + speedSmoothing * speedSmoothing);
}

double ContactSideActuator::calcGate(const SimTK::State& state,
        const PhysicalFrame& frame, const Vec3& pointInFrame,
        ContactPoint::ContactCheckingMethod method, double distanceThreshold,
        double velocityThreshold) const {
    if (!m_usesGate) return 1.0;
    const double height =
            frame.findStationLocationInGround(state, pointInFrame)[1] -
            getContactSide().getGroundHeight();
    double gate =
            logistic((distanceThreshold - height) / get_distance_smoothing());
    if (method == ContactPoint::ContactCheckingMethod::Velocity) {
        // Horizontal speed only (sliding): a point moving into or away from
        // the ground must still be able to carry load, e.g., to stop the foot
        // at impact. Vertical motion is limited by the distance gate and
        // MocoContactPointConstraint.
        Vec3 velocity = frame.findStationVelocityInGround(state, pointInFrame);
        velocity[1] = 0;
        const double speed =
                std::sqrt(velocity.normSqr() + speedSmoothing * speedSmoothing);
        gate *= logistic(
                (velocityThreshold - speed) / get_velocity_smoothing());
    }
    return gate;
}

double ContactSideActuator::calcGate(
        const SimTK::State& state, const ContactPoint& point) const {
    return calcGate(state, point.getParentFrame(), point.get_location(),
            point.getContactCheckingMethod(), point.getDistanceThreshold(),
            point.getVelocityThreshold());
}

std::vector<double> ContactSideActuator::calcPointGates(
        const SimTK::State& state) const {
    const ContactSide& side = getContactSide();
    std::vector<double> gates(side.getNumContactPoints());
    for (int i = 0; i < side.getNumContactPoints(); ++i) {
        gates[i] = calcGate(state, side.getContactPoint(i));
    }
    return gates;
}

//=============================================================================
// LOADS
//=============================================================================
void ContactSideActuator::implProduceForces(
        const SimTK::State& state, ForceConsumer& forceConsumer) const {
    std::vector<PointLoad> loads;
    Vec3 freeMoment(0);
    calcLoads(state, loads, freeMoment);
    for (const auto& load : loads) {
        forceConsumer.consumePointForce(
                state, *load.frame, load.pointInFrame, load.force);
    }
    forceConsumer.consumeTorque(
            state, getContactSide().getContactBody(), freeMoment);
}

Vec3 ContactSideActuator::calcGroundReactionForce(
        const SimTK::State& state) const {
    std::vector<PointLoad> loads;
    Vec3 freeMoment(0);
    calcLoads(state, loads, freeMoment);
    Vec3 force(0);
    for (const auto& load : loads) force += load.force;
    return force;
}

Vec3 ContactSideActuator::calcCenterOfPressure(
        const SimTK::State& state) const {
    // All loads act on the ground plane and their tangential components are
    // proportional to their normal components, so the center of pressure is
    // the normal-force-weighted mean of the application points.
    std::vector<PointLoad> loads;
    Vec3 freeMoment(0);
    calcLoads(state, loads, freeMoment);
    double normal = 0;
    Vec3 weighted(0);
    for (const auto& load : loads) {
        normal += load.force[1];
        weighted += load.force[1] * load.pointInGround;
    }
    if (normal <= 0) return Vec3(0);
    return weighted / normal;
}

Vec3 ContactSideActuator::calcFreeMoment(const SimTK::State& state) const {
    std::vector<PointLoad> loads;
    Vec3 freeMoment(0);
    calcLoads(state, loads, freeMoment);
    return freeMoment;
}

double ContactSideActuator::getPower(const SimTK::State& state) const {
    std::vector<PointLoad> loads;
    Vec3 freeMoment(0);
    calcLoads(state, loads, freeMoment);
    double power = 0;
    for (const auto& load : loads) {
        power += ~load.force * load.frame->findStationVelocityInGround(
                                       state, load.pointInFrame);
    }
    const PhysicalFrame& body = getContactSide().getContactBody();
    power += ~freeMoment * body.getVelocityInGround(state)[0];
    return power;
}
