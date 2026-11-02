/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ZeroMomentPointEstimator.cpp                    *
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

#include "ZeroMomentPointEstimator.h"

#include "Model.h"
#include <format>

using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTION
//=============================================================================
ZeroMomentPointEstimator::ZeroMomentPointEstimator() { constructProperties(); }

ZeroMomentPointEstimator::ZeroMomentPointEstimator(
        const std::string& freeJointName) {
    constructProperties();
    set_free_joint_name(freeJointName);
}

ZeroMomentPointEstimator::ZeroMomentPointEstimator(
        const std::string& freeJointName, double contactForceThreshold) {
    constructProperties();
    set_free_joint_name(freeJointName);
    set_contact_force_threshold(contactForceThreshold);
}

void ZeroMomentPointEstimator::constructProperties() {
    constructProperty_contact_sides();
    constructProperty_free_joint_name("ground_pelvis");
    constructProperty_contact_force_threshold(20.0);
    constructProperty_ground_height(0.0);
}

//=============================================================================
// CONTACT SIDES
//=============================================================================
int ZeroMomentPointEstimator::adoptContactSide(ContactSide* side) {
    OPENSIM_THROW_IF_FRMOBJ(side == nullptr, Exception,
            "Expected a ContactSide, but got a null pointer.");

    // Validate before taking ownership (as Component::addComponent does), so
    // the caller (including the Python and Java bindings) still owns `side`
    // if an exception is thrown.
    ContactSide* owned = side;
    const int n = getNumContactSides();

    OPENSIM_THROW_IF_FRMOBJ(n >= 2, Exception,
            "Expected at most 2 ContactSides, but 2 already exist.");

    if (owned->getName().empty()) {
        const auto& socket = owned->getSocket<PhysicalFrame>("contact_body");
        if (socket.isConnected()) {
            owned->setName(socket.getConnectee().getName());
        } else {
            owned->setName("contact_side_" + std::to_string(n + 1));
        }
    }

    for (int i = 0; i < n; ++i) {
        OPENSIM_THROW_IF_FRMOBJ(
                get_contact_sides(i).getName() == owned->getName(), Exception,
                std::format("A ContactSide named '{}' already exists. "
                            "ContactSide names are used as output channel "
                            "names and must be unique.",
                        owned->getName()));
    }

    return updProperty_contact_sides().adoptAndAppendValue(owned);
}

int ZeroMomentPointEstimator::addContactSide(const ContactSide& side) {
    // Copying a component does not copy socket connections made by
    // reference, so reconnect the copy (and its points) to the original's
    // frames.
    std::unique_ptr<ContactSide> copy(side.clone());
    if (side.getSocket<PhysicalFrame>("contact_body").isConnected()) {
        copy->setContactBody(side.getContactBody());
    }
    for (int i = 0; i < side.getNumContactPoints(); ++i) {
        const ContactPoint& point = side.getContactPoint(i);
        if (point.getSocket<PhysicalFrame>("parent_frame").isConnected()) {
            copy->updContactPoint(i).setParentFrame(point.getParentFrame());
        }
    }
    const int index = adoptContactSide(copy.get());
    copy.release();
    return index;
}

int ZeroMomentPointEstimator::getNumContactSides() const {
    return getProperty_contact_sides().size();
}

const ContactSide& ZeroMomentPointEstimator::getContactSide(int index) const {
    return get_contact_sides(index);
}

const ContactSide& ZeroMomentPointEstimator::getContactSide(
        const std::string& name) const {
    return get_contact_sides(getContactSideIndex(name));
}

ContactSide& ZeroMomentPointEstimator::updContactSide(int index) {
    return upd_contact_sides(index);
}

int ZeroMomentPointEstimator::getContactSideIndex(
        const std::string& name) const {
    for (int i = 0; i < getNumContactSides(); ++i) {
        if (get_contact_sides(i).getName() == name) return i;
    }
    OPENSIM_THROW_FRMOBJ(
            Exception, std::format("No ContactSide named '{}'.", name));
}

//=============================================================================
// PROPERTY ACCESSORS
//=============================================================================
void ZeroMomentPointEstimator::setFreeJointName(
        const std::string& freeJointName) {
    set_free_joint_name(freeJointName);
}

void ZeroMomentPointEstimator::setContactForceThreshold(
        double contactForceThreshold) {
    set_contact_force_threshold(contactForceThreshold);
}

void ZeroMomentPointEstimator::setGroundHeight(double groundHeight) {
    set_ground_height(groundHeight);
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void ZeroMomentPointEstimator::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    const int n = getNumContactSides();
    OPENSIM_THROW_IF_FRMOBJ(n == 0 || n > 2, InvalidPropertyValue,
            getProperty_contact_sides().getName(),
            std::format("Expected 1 or 2 ContactSides, but got {}.", n));

    OPENSIM_THROW_IF_FRMOBJ(get_contact_force_threshold() < 0.0,
            InvalidPropertyValue,
            getProperty_contact_force_threshold().getName(),
            std::format("Expected contact_force_threshold to be "
                        "non-negative, but got {}.",
                    get_contact_force_threshold()));

    OPENSIM_THROW_IF_FRMOBJ(!SimTK::isFinite(get_ground_height()),
            InvalidPropertyValue, getProperty_ground_height().getName(),
            "Expected ground_height to be finite.");

    // One output channel per side, named after the side. Clear first so that
    // renamed or removed sides do not leave stale channels.
    for (const std::string& output :
            {"ground_reaction_force", "center_of_pressure", "free_moment",
                    "is_in_contact", "is_valid"}) {
        updOutput(output).clearChannels();
        for (int i = 0; i < n; ++i) {
            updOutput(output).addChannel(get_contact_sides(i).getName());
        }
    }
}

void ZeroMomentPointEstimator::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Model connects all joints and their frames before other components, so
    // the joint's parent frame can be resolved here.
    const std::string& jointName = get_free_joint_name();
    OPENSIM_THROW_IF_FRMOBJ(!model.getJointSet().contains(jointName), Exception,
            std::format("free_joint_name '{}' was not found in the model's "
                        "JointSet.",
                    jointName));
    const Joint& joint = model.getJointSet().get(jointName);

    OPENSIM_THROW_IF_FRMOBJ(
            &joint.getParentFrame().findBaseFrame() != &model.getGround(),
            Exception,
            std::format("Expected the parent frame of free joint '{}' to be "
                        "Ground (or a frame fixed to Ground), but it is "
                        "'{}'.",
                    jointName, joint.getParentFrame().getAbsolutePathString()));

    OPENSIM_THROW_IF_FRMOBJ(joint.numCoordinates() == 0, Exception,
            std::format("Free joint '{}' has no coordinates, so it has no "
                        "inverse dynamics residual.",
                    jointName));

    _freeJoint.reset(&joint);
    _idSolver.reset(new InverseDynamicsSolver(model));
}

void ZeroMomentPointEstimator::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    // The result depends on the accelerations (udot).
    _zmpResultCV = addCacheVariable(
            "zmp_result", ZMPResult(), SimTK::Stage::Acceleration);
}

//=============================================================================
// OUTPUT VALUES
//=============================================================================
Vec3 ZeroMomentPointEstimator::getGroundReactionForce(
        const SimTK::State& state, const std::string& side) const {
    return getZMPResult(state)
            .sides[getContactSideIndex(side)]
            .groundReactionForce;
}

Vec3 ZeroMomentPointEstimator::getCenterOfPressure(
        const SimTK::State& state, const std::string& side) const {
    return getZMPResult(state)
            .sides[getContactSideIndex(side)]
            .centerOfPressure;
}

Vec3 ZeroMomentPointEstimator::getFreeMoment(
        const SimTK::State& state, const std::string& side) const {
    return getZMPResult(state).sides[getContactSideIndex(side)].freeMoment;
}

bool ZeroMomentPointEstimator::getIsInContact(
        const SimTK::State& state, const std::string& side) const {
    // Velocity-stage quantity: evaluate directly rather than via the
    // Acceleration-stage cache.
    return get_contact_sides(getContactSideIndex(side)).isInContact(state);
}

bool ZeroMomentPointEstimator::getIsValid(
        const SimTK::State& state, const std::string& side) const {
    return getZMPResult(state).sides[getContactSideIndex(side)].isValid;
}

ZeroMomentPointEstimator::ContactMode ZeroMomentPointEstimator::getContactMode(
        const SimTK::State& state) const {
    int numInContact = 0;
    for (int i = 0; i < getNumContactSides(); ++i) {
        if (get_contact_sides(i).isInContact(state)) ++numInContact;
    }
    if (numInContact == 0) return ContactMode::NoContact;
    if (numInContact == 1) return ContactMode::Unilateral;
    return ContactMode::Bilateral;
}

int ZeroMomentPointEstimator::getContactModeAsInt(
        const SimTK::State& state) const {
    return static_cast<int>(getContactMode(state));
}

//=============================================================================
// COMPUTATION
//=============================================================================
const ZeroMomentPointEstimator::ZMPResult&
ZeroMomentPointEstimator::getZMPResult(const SimTK::State& state) const {
    if (!isCacheVariableValid(state, _zmpResultCV)) {
        // Outputs check the stage themselves; this guards direct calls.
        OPENSIM_THROW_IF_FRMOBJ(
                state.getSystemStage() < SimTK::Stage::Acceleration, Exception,
                std::format("Expected the system to be realized to "
                            "Stage::Acceleration, but it is realized to "
                            "Stage::{}.",
                        std::string(state.getSystemStage().getName())));
        setCacheVariableValue(
                state, _zmpResultCV, calcZMPResult(state, state.getUDot()));
        markCacheVariableValid(state, _zmpResultCV);
    }
    return getCacheVariableValue(state, _zmpResultCV);
}

ZeroMomentPointEstimator::ZMPResult ZeroMomentPointEstimator::calcZMPResult(
        const SimTK::State& state, const SimTK::Vector& udot) const {
    OPENSIM_THROW_IF_FRMOBJ(state.getSystemStage() < SimTK::Stage::Velocity,
            Exception,
            std::format("Expected the system to be realized to "
                        "Stage::Velocity, but it is realized to Stage::{}.",
                    std::string(state.getSystemStage().getName())));
    OPENSIM_THROW_IF_FRMOBJ(udot.size() != state.getNU(), Exception,
            std::format("Expected udot to have {} elements, but it has {}.",
                    state.getNU(), udot.size()));

    const int n = getNumContactSides();
    ZMPResult result;
    result.sides.resize(n, SideResult{});

    // Contact detection.
    int numInContact = 0;
    int sideInContact = -1;
    for (int i = 0; i < n; ++i) {
        const bool inContact = get_contact_sides(i).isInContact(state);
        result.sides[i].isInContact = inContact;
        if (inContact) {
            ++numInContact;
            sideInContact = i;
        }
    }
    if (numInContact == 0) {
        result.mode = ContactMode::NoContact;
        return result;
    }
    result.mode = numInContact == 1 ? ContactMode::Unilateral
                                    : ContactMode::Bilateral;

    // TODO: split the GRF between sides in double support. Until a bilateral
    // method is implemented, outputs remain zero and not valid.
    // createZMPLoadsFromStates() reports these periods to the user.
    if (result.mode == ContactMode::Bilateral) return result;

    // Whole-body external wrench: inverse dynamics with gravity as the only
    // applied force, so that the model's own forces (muscles, actuators,
    // contact elements, ExternalLoads) do not cancel the residual.
    const Model& model = getModel();
    const SimTK::Vector noMobilityForces(state.getNU(), 0.0);
    const SimTK::Vector tau = _idSolver->solve(
            state, udot, noMobilityForces, model.getGravityBodyForces(state));

    // Equivalent spatial force on the root body at the joint's child frame
    // origin, expressed in Ground: [moment, force].
    const Joint& joint = *_freeJoint;
    const SimTK::SpatialVec wrench =
            joint.calcEquivalentSpatialForce(state, tau);
    const Vec3& force = wrench[1];

    if (force[1] < get_contact_force_threshold()) return result;

    // Shift the moment to the Ground origin (Xiang et al. 2009, eq. 16).
    const Vec3 r = joint.getChildFrame().getPositionInGround(state);
    const Vec3 moment = wrench[0] + SimTK::cross(r, force);

    // ZMP on the plane y = h (Xiang et al. 2009, eq. 19, generalized to
    // h != 0), and the free moment about it. Xiang's eq. 20 has the opposite
    // sign on the bracketed terms; this form is consistent with eq. 19.
    const double h = get_ground_height();
    const double copX = (moment[2] + h * force[0]) / force[1];
    const double copZ = (h * force[2] - moment[0]) / force[1];
    const double freeMomentY = moment[1] - (copZ * force[0] - copX * force[2]);

    SideResult& side = result.sides[sideInContact];
    side.groundReactionForce = force;
    side.centerOfPressure = Vec3(copX, h, copZ);
    side.freeMoment = Vec3(0.0, freeMomentY, 0.0);
    side.isValid = true;
    return result;
}
