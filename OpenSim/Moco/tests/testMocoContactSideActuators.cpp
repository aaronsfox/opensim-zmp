/* -------------------------------------------------------------------------- *
 *               OpenSim:  testMocoContactSideActuators.cpp                   *
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

// Tests for ContactSidePointActuator and ContactSideWrenchActuator. Most cases
// evaluate the actuators directly; the "[casadi]" cases solve a predictive
// landing problem with MocoCasADiSolver and are only compiled when OpenSim is
// built with CasADi.

#include <OpenSim/Actuators/ContactSidePointActuator.h>
#include <OpenSim/Actuators/ContactSideWrenchActuator.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Moco/MocoContactPointConstraint.h>
#include <OpenSim/Moco/MocoGoal/MocoContactPenaltyGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoControlGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>
#include <OpenSim/Moco/MocoGoal/MocoStateTrackingGoal.h>
#include <OpenSim/Moco/MocoProblemRep.h>
#include <OpenSim/Moco/MocoStudy.h>
#include <OpenSim/Moco/MocoTrajectory.h>
#include <OpenSim/Moco/MocoUtilities.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>

#ifdef OPENSIM_WITH_CASADI
#    include <OpenSim/Moco/MocoCasADiSolver/MocoCasADiSolver.h>
#endif

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace OpenSim;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using SimTK::Vec3;

namespace {

constexpr double mass = 10.0;
constexpr double halfHeight = 0.05;
constexpr double halfLength = 0.1;
constexpr double halfWidth = 0.04;

// A rigid planar "foot" (Moco does not support FreeJoint) with four
// ContactPoints at the corners of its sole, owned by a ZeroMomentPointEstimator
// side named "right", and an actuator of type T at "/forceset/contact".
template <typename T>
Model createFootModel(const std::string& contactCheckingMethod = "distance",
        const std::string& contactEnforcement = "gate") {
    Model model;
    model.setName("foot");
    auto* foot = new Body("foot", mass, Vec3(0), SimTK::Inertia(0.05));
    model.addBody(foot);
    auto* joint = new PlanarJoint("ground_foot", model.getGround(), *foot);
    joint->upd_coordinates(0).setName("rz");
    joint->upd_coordinates(1).setName("tx");
    joint->upd_coordinates(2).setName("ty");
    model.addJoint(joint);

    auto* zmp = new ZeroMomentPointEstimator("ground_foot");
    zmp->setName("zmp");
    auto* side = new ContactSide("right", *foot);
    for (const Vec3& corner : {Vec3(-halfLength, -halfHeight, -halfWidth),
                 Vec3(-halfLength, -halfHeight, halfWidth),
                 Vec3(halfLength, -halfHeight, -halfWidth),
                 Vec3(halfLength, -halfHeight, halfWidth)}) {
        auto* point = new ContactPoint(*foot, corner);
        point->setContactCheckingMethod(contactCheckingMethod);
        side->adoptContactPoint(point);
    }
    zmp->adoptContactSide(side);
    model.addComponent(zmp);
    model.finalizeConnections();

    auto* actuator = new T(model.getComponent<ContactSide>("/zmp/right"));
    actuator->setName("contact");
    actuator->set_contact_enforcement(contactEnforcement);
    model.addForce(actuator);
    model.finalizeConnections();
    return model;
}

// Set the foot pose and the actuator's controls, and realize to Velocity.
void setFoot(const Model& model, SimTK::State& state, double ty,
        const std::vector<double>& actuatorControls, double rz = 0,
        double tyDot = 0) {
    model.getCoordinateSet().get("ty").setValue(state, ty, false);
    model.getCoordinateSet().get("rz").setValue(state, rz, false);
    model.getCoordinateSet().get("tx").setValue(state, 0, false);
    model.getCoordinateSet().get("ty").setSpeedValue(state, tyDot);
    const auto& actuator =
            model.getComponent<ContactSideActuator>("/forceset/contact");
    SimTK::Vector controls = model.getDefaultControls();
    SimTK::Vector values(static_cast<int>(actuatorControls.size()));
    for (int i = 0; i < values.size(); ++i) values[i] = actuatorControls[i];
    actuator.setControls(values, controls);
    model.realizePosition(state);
    model.setControls(state, controls);
    model.realizeVelocity(state);
}

double logistic(double x) { return 1.0 / (1.0 + std::exp(-x)); }

void checkVec3(const Vec3& actual, const Vec3& expected, double tolerance) {
    for (int k = 0; k < 3; ++k) {
        CHECK_THAT(actual[k], WithinAbs(expected[k], tolerance));
    }
}

} // namespace

TEST_CASE("ContactSidePointActuator loads") {
    Model model = createFootModel<ContactSidePointActuator>();
    SimTK::State& state = model.initSystem();
    const auto& actuator =
            model.getComponent<ContactSidePointActuator>("/forceset/contact");
    const double maxForce = actuator.get_maximum_normal_force();
    const double mu = actuator.get_friction_coefficient();
    const double radius = actuator.get_free_moment_radius();

    REQUIRE(actuator.numControls() == 8);
    CHECK(actuator.getControlLabels()[0] == "normal");
    CHECK(actuator.getControlLabels()[1] == "weight_right_point_1");
    CHECK(actuator.getControlLabels()[5] == "friction_x");
    CHECK(actuator.getControlBounds()[0].first == 0);
    CHECK(actuator.getControlBounds()[7].first == -1);

    SECTION("Flat on the ground: loads, center of pressure, free moment") {
        // 80% of the maximum load; heel points weighted less than toe points.
        setFoot(model, state, halfHeight,
                {0.8, 0.2, 0.2, 0.6, 0.6, 0.5, 0.0, 0.5});
        const double gate = logistic(0.008 / 0.002);
        for (const double g : actuator.calcPointGates(state)) {
            CHECK_THAT(g, WithinRel(gate, 1e-6));
        }
        // Shares (w + 1e-3) / sum(w + 1e-3) sum to 1.
        const double normal = maxForce * gate * 0.8;
        checkVec3(actuator.calcGroundReactionForce(state),
                Vec3(mu * normal * 0.5, normal, 0), 1e-8);
        // CoP x: share-weighted mean of -0.1 (heel) and 0.1 (toe).
        const double heel = 2 * (0.2 + 1e-3) / (1.6 + 4e-3);
        checkVec3(actuator.calcCenterOfPressure(state),
                Vec3(heel * -halfLength + (1 - heel) * halfLength, 0, 0),
                1e-10);
        checkVec3(actuator.calcFreeMoment(state),
                Vec3(0, radius * normal * 0.5, 0), 1e-8);
    }

    SECTION("Points off the ground carry no load") {
        // Rotate toe-up about the origin: toe points rise, heel points sink.
        const double rz = 0.3;
        // Height that keeps the heel corners on the ground.
        const double ty = halfHeight * std::cos(rz) + halfLength * std::sin(rz);
        setFoot(model, state, ty, {1, 1, 1, 1, 1, 0, 0, 0}, rz);
        const auto gates = actuator.calcPointGates(state);
        CHECK(gates[0] > 0.9);  // Heel.
        CHECK(gates[2] < 1e-6); // Toe.
        // So the center of pressure is at the heel.
        CHECK_THAT(actuator.calcCenterOfPressure(state)[0],
                WithinAbs(
                        -halfLength * std::cos(rz) + halfHeight * std::sin(rz),
                        1e-4));
    }

    SECTION("In the air: no load") {
        setFoot(model, state, halfHeight + 0.05, {1, 1, 1, 1, 1, 0, 0, 0});
        CHECK(actuator.calcGroundReactionForce(state)[1] < 1e-3);
    }

    SECTION("Velocity method: sliding points carry no load") {
        Model velocityModel =
                createFootModel<ContactSidePointActuator>("velocity");
        SimTK::State& s = velocityModel.initSystem();
        const auto& velocityActuator =
                velocityModel.getComponent<ContactSidePointActuator>(
                        "/forceset/contact");
        setFoot(velocityModel, s, halfHeight, {1, 1, 1, 1, 1, 0, 0, 0}, 0, 0);
        const double still = velocityActuator.calcGroundReactionForce(s)[1];
        // Moving down (e.g., at impact) is not gated.
        setFoot(velocityModel, s, halfHeight, {1, 1, 1, 1, 1, 0, 0, 0}, 0,
                -0.1);
        const double movingDown =
                velocityActuator.calcGroundReactionForce(s)[1];
        // Sliding is. (Set speeds before the controls: changing the state
        // after setting the controls resets them.)
        velocityModel.getCoordinateSet().get("tx").setSpeedValue(s, 0.1);
        setFoot(velocityModel, s, halfHeight, {1, 1, 1, 1, 1, 0, 0, 0}, 0, 0);
        const double sliding = velocityActuator.calcGroundReactionForce(s)[1];
        CHECK(still > 0.9 * maxForce * logistic(4));
        CHECK_THAT(movingDown, WithinRel(still, 1e-6));
        CHECK(sliding < 1e-6 * still);
    }
}

TEST_CASE("ContactSideWrenchActuator loads") {
    Model model = createFootModel<ContactSideWrenchActuator>();
    SimTK::State& state = model.initSystem();
    const auto& actuator =
            model.getComponent<ContactSideWrenchActuator>("/forceset/contact");
    const double maxForce = actuator.get_maximum_normal_force();
    const double mu = actuator.get_friction_coefficient();

    REQUIRE(actuator.numControls() == 6);
    checkVec3(actuator.calcCenterOfPressureInBody(0, 0),
            Vec3(-halfLength, -halfHeight, -halfWidth), 1e-12);
    checkVec3(actuator.calcCenterOfPressureInBody(1, 0.5),
            Vec3(halfLength, -halfHeight, 0), 1e-12);

    SECTION("Flat on the ground") {
        setFoot(model, state, halfHeight, {0.5, -0.5, 0, 0.75, 0.5, -1});
        const double normal = maxForce * logistic(4) * 0.5;
        CHECK_THAT(actuator.calcCenterOfPressureGate(state),
                WithinRel(logistic(4), 1e-6));
        checkVec3(actuator.calcGroundReactionForce(state),
                Vec3(-mu * normal * 0.5, normal, 0), 1e-8);
        checkVec3(actuator.calcCenterOfPressure(state),
                Vec3(0.5 * halfLength, 0, 0), 1e-12);
        checkVec3(actuator.calcFreeMoment(state),
                Vec3(0, -actuator.get_free_moment_radius() * normal, 0), 1e-8);
    }

    SECTION("Center of pressure under an airborne part of the sole") {
        const double rz = 0.3; // Toe up.
        // Height that keeps the heel corners on the ground.
        const double ty = halfHeight * std::cos(rz) + halfLength * std::sin(rz);
        setFoot(model, state, ty, {1, 0, 0, 0, 0.5, 0}, rz);
        const double atHeel = actuator.calcGroundReactionForce(state)[1];
        setFoot(model, state, ty, {1, 0, 0, 1, 0.5, 0}, rz);
        const double atToe = actuator.calcGroundReactionForce(state)[1];
        CHECK(atHeel > 0.9 * maxForce);
        CHECK(atToe < 1e-6);
    }
}

TEST_CASE("ContactSideActuator validation") {
    SECTION("Wrench actuator: points must be on the contact body") {
        Model model = createFootModel<ContactSideWrenchActuator>();
        auto* toe = new Body("toe", 0.1, Vec3(0), SimTK::Inertia(0.001));
        model.addBody(toe);
        model.addJoint(new PlanarJoint("ground_toe", model.getGround(), *toe));
        model.updComponent<ContactSide>("/zmp/right")
                .adoptContactPoint(new ContactPoint(*toe, Vec3(0)));
        CHECK_THROWS([&] {
            model.finalizeConnections();
            model.initSystem();
        }());
    }

    SECTION("Wrench actuator: one contact_checking_method") {
        Model model = createFootModel<ContactSideWrenchActuator>();
        model.updComponent<ContactSide>("/zmp/right")
                .updContactPoint(0)
                .setContactCheckingMethod("velocity");
        CHECK_THROWS(model.initSystem());
    }

    SECTION("Positive maximum force and smoothing") {
        Model model = createFootModel<ContactSidePointActuator>();
        model.updComponent<ContactSidePointActuator>("/forceset/contact")
                .set_maximum_normal_force(0);
        CHECK_THROWS(model.initSystem());
    }

    SECTION("Serialization") {
        Model model = createFootModel<ContactSidePointActuator>();
        model.initSystem();
        model.print("testMocoContactSideActuators.osim");
        Model reloaded("testMocoContactSideActuators.osim");
        reloaded.initSystem();
        const auto& actuator = reloaded.getComponent<ContactSidePointActuator>(
                "/forceset/contact");
        CHECK(actuator.numControls() == 8);
        CHECK(&actuator.getContactSide() ==
                &reloaded.getComponent<ContactSide>("/zmp/right"));
    }
}

TEST_CASE("ContactSideActuator load dynamics") {
    Model model =
            createFootModel<ContactSidePointActuator>("distance", "constraint");
    auto& actuator =
            model.updComponent<ContactSidePointActuator>("/forceset/contact");
    CHECK_FALSE(actuator.hasLoadDynamics());
    actuator.set_load_time_constant(0.02);
    model.finalizeConnections();
    SimTK::State& state = model.initSystem();
    REQUIRE(actuator.hasLoadDynamics());
    CHECK(actuator.getStateVariableNames()[0] ==
            "/forceset/contact/normal_load");
    // Initially unloaded, whatever the control.
    CHECK(actuator.getNormalLoad(state) == 0);

    // Flat on the ground, control 0.8: the load is the state (0.5), and its
    // derivative drives it toward the control.
    actuator.setNormalLoad(state, 0.5);
    setFoot(model, state, halfHeight, {0.8, 0, 0, 0, 0, 0, 0, 0});
    CHECK(actuator.getNormalLoad(state) == 0.5);
    const double maxForce = actuator.get_maximum_normal_force();
    CHECK_THAT(actuator.calcGroundReactionForce(state)[1],
            WithinRel(0.5 * maxForce, 1e-12));
    model.realizeAcceleration(state);
    CHECK_THAT(actuator.getStateVariableDerivativeValue(state, "normal_load"),
            WithinRel((0.8 - 0.5) / 0.02, 1e-12));

    // Separate release time constant: falling (control 0.2 below the load
    // 0.5) uses mostly the release time constant.
    actuator.set_load_release_time_constant(0.004);
    {
        SimTK::State& s2 = model.initSystem();
        actuator.setNormalLoad(s2, 0.5);
        setFoot(model, s2, halfHeight, {0.3, 0, 0, 0, 0, 0, 0, 0});
        model.realizeAcceleration(s2);
        const double f = 0.5 + 0.5 * std::tanh(10 * (0.3 - 0.5));
        CHECK_THAT(actuator.getStateVariableDerivativeValue(s2, "normal_load"),
                WithinRel((0.3 - 0.5) * (f / 0.02 + (1 - f) / 0.004), 1e-12));
    }
    actuator.set_load_release_time_constant(-1);
    model.initSystem();

    // Moco's default bounds for the state come from the
    // statebounds_normal_load output.
    {
        MocoProblem problem;
        problem.setModelAsCopy(model);
        const auto rep = problem.createRep();
        const auto bounds =
                rep.getStateInfo("/forceset/contact/normal_load").getBounds();
        CHECK(bounds.getLower() == 0);
        CHECK(bounds.getUpper() == 1);
    }

    // Without dynamics, setting the state is an error.
    Model plain =
            createFootModel<ContactSidePointActuator>("distance", "constraint");
    SimTK::State& plainState = plain.initSystem();
    CHECK_THROWS(
            plain.getComponent<ContactSidePointActuator>("/forceset/contact")
                    .setNormalLoad(plainState, 0.5));
}

TEST_CASE("Contact complementarity") {
    SECTION("'constraint' enforcement: no gate, contact locations") {
        Model model = createFootModel<ContactSidePointActuator>(
                "velocity", "constraint");
        SimTK::State& state = model.initSystem();
        const auto& actuator = model.getComponent<ContactSidePointActuator>(
                "/forceset/contact");
        CHECK_FALSE(actuator.usesGate());
        // 1 cm above the ground, sliding at 0.1 m/s: load is not gated.
        model.getCoordinateSet().get("tx").setSpeedValue(state, 0.1);
        setFoot(model, state, halfHeight + 0.01,
                {0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0});
        for (const double g : actuator.calcPointGates(state)) CHECK(g == 1);
        const auto locations = actuator.calcContactLocations(state);
        REQUIRE(locations.size() == 4);
        CHECK(locations[0].name == "right_point_1");
        CHECK_THAT(locations[0].load, WithinRel(0.125, 1e-12));
        CHECK(locations[0].contactHeight == 0.008);
        CHECK_THAT(locations[0].height, WithinAbs(0.01, 1e-12));
        CHECK_THAT(locations[0].horizontalSpeed, WithinAbs(0.1, 1e-6));
        CHECK(locations[0].checksSliding);
    }

    SECTION("Constraint equations, labels and bounds") {
        MocoProblem problem;
        problem.setModelAsCopy(createFootModel<ContactSideWrenchActuator>(
                "velocity", "constraint"));
        problem.setTimeBounds(0, 1);
        auto* constraint =
                problem.addPathConstraint<MocoContactPointConstraint>();
        constraint->setName("contact");
        constraint->setEnforceContactComplementarity(true);
        constraint->setComplementarityTolerance(2e-4);
        const auto rep = problem.createRep();
        const auto& info = rep.getPathConstraint("contact").getConstraintInfo();
        // 4 points, then the wrench actuator's center of pressure.
        REQUIRE(info.getNumEquations() == 5);
        const auto labels = info.getConstraintLabels();
        CHECK(labels[4] == "contact_contact_center_of_pressure_height");
        const auto bounds = info.getBounds();
        CHECK(bounds[0].getLower() == 0);
        CHECK(bounds[4].getUpper() == 2e-4);
    }

    SECTION("Complementarity without a ContactSideActuator throws") {
        Model model = createFootModel<ContactSidePointActuator>();
        model.updForceSet().remove(0);
        model.finalizeConnections();
        MocoProblem problem;
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 1);
        problem.addPathConstraint<MocoContactPointConstraint>()
                ->setEnforceContactComplementarity(true);
        CHECK_THROWS(problem.createRep());
    }
}

#ifdef OPENSIM_WITH_CASADI
namespace {

struct LandingOptions {
    std::string contactCheckingMethod = "distance";
    std::string contactEnforcement = "constraint";
    // Start from a guess with the foot reaching the ground early and resting
    // on it (otherwise Moco's default guess, with the foot in the air).
    bool guessNearGround = false;
    // ContactSideActuator::load_time_constant.
    double loadTimeConstant = -1;
    // Track the kinematics of this solution, and use it as the guess.
    const MocoTrajectory* reference = nullptr;
};

// Landing: the foot starts 2 cm above the ground, moving down at 0.3 m/s
// (about 0.7 m/s at impact), and must be at rest on the ground at the final
// time. Its only external load is the contact actuator, so the ground reaction
// must stop it and then support its weight, under the center of mass.
template <typename T> MocoSolution testLanding(const LandingOptions& options) {
    const double duration = 0.6;
    const double initialSpeed = -0.3;
    const double dropHeight = 0.02;
    Model model = createFootModel<T>(
            options.contactCheckingMethod, options.contactEnforcement);
    // Scale the controls to the body (6 times its weight, per side), so that
    // loads are a sizeable fraction of the maximum and the problem is well
    // scaled. Stopping from about 0.7 m/s within the 8 mm contact band needs
    // about 4 times body weight, so leave a margin.
    model.updComponent<T>("/forceset/contact")
            .set_maximum_normal_force(6 * mass * 9.80665);
    model.updComponent<T>("/forceset/contact")
            .set_load_time_constant(options.loadTimeConstant);
    model.finalizeConnections();
    const auto& actuator = model.getComponent<T>("/forceset/contact");
    const std::string actuatorPath = actuator.getAbsolutePathString();

    MocoStudy study;
    MocoProblem& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, duration);
    const std::string joint = "/jointset/ground_foot/";
    // At rest on the ground at the final time (fixing only the final speeds
    // also admits ending at the top of a hop).
    problem.setStateInfo(joint + "rz/value", {-0.5, 0.5}, 0, 0);
    problem.setStateInfo(joint + "tx/value", {-0.2, 0.2}, 0);
    problem.setStateInfo(
            joint + "ty/value", {0, 0.3}, halfHeight + dropHeight, halfHeight);
    problem.setStateInfo(joint + "rz/speed", {-10, 10}, 0, 0);
    problem.setStateInfo(joint + "tx/speed", {-5, 5}, 0, 0);
    problem.setStateInfo(joint + "ty/speed", {-5, 5}, initialSpeed, 0);

    if (actuator.hasLoadDynamics()) {
        // No load in the air at the start.
        problem.setStateInfo(actuatorPath + "/normal_load", {0, 1}, 0);
    }
    const auto bounds = actuator.getControlBounds();
    for (int i = 0; i < actuator.numControls(); ++i) {
        problem.setControlInfo(actuatorPath + "_" + std::to_string(i),
                {bounds[i].first, bounds[i].second});
    }
    problem.addGoal<MocoControlGoal>("effort");
    // Small cost on the foot's speeds: rewards settling on the ground.
    // Without it, the complementarity admits local optima in which the foot
    // bounces (hops) before coming to rest. (Tracking costs play this role
    // in tracking problems.)
    for (const std::string coord : {"rz", "tx", "ty"}) {
        auto* speed = problem.addGoal<MocoOutputGoal>(coord + "_speed", 0.1);
        speed->setOutputPath(joint + coord + "|speed");
        speed->setExponent(2);
    }
    auto* noPenetration =
            problem.addPathConstraint<MocoContactPointConstraint>();
    noPenetration->setName("no_penetration");
    noPenetration->setEnforceContactComplementarity(
            options.contactEnforcement == "constraint");
    if (options.contactEnforcement == "constraint") {
        // Drives the load allowed above the band by the relaxed constraint,
        // and sliding (velocity method), toward zero.
        problem.addGoal<MocoContactPenaltyGoal>("contact_penalty", 10.0);
    }

    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(40);
    solver.set_enforce_path_constraint_mesh_interior_points(true);
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-6);

    // With 'gate' enforcement, start with the foot reaching the ground early
    // and resting on it: the contact gate gives no gradient while the foot is
    // high in the air. Otherwise, use the default guess (from the bounds),
    // with the foot in the air throughout.
    if (options.reference) {
        TimeSeriesTable reference = options.reference->exportToStatesTable();
        auto* tracking = problem.addGoal<MocoStateTrackingGoal>("tracking", 10);
        tracking->setReference(TableProcessor(reference));
        tracking->setAllowUnusedReferences(true);
        // Guess: the reference states and normal load, with the center of
        // pressure (if any) at the middle of the sole.
        MocoTrajectory guess = solver.createGuess();
        guess.insertStatesTrajectory(reference, true);
        const auto& times = guess.getTime();
        const auto normal = options.reference->getControl(actuatorPath + "_0");
        const auto& referenceTimes = options.reference->getTime();
        SimTK::Vector normalGuess(times.size());
        for (int i = 0; i < times.size(); ++i) {
            // Same mesh: the times match.
            normalGuess[i] = normal[std::min(i, referenceTimes.size() - 1)];
        }
        guess.setControl(actuatorPath + "_0", normalGuess);
        for (int i = 1; i < actuator.numControls(); ++i) {
            const auto& label = actuator.getControlLabels()[i];
            if (label.starts_with("cop_")) {
                guess.setControl(actuatorPath + "_" + std::to_string(i),
                        SimTK::Vector(times.size(), 0.5));
            }
        }
        solver.setGuess(guess);
    } else if (options.guessNearGround) {
        MocoTrajectory guess = solver.createGuess();
        const auto& guessTimes = guess.getTime();
        SimTK::Vector ty(guessTimes.size());
        for (int i = 0; i < ty.size(); ++i) {
            const double fraction =
                    std::min(1.0, guessTimes[i] / (0.2 * duration));
            ty[i] = halfHeight + dropHeight * (1 - fraction);
        }
        guess.setState(joint + "ty/value", ty);
        solver.setGuess(guess);
    }

    const MocoSolution solution = study.solve();
    const std::string prefix = "testMocoContactSideActuators_landing_" +
                               actuator.getConcreteClassName() + "_" +
                               options.contactCheckingMethod +
                               (options.reference ? "_tracking" : "") +
                               (actuator.hasLoadDynamics() ? "_dynamics" : "");
    solution.write(prefix + "_solution.sto");
    REQUIRE(solution.success());

    // Evaluate the ground reaction along the solution, with its controls.
    const std::string output = actuatorPath + "\\|";
    const auto table = analyzeMocoTrajectory<Vec3>(model, solution,
            {output + "ground_reaction_force", output + "center_of_pressure"});
    STOFileAdapter::write(table.flatten(), prefix + "_loads.sto");
    const auto forces =
            table.getDependentColumn(actuatorPath + "|ground_reaction_force");
    const auto cops =
            table.getDependentColumn(actuatorPath + "|center_of_pressure");
    const auto& times = table.getIndependentColumn();
    const double g = 9.80665;
    double impulse = 0;
    for (int i = 1; i < forces.size(); ++i) {
        impulse += 0.5 * (forces[i][1] + forces[i - 1][1]) *
                   (times[i] - times[i - 1]);
    }

    Model solved = model;
    solved.initSystem();
    const auto states = solution.exportToStatesTrajectory(solved);
    double minimumHeight = SimTK::Infinity;
    for (int i = 0; i < static_cast<int>(states.getSize()); ++i) {
        solved.realizePosition(states[i]);
        for (const auto& point : solved.getComponentList<ContactPoint>()) {
            minimumHeight = std::min(
                    minimumHeight, point.calcHeightAboveGround(states[i]));
        }
    }

    // No penetration.
    CHECK(minimumHeight > -1e-5);
    // In the air at the start: no load.
    CHECK(forces[0][1] < 1.0);
    // Vertical impulse = momentum change + weight * duration.
    CHECK_THAT(impulse,
            WithinRel(mass * (0 - initialSpeed) + mass * g * duration, 0.02));
    // At rest at the end: over the last 20% of the motion, the load supports
    // the weight, under the center of mass (the body origin), with negligible
    // friction. (The controls at the final mesh point barely affect the
    // dynamics, so the final point alone is not reliable.)
    const auto tx = solution.getState(joint + "tx/value");
    Vec3 meanForce(0);
    double meanOffset = 0;
    int count = 0;
    for (int i = 0; i < forces.size(); ++i) {
        if (times[i] < 0.8 * duration) continue;
        meanForce += forces[i];
        meanOffset += cops[i][0] - tx[i];
        ++count;
    }
    meanForce /= count;
    meanOffset /= count;
    CHECK_THAT(meanForce[1], WithinRel(mass * g, 0.02));
    CHECK_THAT(meanForce[0], WithinAbs(0, 0.02 * mass * g));
    CHECK_THAT(meanOffset, WithinAbs(0, 0.005));
    return solution;
}

} // namespace

// Predictive: 'constraint' enforcement, from Moco's default guess (foot in
// the air throughout).
TEST_CASE("Predictive landing: ContactSidePointActuator", "[casadi]") {
    using Actuator = ContactSidePointActuator;
    SECTION("distance") { testLanding<Actuator>({}); }
    SECTION("velocity") {
        LandingOptions options;
        options.contactCheckingMethod = "velocity";
        testLanding<Actuator>(options);
    }
    SECTION("distance, load dynamics") {
        LandingOptions options;
        options.loadTimeConstant = 0.02;
        testLanding<Actuator>(options);
    }
}

// The wrench actuator is intended for tracking problems: 'gate' enforcement,
// tracking kinematics (here, the point actuator's landing) from a guess
// based on them.
TEST_CASE("Tracking landing: ContactSideWrenchActuator", "[casadi]") {
    const MocoSolution reference = testLanding<ContactSidePointActuator>({});
    LandingOptions options;
    options.contactEnforcement = "gate";
    options.reference = &reference;
    testLanding<ContactSideWrenchActuator>(options);
}

// The point actuator with 'gate' enforcement (for comparison; slower).
TEST_CASE("Predictive landing: ContactSidePointActuator with contact gates",
        "[casadi][.gate]") {
    LandingOptions options;
    options.contactEnforcement = "gate";
    options.guessNearGround = true;
    testLanding<ContactSidePointActuator>(options);
}
#endif // OPENSIM_WITH_CASADI
