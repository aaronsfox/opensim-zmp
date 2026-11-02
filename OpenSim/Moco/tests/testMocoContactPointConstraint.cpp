/* -------------------------------------------------------------------------- *
 *               OpenSim:  testMocoContactPointConstraint.cpp                 *
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

// These tests evaluate the constraint through MocoProblemRep, so they do not
// need a solver.

#include <catch2/catch_all.hpp>
#include <string>
#include <vector>

#include <OpenSim/Moco/MocoContactPointConstraint.h>
#include <OpenSim/Moco/MocoProblem.h>
#include <OpenSim/Moco/MocoProblemRep.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>

using namespace OpenSim;
using Catch::Matchers::WithinAbs;
using SimTK::Vec3;

namespace {

constexpr double halfHeight = 0.05;

// Two feet on planar joints (Moco does not support FreeJoint), each with a
// ContactSide. The right foot has four
// points at the corners of its sole; the left foot has two (heel and toe).
Model createTwoFootModel(double groundHeight = 0) {
    Model model;
    model.setName("two_feet");
    auto* zmp = new ZeroMomentPointEstimator("ground_right");
    zmp->setName("zmp");
    zmp->setGroundHeight(groundHeight);
    for (const std::string side : {"right", "left"}) {
        auto* foot =
                new Body(side + "_foot", 1.0, Vec3(0), SimTK::Inertia(0.01));
        model.addBody(foot);
        auto* joint =
                new PlanarJoint("ground_" + side, model.getGround(), *foot);
        const std::vector<std::string> names{"rz", "tx", "ty"};
        for (int i = 0; i < 3; ++i) {
            joint->upd_coordinates(i).setName(side + "_" + names[i]);
        }
        model.addJoint(joint);

        auto* contactSide = new ContactSide(side, *foot);
        std::vector<Vec3> locations;
        if (side == "right") {
            locations = {Vec3(-0.1, -halfHeight, -0.04),
                    Vec3(-0.1, -halfHeight, 0.04),
                    Vec3(0.1, -halfHeight, -0.04),
                    Vec3(0.1, -halfHeight, 0.04)};
        } else {
            locations = {Vec3(-0.1, -halfHeight, 0), Vec3(0.1, -halfHeight, 0)};
        }
        for (const auto& location : locations) {
            contactSide->adoptContactPoint(new ContactPoint(*foot, location));
        }
        zmp->adoptContactSide(contactSide);
    }
    model.addComponent(zmp);
    model.finalizeConnections();
    return model;
}

struct Evaluation {
    std::vector<std::string> labels;
    std::vector<MocoBounds> bounds;
    SimTK::Vector errors;
};

// Evaluate the constraint named "no_penetration" with each foot's origin at
// the given heights (and the right foot tilted about z by `tilt` rad).
Evaluation evaluate(const MocoProblem& problem, double rightHeight,
        double leftHeight, double tilt = 0) {
    const MocoProblemRep rep = problem.createRep();
    const auto& constraint = rep.getPathConstraint("no_penetration");
    Evaluation evaluation;
    evaluation.labels = constraint.getConstraintInfo().getConstraintLabels();
    evaluation.bounds = constraint.getConstraintInfo().getBounds();

    const Model& model = rep.getModelDisabledConstraints();
    SimTK::State& state = rep.updStateDisabledConstraints();
    model.getCoordinateSet().get("right_ty").setValue(state, rightHeight);
    model.getCoordinateSet().get("left_ty").setValue(state, leftHeight);
    model.getCoordinateSet().get("right_rz").setValue(state, tilt);
    model.realizePosition(state);
    evaluation.errors.resize(rep.getNumPathConstraintEquations());
    rep.calcPathConstraintErrors(state, evaluation.errors);
    return evaluation;
}

MocoProblem createProblem(const Model& model) {
    MocoProblem problem;
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    return problem;
}

} // namespace

TEST_CASE("MocoContactPointConstraint selects points") {
    const Model model = createTwoFootModel();

    SECTION("Default: every ContactPoint in the model") {
        MocoProblem problem = createProblem(model);
        problem.addPathConstraint<MocoContactPointConstraint>()->setName(
                "no_penetration");
        const auto evaluation =
                evaluate(problem, halfHeight + 0.01, halfHeight + 0.02);
        REQUIRE(evaluation.errors.size() == 6);
        CHECK(evaluation.labels[0] == "no_penetration_right_point_1");
        CHECK(evaluation.labels[5] == "no_penetration_left_point_2");
        for (int i = 0; i < 4; ++i) {
            CHECK_THAT(evaluation.errors[i], WithinAbs(0.01, 1e-12));
        }
        for (int i = 4; i < 6; ++i) {
            CHECK_THAT(evaluation.errors[i], WithinAbs(0.02, 1e-12));
        }
        for (const auto& bounds : evaluation.bounds) {
            CHECK(bounds.getLower() == 0);
            CHECK(bounds.getUpper() == SimTK::Infinity);
        }
    }

    SECTION("A side path, with a duplicate point path counted once") {
        MocoProblem problem = createProblem(model);
        auto* constraint =
                problem.addPathConstraint<MocoContactPointConstraint>();
        constraint->setName("no_penetration");
        constraint->addContactSidePath("/zmp/left");
        constraint->addContactPointPath("/zmp/left/left_point_1");
        const auto evaluation = evaluate(problem, 0.5, halfHeight - 0.003);
        REQUIRE(evaluation.errors.size() == 2);
        CHECK(evaluation.labels[0] == "no_penetration_left_point_1");
        // Below the ground: a violated constraint (negative value).
        CHECK_THAT(evaluation.errors[0], WithinAbs(-0.003, 1e-12));
    }

    SECTION("Point paths only") {
        MocoProblem problem = createProblem(model);
        auto* constraint =
                problem.addPathConstraint<MocoContactPointConstraint>();
        constraint->setName("no_penetration");
        constraint->addContactPointPath("/zmp/right/right_point_3");
        // Tilt the right foot so the toe (x = 0.1) is lower than the heel.
        const double tilt = -0.1;
        const auto evaluation = evaluate(problem, halfHeight, halfHeight, tilt);
        REQUIRE(evaluation.errors.size() == 1);
        const double expected =
                halfHeight + 0.1 * std::sin(tilt) - halfHeight * std::cos(tilt);
        CHECK_THAT(evaluation.errors[0], WithinAbs(expected, 1e-12));
    }
}

TEST_CASE("MocoContactPointConstraint uses the estimator's ground height") {
    MocoProblem problem = createProblem(createTwoFootModel(0.1));
    problem.addPathConstraint<MocoContactPointConstraint>()->setName(
            "no_penetration");
    const auto evaluation =
            evaluate(problem, 0.1 + halfHeight + 0.01, 0.1 + halfHeight);
    CHECK_THAT(evaluation.errors[0], WithinAbs(0.01, 1e-12));
    CHECK_THAT(evaluation.errors[5], WithinAbs(0.0, 1e-12));
}

TEST_CASE("MocoContactPointConstraint penetration tolerance") {
    MocoProblem problem = createProblem(createTwoFootModel());
    auto* constraint = problem.addPathConstraint<MocoContactPointConstraint>();
    constraint->setName("no_penetration");
    constraint->setPenetrationTolerance(0.005);
    const auto evaluation = evaluate(problem, halfHeight, halfHeight);
    for (const auto& bounds : evaluation.bounds) {
        CHECK(bounds.getLower() == -0.005);
    }

    constraint->setPenetrationTolerance(-0.001);
    CHECK_THROWS(problem.createRep());
}

TEST_CASE("MocoContactPointConstraint invalid input") {
    const Model model = createTwoFootModel();

    SECTION("Unknown point path") {
        MocoProblem problem = createProblem(model);
        problem.addPathConstraint<MocoContactPointConstraint>()
                ->addContactPointPath("/zmp/right/missing");
        CHECK_THROWS(problem.createRep());
    }

    SECTION("Unknown side path") {
        MocoProblem problem = createProblem(model);
        problem.addPathConstraint<MocoContactPointConstraint>()
                ->addContactSidePath("/zmp/middle");
        CHECK_THROWS(problem.createRep());
    }

    SECTION("Model without ContactPoints") {
        Model empty;
        auto* body = new Body("body", 1.0, Vec3(0), SimTK::Inertia(0.01));
        empty.addBody(body);
        empty.addJoint(new PlanarJoint("planar", empty.getGround(), *body));
        empty.finalizeConnections();
        MocoProblem problem = createProblem(empty);
        problem.addPathConstraint<MocoContactPointConstraint>();
        CHECK_THROWS(problem.createRep());
    }
}

TEST_CASE("MocoContactPointConstraint serialization") {
    MocoProblem problem = createProblem(createTwoFootModel());
    auto* constraint = problem.addPathConstraint<MocoContactPointConstraint>();
    constraint->setName("no_penetration");
    constraint->addContactSidePath("/zmp/right");
    constraint->setPenetrationTolerance(0.002);
    const std::string file = "testMocoContactPointConstraint.xml";
    constraint->print(file);

    std::unique_ptr<Object> object(Object::makeObjectFromFile(file));
    const auto* loaded =
            dynamic_cast<MocoContactPointConstraint*>(object.get());
    REQUIRE(loaded != nullptr);
    CHECK(loaded->getPenetrationTolerance() == 0.002);
}
