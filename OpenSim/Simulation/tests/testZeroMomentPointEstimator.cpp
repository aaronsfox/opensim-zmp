/* -------------------------------------------------------------------------- *
 *               OpenSim:  testZeroMomentPointEstimator.cpp                   *
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

#include <array>
#include <catch2/catch_all.hpp>
#include <memory>
#include <vector>

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PointToPointSpring.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/PositionMotion.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>

using namespace OpenSim;
using Catch::Matchers::WithinAbs;
using SimTK::Vec3;

namespace {

constexpr double tol = 1e-8;
constexpr double mass = 10.0;
constexpr double halfHeight = 0.05;
const Vec3 defaultMassCenter(0.02, 0.0, 0.01);
const SimTK::Inertia inertia(0.01, 0.02, 0.03);
const double g = 9.80665;

// Contact points at the corners of the sole, which is halfHeight below the
// body origin.
std::vector<Vec3> soleCorners() {
    return {Vec3(-0.1, -halfHeight, -0.04), Vec3(-0.1, -halfHeight, 0.04),
            Vec3(0.1, -halfHeight, -0.04), Vec3(0.1, -halfHeight, 0.04)};
}

struct FootModelOptions {
    // Coordinate functions for (rx, ry, rz, tx, ty, tz).
    std::array<std::shared_ptr<Function>, 6> coordinates{
            std::make_shared<Constant>(0.0), std::make_shared<Constant>(0.0),
            std::make_shared<Constant>(0.0), std::make_shared<Constant>(0.3),
            std::make_shared<Constant>(halfHeight),
            std::make_shared<Constant>(-0.2)};
    std::vector<std::string> sideNames{"right"};
    std::vector<Vec3> contactPoints = soleCorners();
    Vec3 massCenter = defaultMassCenter;
    std::string contactCheckingMethod = "distance";
    double groundHeight = 0.0;
};

// A single rigid "foot" on a FreeJoint whose motion is prescribed with a
// PositionMotion, with a ZeroMomentPointEstimator named "zmp".
std::unique_ptr<Model> createFootModel(const FootModelOptions& options) {
    auto model = std::make_unique<Model>();
    model->setName("foot_model");

    auto* foot = new Body("foot", mass, options.massCenter, inertia);
    model->addBody(foot);

    auto* joint = new FreeJoint("ground_foot", model->getGround(), *foot);
    const std::array<std::string, 6> names{"rx", "ry", "rz", "tx", "ty", "tz"};
    for (int i = 0; i < 6; ++i) joint->upd_coordinates(i).setName(names[i]);
    model->addJoint(joint);

    auto* estimator = new ZeroMomentPointEstimator("ground_foot");
    estimator->setName("zmp");
    estimator->setGroundHeight(options.groundHeight);
    for (const auto& sideName : options.sideNames) {
        auto* side = new ContactSide(sideName, *foot);
        for (const auto& location : options.contactPoints) {
            auto* point = new ContactPoint(*foot, location);
            point->setContactCheckingMethod(options.contactCheckingMethod);
            side->adoptContactPoint(point);
        }
        estimator->adoptContactSide(side);
    }
    model->addComponent(estimator);

    // Coordinates need owners before PositionMotion can find their paths.
    model->finalizeFromProperties();
    auto* motion = new PositionMotion("motion");
    for (int i = 0; i < 6; ++i) {
        motion->setPositionForCoordinate(
                joint->get_coordinates(i), *options.coordinates[i]);
    }
    model->addComponent(motion);
    return model;
}

void realizeAtTime(const Model& model, SimTK::State& state, double time) {
    state.setTime(time);
    model.getSystem().prescribe(state);
    model.realizeAcceleration(state);
}

// f(t) = c2 t^2 + c1 t + c0.
std::shared_ptr<Function> quadratic(double c2, double c1, double c0) {
    SimTK::Vector coefficients(3);
    coefficients[0] = c2;
    coefficients[1] = c1;
    coefficients[2] = c0;
    return std::make_shared<PolynomialFunction>(coefficients);
}

const ZeroMomentPointEstimator& getEstimator(const Model& model) {
    return model.getComponent<ZeroMomentPointEstimator>("/zmp");
}

void checkVec3(const Vec3& actual, const Vec3& expected, double tolerance) {
    for (int i = 0; i < 3; ++i) {
        CHECK_THAT(actual[i], WithinAbs(expected[i], tolerance));
    }
}

} // namespace

TEST_CASE("ZeroMomentPointEstimator single support") {
    SECTION("Body at rest: GRF = mg, CoP under COM, no free moment") {
        auto model = createFootModel({});
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& zmp = getEstimator(*model);

        CHECK(zmp.getContactMode(state) ==
                ZeroMomentPointEstimator::ContactMode::Unilateral);
        CHECK(zmp.getIsInContact(state, "right"));
        CHECK(zmp.getIsValid(state, "right"));
        checkVec3(zmp.getGroundReactionForce(state, "right"),
                Vec3(0, mass * g, 0), tol);
        checkVec3(zmp.getCenterOfPressure(state, "right"),
                Vec3(0.3 + defaultMassCenter[0], 0,
                        -0.2 + defaultMassCenter[2]),
                tol);
        checkVec3(zmp.getFreeMoment(state, "right"), Vec3(0), tol);
    }

    SECTION("Horizontal acceleration: GRF = m(a - g), line through COM") {
        // tx(t) = 0.5 a t^2 + 0.3, so the COM accelerates at a along x.
        const double a = 2.0;
        FootModelOptions options;
        options.coordinates[3] = quadratic(0.5 * a, 0, 0.3);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        const double t = 0.1;
        realizeAtTime(*model, state, t);
        const auto& zmp = getEstimator(*model);

        const Vec3 force(mass * a, mass * g, 0);
        checkVec3(zmp.getGroundReactionForce(state, "right"), force, tol);

        // A purely translating body needs no moment about its COM, so the GRF
        // acts along a line through the COM.
        const Vec3 com(0.5 * a * t * t + 0.3 + defaultMassCenter[0], halfHeight,
                -0.2 + defaultMassCenter[2]);
        const Vec3 expectedCoP(
                com[0] - com[1] * force[0] / force[1], 0, com[2]);
        checkVec3(zmp.getCenterOfPressure(state, "right"), expectedCoP, tol);
        checkVec3(zmp.getFreeMoment(state, "right"), Vec3(0), tol);
    }

    SECTION("Angular acceleration about the vertical axis: free moment") {
        // ry(t) = 0.5 alpha t^2 about an axis through the COM.
        const double alpha = 3.0;
        FootModelOptions options;
        options.massCenter = Vec3(0);
        options.coordinates[1] = quadratic(0.5 * alpha, 0, 0);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.2);
        const auto& zmp = getEstimator(*model);

        checkVec3(zmp.getGroundReactionForce(state, "right"),
                Vec3(0, mass * g, 0), tol);
        checkVec3(zmp.getCenterOfPressure(state, "right"), Vec3(0.3, 0, -0.2),
                tol);
        // The ground must supply the torque I_yy * alpha.
        checkVec3(zmp.getFreeMoment(state, "right"),
                Vec3(0, inertia.getMoments()[1] * alpha, 0), tol);
    }

    SECTION("Model forces do not change the estimate") {
        // A spring between Ground and the foot is an external force that the
        // previous (all-forces) inverse dynamics would have subtracted.
        auto model = createFootModel({});
        const auto& foot = model->getBodySet().get("foot");
        model->addForce(new PointToPointSpring(
                model->getGround(), Vec3(0, 1, 0), foot, Vec3(0), 1000, 0));
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        checkVec3(getEstimator(*model).getGroundReactionForce(state, "right"),
                Vec3(0, mass * g, 0), tol);
    }

    SECTION("Outputs are available by their lower-case names") {
        auto model = createFootModel({});
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& zmp = getEstimator(*model);

        const auto& grf = dynamic_cast<const Output<Vec3>&>(
                zmp.getOutput("ground_reaction_force"));
        const auto& channel = dynamic_cast<const Output<Vec3>::Channel&>(
                grf.getChannel("right"));
        checkVec3(channel.getValue(state), Vec3(0, mass * g, 0), tol);
        for (const std::string name : {"center_of_pressure", "free_moment",
                     "is_in_contact", "is_valid", "contact_mode"}) {
            CHECK_NOTHROW(zmp.getOutput(name));
        }
        CHECK(zmp.getOutput("is_in_contact").getDependsOnStage() ==
                SimTK::Stage::Velocity);
        CHECK(zmp.getOutput("contact_mode").getDependsOnStage() ==
                SimTK::Stage::Velocity);
        CHECK(zmp.getOutput("ground_reaction_force").getDependsOnStage() ==
                SimTK::Stage::Acceleration);
    }

    SECTION("GRF requires Stage::Acceleration; contact needs only Velocity") {
        auto model = createFootModel({});
        SimTK::State& state = model->initSystem();
        state.setTime(0.0);
        model->getSystem().prescribe(state);
        model->realizeVelocity(state);
        const auto& zmp = getEstimator(*model);
        CHECK(zmp.getIsInContact(state, "right"));
        CHECK(zmp.getContactModeAsInt(state) == 1);
        CHECK_THROWS(zmp.getGroundReactionForce(state, "right"));
    }
}

TEST_CASE("ZeroMomentPointEstimator contact modes") {
    SECTION("No contact: outputs zero and not valid") {
        FootModelOptions options;
        options.coordinates[4] = std::make_shared<Constant>(0.5);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& zmp = getEstimator(*model);

        CHECK(zmp.getContactMode(state) ==
                ZeroMomentPointEstimator::ContactMode::NoContact);
        CHECK_FALSE(zmp.getIsInContact(state, "right"));
        CHECK_FALSE(zmp.getIsValid(state, "right"));
        checkVec3(zmp.getGroundReactionForce(state, "right"), Vec3(0), 0);
    }

    SECTION("Both sides in contact: outputs zero and not valid (TODO)") {
        FootModelOptions options;
        options.sideNames = {"right", "left"};
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& zmp = getEstimator(*model);

        CHECK(zmp.getContactMode(state) ==
                ZeroMomentPointEstimator::ContactMode::Bilateral);
        for (const std::string side : {"right", "left"}) {
            CHECK(zmp.getIsInContact(state, side));
            CHECK_FALSE(zmp.getIsValid(state, side));
            checkVec3(zmp.getGroundReactionForce(state, side), Vec3(0), 0);
        }
    }

    SECTION("Vertical force below contact_force_threshold is not valid") {
        // Downward acceleration greater than g gives a negative vertical GRF.
        FootModelOptions options;
        options.coordinates[4] = quadratic(-0.5 * 2 * g, 0, halfHeight);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& zmp = getEstimator(*model);
        CHECK(zmp.getIsInContact(state, "right"));
        CHECK_FALSE(zmp.getIsValid(state, "right"));
    }
}

TEST_CASE("ZeroMomentPointEstimator ground_height") {
    SECTION("CoP lies on the ground plane and the ZMP formula uses h") {
        const double h = 0.1;
        const double a = 2.0;
        FootModelOptions options;
        options.groundHeight = h;
        options.coordinates[3] = quadratic(0.5 * a, 0, 0.3);
        options.coordinates[4] = std::make_shared<Constant>(h + halfHeight);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        const double t = 0.1;
        realizeAtTime(*model, state, t);
        const auto& zmp = getEstimator(*model);

        const Vec3 force(mass * a, mass * g, 0);
        const Vec3 com(0.5 * a * t * t + 0.3 + defaultMassCenter[0],
                h + halfHeight, -0.2 + defaultMassCenter[2]);
        const Vec3 expectedCoP(
                com[0] - (com[1] - h) * force[0] / force[1], h, com[2]);
        checkVec3(zmp.getGroundReactionForce(state, "right"), force, tol);
        checkVec3(zmp.getCenterOfPressure(state, "right"), expectedCoP, tol);
    }

    SECTION("Contact detection uses the estimator's ground height") {
        // Sole 5 mm above a plane at h = 0.1 (in contact), but 105 mm above
        // y = 0 (not in contact).
        FootModelOptions options;
        options.coordinates[4] =
                std::make_shared<Constant>(0.1 + 0.005 + halfHeight);
        options.groundHeight = 0.1;
        {
            auto model = createFootModel(options);
            SimTK::State& state = model->initSystem();
            realizeAtTime(*model, state, 0.0);
            const auto& zmp = getEstimator(*model);
            const auto& point = zmp.getContactSide("right").getContactPoint(0);
            CHECK(point.getGroundHeight() == 0.1);
            CHECK_THAT(
                    point.calcHeightAboveGround(state), WithinAbs(0.005, tol));
            CHECK(zmp.getIsInContact(state, "right"));
        }
        options.groundHeight = 0.0;
        {
            auto model = createFootModel(options);
            SimTK::State& state = model->initSystem();
            realizeAtTime(*model, state, 0.0);
            CHECK_FALSE(getEstimator(*model).getIsInContact(state, "right"));
        }
    }
}

TEST_CASE("ContactPoint contact checking methods") {
    // Foot sliding along x at constant speed v with the sole on the ground.
    auto isInContact = [](const std::string& method, double v) {
        FootModelOptions options;
        options.contactCheckingMethod = method;
        options.coordinates[3] = std::make_shared<LinearFunction>(v, 0.3);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        return getEstimator(*model).getIsInContact(state, "right");
    };

    SECTION("Defaults follow Karcnik (2003)") {
        ContactPoint point;
        CHECK(point.getDistanceThreshold() == 0.008);
        CHECK(point.getVelocityThreshold() == 0.015);
        CHECK(point.getContactCheckingMethodName() == "distance");
    }

    SECTION("distance ignores velocity") {
        CHECK(isInContact("distance", 0.1));
    }

    SECTION("velocity requires the point to be nearly stationary") {
        CHECK_FALSE(isInContact("velocity", 0.1));
        CHECK(isInContact("velocity", 0.01));
    }

    SECTION("Method is parsed into an enum") {
        FootModelOptions options;
        options.contactCheckingMethod = "velocity";
        auto model = createFootModel(options);
        model->initSystem();
        CHECK(getEstimator(*model)
                        .getContactSide(0)
                        .getContactPoint(0)
                        .getContactCheckingMethod() ==
                ContactPoint::ContactCheckingMethod::Velocity);
    }

    SECTION("Point above the distance threshold is not in contact") {
        FootModelOptions options;
        options.coordinates[4] = std::make_shared<Constant>(halfHeight + 0.01);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        CHECK_FALSE(getEstimator(*model).getIsInContact(state, "right"));
    }
}

TEST_CASE("ContactSide support polygon") {
    SECTION("Four corners in contact") {
        auto model = createFootModel({});
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& side = getEstimator(*model).getContactSide("right");

        CHECK(side.calcNumActiveContactPoints(state) == 4);
        CHECK(side.calcSupportPolygonInGround(state).size() == 4);
        checkVec3(side.calcActiveContactPointCentroidInGround(state),
                Vec3(0.3, 0, -0.2), tol);

        // The CoP of the body at rest is 3 cm from the nearest (z) edge.
        const Vec3 cop(0.32, 0, -0.19);
        CHECK_THAT(side.calcSignedDistanceToSupportPolygon(state, cop),
                WithinAbs(-0.03, tol));
        CHECK(side.isPointInSupportPolygon(state, cop));

        const Vec3 outside(0.41, 0, -0.2);
        CHECK_THAT(side.calcSignedDistanceToSupportPolygon(state, outside),
                WithinAbs(0.01, tol));
        CHECK_FALSE(side.isPointInSupportPolygon(state, outside));
        CHECK(side.isPointInSupportPolygon(state, outside, 0.02));
    }

    SECTION("Two points form a segment; one point is a point") {
        FootModelOptions options;
        options.contactPoints = {
                Vec3(-0.1, -halfHeight, 0), Vec3(0.1, -halfHeight, 0)};
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& side = getEstimator(*model).getContactSide("right");

        CHECK(side.calcSupportPolygonInGround(state).size() == 2);
        CHECK_THAT(side.calcSignedDistanceToSupportPolygon(
                           state, Vec3(0.3, 0, -0.1)),
                WithinAbs(0.1, tol));
        CHECK_THAT(side.calcSignedDistanceToSupportPolygon(
                           state, Vec3(0.5, 0, -0.2)),
                WithinAbs(0.1, tol));

        options.contactPoints = {Vec3(0, -halfHeight, 0)};
        auto model1 = createFootModel(options);
        SimTK::State& state1 = model1->initSystem();
        realizeAtTime(*model1, state1, 0.0);
        const auto& side1 = getEstimator(*model1).getContactSide("right");
        CHECK(side1.calcSupportPolygonInGround(state1).size() == 1);
        CHECK_THAT(side1.calcSignedDistanceToSupportPolygon(
                           state1, Vec3(0.3, 0, -0.1)),
                WithinAbs(0.1, tol));
    }

    SECTION("No points in contact") {
        FootModelOptions options;
        options.coordinates[4] = std::make_shared<Constant>(0.5);
        auto model = createFootModel(options);
        SimTK::State& state = model->initSystem();
        realizeAtTime(*model, state, 0.0);
        const auto& side = getEstimator(*model).getContactSide("right");
        CHECK(side.calcSupportPolygonInGround(state).empty());
        CHECK(SimTK::isNaN(
                side.calcSignedDistanceToSupportPolygon(state, Vec3(0))));
        CHECK_FALSE(side.isPointInSupportPolygon(state, Vec3(0), 1.0));
    }
}

TEST_CASE("ZeroMomentPointEstimator naming and validation") {
    Model model;
    auto* foot = new Body("foot", mass, defaultMassCenter, inertia);
    model.addBody(foot);
    model.addJoint(new FreeJoint("ground_foot", model.getGround(), *foot));

    SECTION("Unnamed sides and points get descriptive names") {
        ZeroMomentPointEstimator zmp("ground_foot");
        auto* side = new ContactSide();
        side->setContactBody(*foot);
        zmp.adoptContactSide(side);
        CHECK(zmp.getContactSide(0).getName() == "foot");

        auto* named = new ContactSide(*foot);
        CHECK(named->getName() == "foot");
        delete named;

        ContactSide right("right", *foot);
        right.adoptContactPoint(new ContactPoint(*foot, Vec3(0)));
        right.adoptContactPoint(new ContactPoint(*foot, Vec3(0.1, 0, 0)));
        CHECK(right.getContactPoint(0).getName() == "right_point_1");
        CHECK(right.getContactPoint(1).getName() == "right_point_2");
    }

    SECTION("Duplicate names and a third side throw") {
        ZeroMomentPointEstimator zmp("ground_foot");
        zmp.adoptContactSide(new ContactSide("right", *foot));
        // On failure, ownership stays with the caller.
        auto duplicate = std::make_unique<ContactSide>("right", *foot);
        CHECK_THROWS(zmp.adoptContactSide(duplicate.get()));
        zmp.adoptContactSide(new ContactSide("left", *foot));
        auto third = std::make_unique<ContactSide>("third", *foot);
        CHECK_THROWS(zmp.adoptContactSide(third.get()));
        CHECK_THROWS(zmp.adoptContactSide(nullptr));
        CHECK(zmp.getNumContactSides() == 2);

        ContactSide side("right", *foot);
        side.adoptContactPoint(new ContactPoint("heel", *foot, Vec3(0)));
        auto heel =
                std::make_unique<ContactPoint>("heel", *foot, Vec3(0.1, 0, 0));
        CHECK_THROWS(side.adoptContactPoint(heel.get()));
        CHECK(side.getNumContactPoints() == 1);
    }

    SECTION("addContactSide/addContactPoint copy the argument") {
        ZeroMomentPointEstimator zmp("ground_foot");
        ContactSide side("right", *foot);
        ContactPoint point("heel", *foot, Vec3(0));
        side.addContactPoint(point);
        zmp.addContactSide(side);
        CHECK(zmp.getContactSide("right").getContactPoint("heel").getName() ==
                "heel");
        CHECK(side.getNumContactPoints() == 1);
    }

    auto addEstimator = [&](const std::string& jointName) {
        auto* zmp = new ZeroMomentPointEstimator(jointName);
        auto* side = new ContactSide("right", *foot);
        side->adoptContactPoint(new ContactPoint(*foot, Vec3(0)));
        zmp->adoptContactSide(side);
        model.addComponent(zmp);
        return zmp;
    };

    SECTION("Valid model initializes") {
        addEstimator("ground_foot");
        CHECK_NOTHROW(model.initSystem());
    }

    SECTION("Copied sides and points keep their connections") {
        auto* zmp = new ZeroMomentPointEstimator("ground_foot");
        ContactSide side("right", *foot);
        side.addContactPoint(ContactPoint("heel", *foot, Vec3(0)));
        zmp->addContactSide(side);
        model.addComponent(zmp);
        CHECK_NOTHROW(model.initSystem());
        CHECK(&zmp->getContactSide("right").getContactBody() == foot);
    }

    SECTION("Missing free joint throws") {
        addEstimator("ground_pelvis");
        CHECK_THROWS_WITH(model.initSystem(),
                Catch::Matchers::ContainsSubstring("ground_pelvis"));
    }

    SECTION("Free joint whose parent is not Ground throws") {
        auto* toe = new Body("toe", 0.1, Vec3(0), SimTK::Inertia(0.001));
        model.addBody(toe);
        model.addJoint(new FreeJoint("foot_toe", *foot, *toe));
        addEstimator("foot_toe");
        CHECK_THROWS_WITH(model.initSystem(),
                Catch::Matchers::ContainsSubstring("Ground"));
    }

    SECTION("No contact sides throws") {
        // addComponent() finalizes the component, which throws.
        CHECK_THROWS(model.addComponent(
                new ZeroMomentPointEstimator("ground_foot")));
    }

    SECTION("Invalid ContactPoint properties throw") {
        auto* zmp = addEstimator("ground_foot");
        SECTION("method") {
            zmp->updContactSide(0).updContactPoint(0).setContactCheckingMethod(
                    "magic");
            CHECK_THROWS(model.initSystem());
        }
        SECTION("distance threshold") {
            zmp->updContactSide(0).updContactPoint(0).setDistanceThreshold(-1);
            CHECK_THROWS(model.initSystem());
        }
        SECTION("velocity threshold") {
            zmp->updContactSide(0).updContactPoint(0).setVelocityThreshold(-1);
            CHECK_THROWS(model.initSystem());
        }
        SECTION("force threshold") {
            zmp->setContactForceThreshold(-1);
            CHECK_THROWS(model.initSystem());
        }
    }
}

TEST_CASE("ZeroMomentPointEstimator serialization") {
    FootModelOptions options;
    options.groundHeight = 0.0;
    auto model = createFootModel(options);
    model->initSystem();
    const std::string filename =
            "testZeroMomentPointEstimator_serialization.osim";
    model->print(filename);

    Model reloaded(filename);
    SimTK::State& state = reloaded.initSystem();
    realizeAtTime(reloaded, state, 0.0);
    const auto& zmp = getEstimator(reloaded);
    CHECK(zmp.getNumContactSides() == 1);
    CHECK(zmp.getContactSide("right").getNumContactPoints() == 4);
    CHECK(&zmp.getContactSide("right").getContactBody() ==
            &reloaded.getBodySet().get("foot"));
    checkVec3(zmp.getGroundReactionForce(state, "right"), Vec3(0, mass * g, 0),
            tol);
}
