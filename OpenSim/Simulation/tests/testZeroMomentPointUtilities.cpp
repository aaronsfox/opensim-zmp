/* -------------------------------------------------------------------------- *
 *               OpenSim:  testZeroMomentPointUtilities.cpp                   *
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
#include <string>
#include <vector>

#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/PositionMotion.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/ZeroMomentPointUtilities.h>

using namespace OpenSim;
using Catch::Matchers::WithinAbs;
using SimTK::Vec3;

namespace {

constexpr double mass = 10.0;
constexpr double halfHeight = 0.05;
const double g = 9.80665;
const std::array<std::string, 6> coordNames{"rx", "ry", "rz", "tx", "ty", "tz"};
const std::string jointPath = "/jointset/ground_foot/";

// Each coordinate is c2 t^2 + c1 t + c0: the foot slides and accelerates
// along x, drifts along z, and spins up about the vertical axis, with its
// sole on the ground (single support on side "right").
struct Quadratic {
    double c2, c1, c0;
    double value(double t) const { return c2 * t * t + c1 * t + c0; }
    double speed(double t) const { return 2 * c2 * t + c1; }
    double accel() const { return 2 * c2; }
};
const std::array<Quadratic, 6> motion{Quadratic{0, 0, 0}, Quadratic{1.5, 0, 0},
        Quadratic{0, 0, 0}, Quadratic{1.0, 0, 0.3}, Quadratic{0, 0, halfHeight},
        Quadratic{0, 0.1, -0.2}};

std::vector<double> sampleTimes() {
    std::vector<double> times;
    for (int i = 0; i <= 50; ++i) times.push_back(0.01 * i);
    return times;
}

// A rigid foot on a FreeJoint with a ZeroMomentPointEstimator, no motion.
Model createFootModel() {
    Model model;
    model.setName("foot_model");
    auto* foot = new Body("foot", mass, Vec3(0.02, 0, 0.01),
            SimTK::Inertia(0.01, 0.02, 0.03));
    model.addBody(foot);
    auto* joint = new FreeJoint("ground_foot", model.getGround(), *foot);
    for (int i = 0; i < 6; ++i)
        joint->upd_coordinates(i).setName(coordNames[i]);
    model.addJoint(joint);

    auto* estimator = new ZeroMomentPointEstimator("ground_foot");
    estimator->setName("zmp");
    auto* side = new ContactSide("right", *foot);
    for (const Vec3& corner : {Vec3(-0.1, -halfHeight, -0.04),
                 Vec3(-0.1, -halfHeight, 0.04), Vec3(0.1, -halfHeight, -0.04),
                 Vec3(0.1, -halfHeight, 0.04)}) {
        side->adoptContactPoint(new ContactPoint(*foot, corner));
    }
    estimator->adoptContactSide(side);
    model.addComponent(estimator);
    model.finalizeConnections();
    return model;
}

// Reference: the estimator evaluated with the exact motion (polynomial
// PositionMotion), as a 9-column row per time.
std::vector<SimTK::RowVector> referenceRows(
        Model model, const std::vector<double>& times) {
    model.finalizeFromProperties();
    auto* positionMotion = new PositionMotion("reference_motion");
    for (int i = 0; i < 6; ++i) {
        SimTK::Vector coefficients(3);
        coefficients[0] = motion[i].c2;
        coefficients[1] = motion[i].c1;
        coefficients[2] = motion[i].c0;
        positionMotion->setPositionForCoordinate(
                model.getComponent<Coordinate>(jointPath + coordNames[i]),
                PolynomialFunction(coefficients));
    }
    model.addComponent(positionMotion);
    SimTK::State& state = model.initSystem();
    const auto& zmp = model.getComponent<ZeroMomentPointEstimator>("/zmp");

    std::vector<SimTK::RowVector> rows;
    for (const double t : times) {
        state.setTime(t);
        model.getSystem().prescribe(state);
        model.realizeAcceleration(state);
        SimTK::RowVector row(9);
        const Vec3 grf = zmp.getGroundReactionForce(state, "right");
        const Vec3 cop = zmp.getCenterOfPressure(state, "right");
        const Vec3 moment = zmp.getFreeMoment(state, "right");
        for (int k = 0; k < 3; ++k) {
            row[k] = grf[k];
            row[3 + k] = cop[k];
            row[6 + k] = moment[k];
        }
        rows.push_back(row);
    }
    return rows;
}

// Table with the given columns, in the given order, sampled at `times`.
// kind: "value" (coordinate names, rotations in degrees if `inDegrees`),
// "state value", "state speed" or "state accel" (absolute paths, radians).
struct Column {
    int coordinate; // -1 for an unrelated column.
    std::string kind;
};
TimeSeriesTable createTable(const std::vector<Column>& columns,
        const std::vector<double>& times, bool ikStyle) {
    std::vector<std::string> labels;
    for (const auto& column : columns) {
        if (column.coordinate < 0) {
            labels.push_back("unrelated_column");
        } else if (ikStyle) {
            labels.push_back(coordNames[column.coordinate]);
        } else {
            labels.push_back(jointPath + coordNames[column.coordinate] + "/" +
                             column.kind);
        }
    }
    TimeSeriesTable table;
    table.setColumnLabels(labels);
    for (const double t : times) {
        SimTK::RowVector row(static_cast<int>(columns.size()));
        for (int j = 0; j < static_cast<int>(columns.size()); ++j) {
            const int c = columns[j].coordinate;
            if (c < 0) {
                row[j] = 42.0;
                continue;
            }
            const Quadratic& q = motion[c];
            double v = columns[j].kind == "value"   ? q.value(t)
                       : columns[j].kind == "speed" ? q.speed(t)
                                                    : q.accel();
            // Rotational coordinates are the first three.
            if (ikStyle && c < 3) v *= SimTK_RADIAN_TO_DEGREE;
            row[j] = v;
        }
        table.appendRow(t, row);
    }
    if (ikStyle) table.addTableMetaData("inDegrees", std::string("yes"));
    return table;
}

void checkMatchesReference(const TimeSeriesTable& loads,
        const std::vector<SimTK::RowVector>& reference, double tolerance) {
    REQUIRE(loads.getNumRows() == reference.size());
    REQUIRE(loads.getNumColumns() == 9);
    for (std::size_t i = 0; i < reference.size(); ++i) {
        const auto row = loads.getRowAtIndex(i);
        for (int k = 0; k < 9; ++k) {
            CHECK_THAT(row[k], WithinAbs(reference[i][k], tolerance));
        }
    }
}

} // namespace

TEST_CASE("createZMPLoadsFromKinematics") {
    const Model model = createFootModel();
    const auto times = sampleTimes();
    const auto reference = referenceRows(model, times);

    // The motion must actually exercise the estimator.
    REQUIRE(reference[25][1] > 0);
    REQUIRE(std::abs(reference[25][7]) > 0);

    SECTION("IK-style table: coordinate names, degrees, shuffled, extra "
            "column") {
        const auto table =
                createTable({{5, "value"}, {-1, ""}, {1, "value"}, {3, "value"},
                                    {0, "value"}, {4, "value"}, {2, "value"}},
                        times, true);
        const auto loads =
                createZMPLoadsFromKinematics(model, TableProcessor(table));
        // Quintic natural splines reproduce quadratics exactly.
        checkMatchesReference(loads, reference, 1e-6);

        const auto& labels = loads.getColumnLabels();
        CHECK(labels[0] == "right_force_vx");
        CHECK(labels[3] == "right_force_px");
        CHECK(labels[8] == "right_torque_z");
        CHECK(loads.getTableMetaData<std::string>("inDegrees") == "no");
    }

    SECTION("Missing coordinate uses its default value") {
        // rz is always 0, which is also its default.
        const auto table =
                createTable({{0, "value"}, {1, "value"}, {3, "value"},
                                    {4, "value"}, {5, "value"}},
                        times, false);
        checkMatchesReference(
                createZMPLoadsFromKinematics(model, TableProcessor(table)),
                reference, 1e-6);
    }

    SECTION("Speed columns are ignored") {
        std::vector<Column> columns;
        for (int c = 0; c < 6; ++c) {
            columns.push_back({c, "value"});
            columns.push_back({c, "speed"});
        }
        const auto table = createTable(columns, times, false);
        checkMatchesReference(
                createZMPLoadsFromKinematics(model, TableProcessor(table)),
                reference, 1e-6);
    }

    SECTION("Optional low-pass filter keeps the time range") {
        std::vector<Column> columns;
        for (int c = 0; c < 6; ++c) columns.push_back({c, "value"});
        const auto table = createTable(columns, times, false);
        const auto loads =
                createZMPLoadsFromKinematics(model, TableProcessor(table), 6.0);
        CHECK(loads.getNumRows() == times.size());
        CHECK_THAT(loads.getIndependentColumn().front(),
                WithinAbs(times.front(), 1e-12));
        CHECK_THAT(loads.getIndependentColumn().back(),
                WithinAbs(times.back(), 1e-12));
        // A smooth motion is barely changed away from the ends.
        CHECK_THAT(
                loads.getRowAtIndex(25)[1], WithinAbs(reference[25][1], 1.0));
    }

    SECTION("No matching columns throws") {
        const auto table = createTable({{-1, ""}}, times, false);
        CHECK_THROWS(
                createZMPLoadsFromKinematics(model, TableProcessor(table)));
    }
}

TEST_CASE("createZMPLoadsFromStates") {
    const Model model = createFootModel();
    const auto times = sampleTimes();
    const auto reference = referenceRows(model, times);

    SECTION("Values, speeds and accelerations, in any order") {
        std::vector<Column> columns{{-1, ""}};
        for (int c = 5; c >= 0; --c) {
            columns.push_back({c, "accel"});
            columns.push_back({c, "value"});
            columns.push_back({c, "speed"});
        }
        const auto loads = createZMPLoadsFromStates(
                model, TableProcessor(createTable(columns, times, false)));
        checkMatchesReference(loads, reference, 1e-8);
    }

    SECTION("Values and speeds: accelerations from the speed splines") {
        std::vector<Column> columns;
        for (int c = 0; c < 6; ++c) {
            columns.push_back({c, "value"});
            columns.push_back({c, "speed"});
        }
        const auto loads = createZMPLoadsFromStates(
                model, TableProcessor(createTable(columns, times, false)));
        checkMatchesReference(loads, reference, 1e-6);
    }

    SECTION("Accelerations for only some coordinates throws") {
        std::vector<Column> columns;
        for (int c = 0; c < 6; ++c) {
            columns.push_back({c, "value"});
            columns.push_back({c, "speed"});
        }
        columns.push_back({3, "accel"});
        CHECK_THROWS(createZMPLoadsFromStates(
                model, TableProcessor(createTable(columns, times, false))));
    }
}

TEST_CASE("ZMP utilities estimator lookup") {
    Model model = createFootModel();
    const auto times = sampleTimes();
    std::vector<Column> columns;
    for (int c = 0; c < 6; ++c) columns.push_back({c, "value"});
    const TableProcessor table(createTable(columns, times, false));

    // A second estimator makes the lookup ambiguous.
    const auto& foot = model.getBodySet().get("foot");
    auto* second = new ZeroMomentPointEstimator("ground_foot");
    second->setName("zmp2");
    auto* side = new ContactSide("other", foot);
    side->adoptContactPoint(new ContactPoint(foot, Vec3(0, -halfHeight, 0)));
    second->adoptContactSide(side);
    model.addComponent(second);
    model.finalizeConnections();

    CHECK_THROWS(createZMPLoadsFromKinematics(model, table));
    const auto loads = createZMPLoadsFromKinematics(model, table, -1, "/zmp2");
    CHECK(loads.getColumnLabels()[0] == "other_force_vx");
}
