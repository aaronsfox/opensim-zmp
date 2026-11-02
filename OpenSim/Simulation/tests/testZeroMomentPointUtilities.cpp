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
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <OpenSim/Common/LogSink.h>
#include <OpenSim/Common/Logger.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/PositionMotion.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/ZeroMomentPointUtilities.h>

using namespace OpenSim;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using SimTK::Vec3;

namespace {

constexpr double mass = 10.0;
constexpr double halfHeight = 0.05;
const std::array<std::string, 6> coordNames{"rx", "ry", "rz", "tx", "ty", "tz"};
const std::string jointPath = "/jointset/ground_foot/";

// Each coordinate is c2 t^2 + c1 t + c0.
struct Quadratic {
    double c2, c1, c0;
    double value(double t) const { return c2 * t * t + c1 * t + c0; }
    double speed(double t) const { return 2 * c2 * t + c1; }
    double accel() const { return 2 * c2; }
};
using Motion = std::array<Quadratic, 6>;

// The foot slides and accelerates along x, drifts along z and spins up about
// the vertical axis, with its sole on the ground (single support, "right").
const Motion stanceMotion{Quadratic{0, 0, 0}, Quadratic{1.5, 0, 0},
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
    for (int i = 0; i < 6; ++i) {
        joint->upd_coordinates(i).setName(coordNames[i]);
    }
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

// Add a PositionMotion that prescribes `motion` exactly.
void addExactMotion(Model& model, const Motion& motion) {
    model.finalizeFromProperties();
    auto* positionMotion = new PositionMotion("exact_motion");
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
}

// Reference: the estimator evaluated with the exact motion, as a 9-column row
// per time.
std::vector<SimTK::RowVector> referenceRows(
        Model model, const Motion& motion, const std::vector<double>& times) {
    addExactMotion(model, motion);
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

// A column of a test table: a coordinate's "value", "speed" or "accel", or
// an unrelated column (coordinate = -1).
struct Column {
    int coordinate;
    std::string kind;
};

// With `ikStyle`, labels are coordinate names (values only) and rotations
// are in degrees; otherwise labels are absolute state paths in radians,
// without inDegrees metadata.
TimeSeriesTable createTable(const Motion& motion,
        const std::vector<Column>& columns, const std::vector<double>& times,
        bool ikStyle = false) {
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

std::vector<Column> allCoordinates(const std::vector<std::string>& kinds) {
    std::vector<Column> columns;
    for (int c = 0; c < 6; ++c) {
        for (const auto& kind : kinds) columns.push_back({c, kind});
    }
    return columns;
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

// Table with absolute state paths (radians, no inDegrees metadata), with
// coordinate values from `value(coordinate, frame, time)` and, if
// `withSpeeds`, zero speeds.
TimeSeriesTable createCustomTable(const std::vector<double>& times,
        const std::function<double(int, int, double)>& value, bool withSpeeds) {
    std::vector<std::string> labels;
    for (int c = 0; c < 6; ++c) {
        labels.push_back(jointPath + coordNames[c] + "/value");
        if (withSpeeds) labels.push_back(jointPath + coordNames[c] + "/speed");
    }
    TimeSeriesTable table;
    table.setColumnLabels(labels);
    for (int frame = 0; frame < static_cast<int>(times.size()); ++frame) {
        SimTK::RowVector row(static_cast<int>(labels.size()), 0.0);
        int col = 0;
        for (int c = 0; c < 6; ++c) {
            row[col++] = value(c, frame, times[frame]);
            if (withSpeeds) row[col++] = 0.0;
        }
        table.appendRow(times[frame], row);
    }
    return table;
}

// Number of separate periods with a vertical ground reaction.
int countLoadedPeriods(const TimeSeriesTable& loads) {
    int periods = 0;
    bool loaded = false;
    for (std::size_t i = 0; i < loads.getNumRows(); ++i) {
        const bool now = loads.getRowAtIndex(i)[1] > 0;
        if (now && !loaded) ++periods;
        loaded = now;
    }
    return periods;
}

// Collects log messages.
class CapturingLogSink : public LogSink {
public:
    std::vector<std::string> messages;

protected:
    void sinkImpl(const std::string& msg) override { messages.push_back(msg); }
};

struct LogCapture {
    std::shared_ptr<CapturingLogSink> sink =
            std::make_shared<CapturingLogSink>();
    LogCapture() { Logger::addSink(sink); }
    ~LogCapture() { Logger::removeSink(sink); }
    bool contains(const std::string& text) const {
        for (const auto& message : sink->messages) {
            if (message.find(text) != std::string::npos) return true;
        }
        return false;
    }
};

} // namespace

TEST_CASE("createZMPLoadsFromStates chooses the method from the columns") {
    const Model model = createFootModel();
    const auto times = sampleTimes();
    const auto reference = referenceRows(model, stanceMotion, times);

    // The motion must actually exercise the estimator.
    REQUIRE(reference[25][1] > 0);
    REQUIRE(std::abs(reference[25][7]) > 0);

    SECTION("Values only (IK style: names, degrees, shuffled, extra column)") {
        LogCapture log;
        const auto table = createTable(stanceMotion,
                {{5, "value"}, {-1, ""}, {1, "value"}, {3, "value"},
                        {0, "value"}, {4, "value"}, {2, "value"}},
                times, true);
        const auto loads = createZMPLoadsFromStates(model, table);
        // Quintic natural splines reproduce quadratics exactly.
        checkMatchesReference(loads, reference, 1e-6);
        CHECK(log.contains("PositionMotion"));
        CHECK(log.contains("unrelated_column"));

        const auto& labels = loads.getColumnLabels();
        CHECK(labels[0] == "right_force_vx");
        CHECK(labels[3] == "right_force_px");
        CHECK(labels[8] == "right_torque_z");
        CHECK(loads.getTableMetaData<std::string>("inDegrees") == "no");
    }

    SECTION("Values and speeds: accelerations from the speed splines") {
        LogCapture log;
        const auto loads = createZMPLoadsFromStates(
                model, createTable(stanceMotion,
                               allCoordinates({"value", "speed"}), times));
        checkMatchesReference(loads, reference, 1e-6);
        CHECK(log.contains("splines of the speeds"));
    }

    SECTION("Values, speeds and accelerations, in any order") {
        LogCapture log;
        std::vector<Column> columns{{-1, ""}};
        for (int c = 5; c >= 0; --c) {
            for (const char* kind : {"accel", "value", "speed"}) {
                columns.push_back({c, kind});
            }
        }
        const auto loads = createZMPLoadsFromStates(
                model, createTable(stanceMotion, columns, times));
        checkMatchesReference(loads, reference, 1e-8);
        CHECK(log.contains("accelerations from the table"));
    }

    SECTION("Accelerations for some coordinates: falls back to speeds") {
        LogCapture log;
        auto columns = allCoordinates({"value", "speed"});
        columns.push_back({3, "accel"});
        const auto loads = createZMPLoadsFromStates(
                model, createTable(stanceMotion, columns, times));
        checkMatchesReference(loads, reference, 1e-6);
        CHECK(log.contains("ignoring all acceleration columns"));
    }

    SECTION("Speeds for some coordinates: falls back to values only") {
        LogCapture log;
        auto columns = allCoordinates({"value"});
        columns.push_back({3, "speed"});
        const auto loads = createZMPLoadsFromStates(
                model, createTable(stanceMotion, columns, times));
        checkMatchesReference(loads, reference, 1e-6);
        CHECK(log.contains("ignoring all speed columns"));
    }

    SECTION("Missing coordinate uses its default value, with a warning") {
        LogCapture log;
        // rz is always 0, which is also its default.
        const auto loads = createZMPLoadsFromStates(
                model, createTable(stanceMotion,
                               {{0, "value"}, {1, "value"}, {3, "value"},
                                       {4, "value"}, {5, "value"}},
                               times));
        checkMatchesReference(loads, reference, 1e-6);
        CHECK(log.contains("/jointset/ground_foot/rz"));
    }

    SECTION("No coordinate columns throws") {
        CHECK_THROWS(createZMPLoadsFromStates(
                model, createTable(stanceMotion, {{-1, ""}}, times)));
    }
}

TEST_CASE("createZMPLoadsFromStates options") {
    const Model model = createFootModel();
    const auto times = sampleTimes();
    const auto table =
            createTable(stanceMotion, allCoordinates({"value"}), times);

    SECTION("Filtering is optional, last, and keeps the time range") {
        const auto reference = referenceRows(model, stanceMotion, times);
        const auto loads = createZMPLoadsFromStates(model, table, "", 6.0);
        CHECK(loads.getNumRows() == times.size());
        CHECK_THAT(loads.getIndependentColumn().front(),
                WithinAbs(times.front(), 1e-12));
        CHECK_THAT(loads.getIndependentColumn().back(),
                WithinAbs(times.back(), 1e-12));
        // A smooth motion is barely changed away from the ends.
        CHECK_THAT(
                loads.getRowAtIndex(25)[1], WithinAbs(reference[25][1], 1.0));
    }

    SECTION("Estimator lookup by path, or the only one") {
        Model twoEstimators = model;
        const auto& foot = twoEstimators.getBodySet().get("foot");
        auto* second = new ZeroMomentPointEstimator("ground_foot");
        second->setName("zmp2");
        auto* side = new ContactSide("other", foot);
        side->adoptContactPoint(
                new ContactPoint(foot, Vec3(0, -halfHeight, 0)));
        second->adoptContactSide(side);
        twoEstimators.addComponent(second);
        twoEstimators.finalizeConnections();

        CHECK_THROWS(createZMPLoadsFromStates(twoEstimators, table));
        const auto loads =
                createZMPLoadsFromStates(twoEstimators, table, "/zmp2");
        CHECK(loads.getColumnLabels()[0] == "other_force_vx");
    }
}

TEST_CASE("createZMPLoadsFromStates reports periods without loads") {
    const Model model = createFootModel();
    const auto times = sampleTimes();

    // The foot accelerates upward from rest on the ground; the sole height
    // 0.25 t^2 passes the 8 mm threshold between t = 0.17 and 0.18 s.
    Motion liftOff = stanceMotion;
    liftOff[4] = Quadratic{0.25, 0, halfHeight};

    LogCapture log;
    const auto loads = createZMPLoadsFromStates(
            model, createTable(liftOff, allCoordinates({"value"}), times));

    CHECK(log.contains("no contact detected from t = 0.18 to 0.5 s"));
    CHECK(loads.getRowAtIndex(10)[1] > 0);
    CHECK(loads.getRowAtIndex(30)[1] == 0);
}

TEST_CASE("createZMPLoadsFromStates contact hysteresis") {
    Model model = createFootModel();
    std::vector<double> times;
    for (int i = 0; i <= 100; ++i) times.push_back(0.01 * i);

    // The sole height oscillates around the 8 mm on threshold (5.5 to 11.5
    // mm, centred off the threshold so no sample lands exactly on it), so
    // contact without hysteresis switches on and off three times.
    const auto table = createCustomTable(
            times,
            [](int c, int, double t) {
                if (c == 4) {
                    return halfHeight + 0.0085 +
                           0.003 * std::sin(2 * SimTK::Pi * 3 * t);
                }
                return stanceMotion[c].value(0);
            },
            false);

    CHECK(countLoadedPeriods(createZMPLoadsFromStates(model, table)) == 3);

    // With an off threshold above the highest point (12 mm), contact never
    // ends once it starts.
    auto& side = model.updComponent<ContactSide>("/zmp/right");
    for (int j = 0; j < side.getNumContactPoints(); ++j) {
        side.updContactPoint(j).setDistanceOffThreshold(0.012);
    }
    model.finalizeConnections();
    LogCapture log;
    const auto loads = createZMPLoadsFromStates(model, table);
    CHECK(countLoadedPeriods(loads) == 1);
    CHECK(loads.getRowAtIndex(100)[1] > 0);
    CHECK(log.contains("using contact hysteresis"));
}

TEST_CASE("createZMPLoadsFromStates minimum contact duration") {
    Model model = createFootModel();
    std::vector<double> times;
    for (int i = 0; i <= 100; ++i) times.push_back(0.01 * i);

    // The foot is lifted (2 cm) at frames 0-1 (a short period at the start),
    // 20-21 (a 0.02 s gap), and 60-100 except a 0.02 s touch at 80-81.
    // States: values and zero speeds, so the foot is static when in contact.
    auto lifted = [](int frame) {
        return frame <= 1 || frame == 20 || frame == 21 ||
               (frame >= 60 && frame != 80 && frame != 81);
    };
    const auto table = createCustomTable(
            times,
            [&](int c, int frame, double) {
                if (c == 4) return halfHeight + (lifted(frame) ? 0.02 : 0.0);
                return stanceMotion[c].value(0);
            },
            true);

    // Without a minimum duration, every period is kept.
    {
        const auto loads = createZMPLoadsFromStates(model, table);
        CHECK(countLoadedPeriods(loads) == 3);
        CHECK(loads.getRowAtIndex(20)[1] == 0);
        CHECK(loads.getRowAtIndex(80)[1] > 0);
    }

    model.updComponent<ContactSide>("/zmp/right")
            .setMinimumContactDuration(0.05);
    model.finalizeConnections();
    LogCapture log;
    const auto loads = createZMPLoadsFromStates(model, table);
    CHECK(countLoadedPeriods(loads) == 1);
    // The gap is filled and the touch removed; the start period is kept.
    CHECK(loads.getRowAtIndex(0)[1] == 0);
    CHECK(loads.getRowAtIndex(1)[1] == 0);
    for (int frame = 2; frame < 60; ++frame) {
        CHECK(loads.getRowAtIndex(frame)[1] > 0);
    }
    for (int frame = 60; frame <= 100; ++frame) {
        CHECK(loads.getRowAtIndex(frame)[1] == 0);
    }
    CHECK(log.contains("removed 1 contact period(s) and 1 no-contact "
                       "period(s) shorter than 0.05 s"));
}

TEST_CASE("createZMPExternalLoads") {
    const Model model = createFootModel();
    const auto times = sampleTimes();

    const auto table =
            createTable(stanceMotion, allCoordinates({"value"}), times);
    const auto grf = createZMPLoadsFromStates(model, table);
    const std::string dataFile = "testZeroMomentPointUtilities_grf.sto";
    STOFileAdapter::write(grf, dataFile);

    ExternalLoads loads = createZMPExternalLoads(model, dataFile);
    REQUIRE(loads.getSize() == 1);
    const ExternalForce& force = loads.get(0);
    CHECK(force.getName() == "right");
    CHECK(force.get_applied_to_body() == "foot");
    CHECK(force.get_force_expressed_in_body() == "ground");
    CHECK(force.get_point_expressed_in_body() == "ground");

    // The identifiers match the table's column labels.
    for (const std::string& identifier : {force.get_force_identifier(),
                 force.get_point_identifier(), force.get_torque_identifier()}) {
        for (const char* axis : {"x", "y", "z"}) {
            CHECK(grf.hasColumn(identifier + axis));
        }
    }

    SECTION("Applying the loads balances the motion") {
        // With the estimated loads applied, inverse dynamics needs no
        // residual at the root joint.
        const std::string loadsFile = "testZeroMomentPointUtilities_loads.xml";
        loads.print(loadsFile);
        Model withLoads = model;
        withLoads.addModelComponent(new ExternalLoads(loadsFile, true));
        addExactMotion(withLoads, stanceMotion);
        SimTK::State& state = withLoads.initSystem();
        InverseDynamicsSolver solver(withLoads);
        for (int i = 5; i < static_cast<int>(times.size()) - 5; ++i) {
            state.setTime(times[i]);
            withLoads.getSystem().prescribe(state);
            withLoads.realizeAcceleration(state);
            const SimTK::Vector residual = solver.solve(state, state.getUDot());
            for (int k = 0; k < residual.size(); ++k) {
                CHECK_THAT(residual[k], WithinAbs(0, 1e-6));
            }
        }
    }
}
