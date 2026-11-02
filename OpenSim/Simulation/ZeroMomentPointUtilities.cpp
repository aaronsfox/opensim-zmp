/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ZeroMomentPointUtilities.cpp                    *
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

#include "ZeroMomentPointUtilities.h"

#include "Model/ZeroMomentPointEstimator.h"
#include "PositionMotion.h"
#include "SimulationUtilities.h"
#include <algorithm>
#include <format>
#include <memory>
#include <unordered_map>
#include <vector>

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/TableUtilities.h>

namespace OpenSim {

namespace {

constexpr int splineDegree = 5;

// Join strings for messages.
std::string join(const std::vector<std::string>& items) {
    std::string joined;
    for (const auto& item : items) {
        if (!joined.empty()) joined += ", ";
        joined += item;
    }
    return joined;
}

// Apply the processing steps shared by both utilities (see the header).
TimeSeriesTable processInputTable(const Model& model,
        const TableProcessor& input, double lowpassCutoffFrequency) {
    TimeSeriesTable table = input.process(&model);
    OPENSIM_THROW_IF(table.getNumRows() == 0, Exception,
            "Expected the input table to have rows, but it is empty.");

    // Inverse Kinematics output uses coordinate names.
    auto labels = table.getColumnLabels();
    updateStateLabels40(model, labels);
    table.setColumnLabels(labels);

    // Convert only if the table says it is in degrees. Tables created in
    // code often have no inDegrees metadata; treat them as radians.
    if (table.hasTableMetaDataKey("inDegrees")) {
        if (TableUtilities::isInDegrees(table)) {
            model.getSimbodyEngine().convertDegreesToRadians(table);
        }
    } else {
        log_warn("The input table has no 'inDegrees' metadata; assuming "
                 "rotational coordinates are in radians.");
    }

    if (lowpassCutoffFrequency > 0) {
        TabOpLowPassFilter(lowpassCutoffFrequency).operate(table, &model);
    }
    return table;
}

// Find the estimator at `path`, or the only one in the model.
const ZeroMomentPointEstimator& findEstimator(
        const Model& model, const std::string& path) {
    if (!path.empty()) {
        return model.getComponent<ZeroMomentPointEstimator>(path);
    }
    const ZeroMomentPointEstimator* found = nullptr;
    int count = 0;
    for (const auto& estimator :
            model.getComponentList<ZeroMomentPointEstimator>()) {
        found = &estimator;
        ++count;
    }
    OPENSIM_THROW_IF(count != 1, Exception,
            std::format("Expected exactly one ZeroMomentPointEstimator in "
                        "model '{}' when no estimator path is given, but "
                        "found {}.",
                    model.getName(), count));
    return *found;
}

// Output table with 9 columns per side (see the header).
TimeSeriesTable createOutputTable(const ZeroMomentPointEstimator& estimator) {
    std::vector<std::string> labels;
    for (int i = 0; i < estimator.getNumContactSides(); ++i) {
        const std::string& side = estimator.getContactSide(i).getName();
        for (const char* suffix : {"_force_vx", "_force_vy", "_force_vz",
                     "_force_px", "_force_py", "_force_pz", "_torque_x",
                     "_torque_y", "_torque_z"}) {
            labels.push_back(side + suffix);
        }
    }
    TimeSeriesTable table;
    table.setColumnLabels(labels);
    table.addTableMetaData("inDegrees", std::string("no"));
    return table;
}

void appendResult(TimeSeriesTable& table, double time,
        const ZeroMomentPointEstimator::ZMPResult& result) {
    SimTK::RowVector row(static_cast<int>(table.getColumnLabels().size()), 0.0);
    int col = 0;
    for (const auto& side : result.sides) {
        for (const SimTK::Vec3* vec : {&side.groundReactionForce,
                     &side.centerOfPressure, &side.freeMoment}) {
            for (int k = 0; k < 3; ++k) row[col++] = (*vec)[k];
        }
    }
    table.appendRow(time, row);
}

// A quintic interpolating spline of one column of `table`.
GCVSpline createSpline(const TimeSeriesTable& table, const std::string& label) {
    const auto& times = table.getIndependentColumn();
    const auto column = table.getDependentColumn(label);
    std::vector<double> values(column.size());
    for (int i = 0; i < column.size(); ++i) values[i] = column[i];
    OPENSIM_THROW_IF(static_cast<int>(times.size()) <= splineDegree, Exception,
            std::format("Expected at least {} rows to fit splines, but the "
                        "table has {}.",
                    splineDegree + 1, times.size()));
    return GCVSpline(splineDegree, static_cast<int>(times.size()), times.data(),
            values.data(), label, 0.0);
}

bool endsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) ==
                   0;
}

} // namespace

//=============================================================================
// FROM KINEMATICS
//=============================================================================
TimeSeriesTable createZMPLoadsFromKinematics(Model model,
        const TableProcessor& kinematics, double lowpassCutoffFrequency,
        const std::string& estimatorPath) {
    // A system is needed to process the table (state names, degrees).
    model.initSystem();
    const TimeSeriesTable table =
            processInputTable(model, kinematics, lowpassCutoffFrequency);

    // Prescribe every coordinate: with a spline of its values if present,
    // otherwise at its default value.
    auto motion = std::make_unique<PositionMotion>("zmp_position_motion");
    std::vector<std::string> used;
    std::vector<std::string> missing;
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        const std::string label = coord.getAbsolutePathString() + "/value";
        if (table.hasColumn(label)) {
            motion->setPositionForCoordinate(coord, createSpline(table, label));
            used.push_back(label);
        } else {
            motion->setPositionForCoordinate(
                    coord, Constant(coord.getDefaultValue()));
            missing.push_back(coord.getAbsolutePathString());
        }
    }
    OPENSIM_THROW_IF(used.empty(), Exception,
            "Expected the kinematics table to contain coordinate values, but "
            "no column matches a coordinate in the model.");
    if (!missing.empty()) {
        log_warn("createZMPLoadsFromKinematics: no values for coordinate(s) "
                 "{}; using their default values.",
                join(missing));
    }
    std::vector<std::string> ignored;
    for (const auto& label : table.getColumnLabels()) {
        if (std::find(used.begin(), used.end(), label) == used.end()) {
            ignored.push_back(label);
        }
    }
    if (!ignored.empty()) {
        log_info("createZMPLoadsFromKinematics: ignoring column(s) {}.",
                join(ignored));
    }

    model.addComponent(motion.release());
    SimTK::State& state = model.initSystem();
    const auto& estimator = findEstimator(model, estimatorPath);

    TimeSeriesTable loads = createOutputTable(estimator);
    for (const double time : table.getIndependentColumn()) {
        state.setTime(time);
        model.getSystem().prescribe(state);
        model.realizeAcceleration(state);
        appendResult(
                loads, time, estimator.calcZMPResult(state, state.getUDot()));
    }
    return loads;
}

//=============================================================================
// FROM STATES
//=============================================================================
TimeSeriesTable createZMPLoadsFromStates(Model model,
        const TableProcessor& states, double lowpassCutoffFrequency,
        const std::string& estimatorPath) {
    SimTK::State& state = model.initSystem();
    const TimeSeriesTable table =
            processInputTable(model, states, lowpassCutoffFrequency);
    const auto& estimator = findEstimator(model, estimatorPath);

    // Y = [q, u, z], so a speed's mobility index is its Y index minus nq.
    const auto yIndices = createSystemYIndexMap(model);
    const int nq = state.getNQ();
    const int nu = state.getNU();

    // Columns that are state variables, and acceleration columns.
    std::vector<std::pair<int, int>> stateColumns; // (column, Y index)
    std::vector<std::pair<int, int>> accelColumns; // (column, U index)
    std::vector<std::string> ignored;
    const auto& labels = table.getColumnLabels();
    for (int col = 0; col < static_cast<int>(labels.size()); ++col) {
        const std::string& label = labels[col];
        if (const auto it = yIndices.find(label); it != yIndices.end()) {
            stateColumns.emplace_back(col, it->second);
            continue;
        }
        if (endsWith(label, "/accel")) {
            const std::string speed =
                    label.substr(0, label.size() - 6) + "/speed";
            if (const auto it = yIndices.find(speed); it != yIndices.end()) {
                accelColumns.emplace_back(col, it->second - nq);
                continue;
            }
        }
        ignored.push_back(label);
    }
    if (!ignored.empty()) {
        log_info("createZMPLoadsFromStates: ignoring column(s) {}.",
                join(ignored));
    }

    // Coordinate coverage.
    std::vector<std::string> missingValues;
    std::vector<std::string> missingSpeeds;
    std::vector<std::string> missingAccels;
    std::vector<std::pair<std::string, int>> speedColumns; // (label, U index)
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        const std::string path = coord.getAbsolutePathString();
        if (!table.hasColumn(path + "/value")) missingValues.push_back(path);
        if (table.hasColumn(path + "/speed")) {
            speedColumns.emplace_back(
                    path + "/speed", yIndices.at(path + "/speed") - nq);
        } else {
            missingSpeeds.push_back(path);
        }
        if (!table.hasColumn(path + "/accel")) missingAccels.push_back(path);
    }
    OPENSIM_THROW_IF(missingValues.size() == static_cast<std::size_t>(
                                                     model.getNumCoordinates()),
            Exception,
            "Expected the states table to contain coordinate values, but no "
            "column matches a coordinate value in the model.");
    if (!missingValues.empty()) {
        log_warn("createZMPLoadsFromStates: no values for coordinate(s) {}; "
                 "using their default values.",
                join(missingValues));
    }
    if (!missingSpeeds.empty()) {
        log_warn("createZMPLoadsFromStates: no speeds for coordinate(s) {}; "
                 "using their default values (0) and zero accelerations.",
                join(missingSpeeds));
    }
    const bool useAccelColumns = !accelColumns.empty();
    OPENSIM_THROW_IF(useAccelColumns && !missingAccels.empty(), Exception,
            std::format("Expected accelerations ('<coordinate path>/accel') "
                        "for every coordinate or none, but they are missing "
                        "for {}.",
                    join(missingAccels)));

    // Without acceleration columns, differentiate splines of the speeds.
    std::vector<std::pair<GCVSpline, int>> speedSplines;
    if (!useAccelColumns) {
        log_info("createZMPLoadsFromStates: no acceleration columns; "
                 "computing accelerations from splines of the speeds.");
        for (const auto& [label, uIndex] : speedColumns) {
            speedSplines.emplace_back(createSpline(table, label), uIndex);
        }
    }

    TimeSeriesTable loads = createOutputTable(estimator);
    const auto& times = table.getIndependentColumn();
    const std::vector<int> firstDerivative{0};
    SimTK::Vector udot(nu, 0.0);
    for (int row = 0; row < static_cast<int>(times.size()); ++row) {
        const auto values = table.getRowAtIndex(row);
        state.setTime(times[row]);
        for (const auto& [col, yIndex] : stateColumns) {
            state.updY()[yIndex] = values[col];
        }
        if (useAccelColumns) {
            for (const auto& [col, uIndex] : accelColumns) {
                udot[uIndex] = values[col];
            }
        } else {
            const SimTK::Vector time(1, times[row]);
            for (const auto& [spline, uIndex] : speedSplines) {
                udot[uIndex] = spline.calcDerivative(firstDerivative, time);
            }
        }
        model.realizeVelocity(state);
        appendResult(loads, times[row], estimator.calcZMPResult(state, udot));
    }
    return loads;
}

} // namespace OpenSim
