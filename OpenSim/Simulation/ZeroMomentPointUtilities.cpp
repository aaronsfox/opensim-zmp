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

#include "Model/ExternalForce.h"
#include "Model/ZeroMomentPointEstimator.h"
#include "PositionMotion.h"
#include "SimbodyEngine/Body.h"
#include "SimulationUtilities.h"
#include <algorithm>
#include <format>
#include <functional>
#include <memory>
#include <vector>

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/TableUtilities.h>

namespace OpenSim {

namespace {

constexpr int splineDegree = 5;

using ZMPResult = ZeroMomentPointEstimator::ZMPResult;
using ContactMode = ZeroMomentPointEstimator::ContactMode;

// Join strings for messages.
std::string join(const std::vector<std::string>& items) {
    std::string joined;
    for (const auto& item : items) {
        if (!joined.empty()) joined += ", ";
        joined += item;
    }
    return joined;
}

bool endsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) ==
                   0;
}

// Apply the processing steps described in the header.
TimeSeriesTable processInputTable(const Model& model,
        const TableProcessor& input, double lowpassCutoffFrequency) {
    TimeSeriesTable table = input.process(&model);
    OPENSIM_THROW_IF(table.getNumRows() == 0, Exception,
            "Expected the input table to have rows, but it is empty.");

    // Inverse Kinematics output uses coordinate names.
    auto labels = table.getColumnLabels();
    updateStateLabels40(model, labels);
    table.setColumnLabels(labels);

    // Tables created in code often have no inDegrees metadata; treat them as
    // radians.
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

// Column labels and ExternalForce identifiers for a side. Both come from
// here so the table and the ExternalLoads always agree.
std::string forceIdentifier(const std::string& side) {
    return side + "_force_v";
}
std::string pointIdentifier(const std::string& side) {
    return side + "_force_p";
}
std::string torqueIdentifier(const std::string& side) {
    return side + "_torque_";
}

TimeSeriesTable createOutputTable(const ZeroMomentPointEstimator& estimator) {
    std::vector<std::string> labels;
    for (int i = 0; i < estimator.getNumContactSides(); ++i) {
        const std::string& side = estimator.getContactSide(i).getName();
        for (const std::string& prefix : {forceIdentifier(side),
                     pointIdentifier(side), torqueIdentifier(side)}) {
            for (const char* axis : {"x", "y", "z"}) {
                labels.push_back(prefix + axis);
            }
        }
    }
    TimeSeriesTable table;
    table.setColumnLabels(labels);
    table.addTableMetaData("inDegrees", std::string("no"));
    return table;
}

void appendResult(
        TimeSeriesTable& table, double time, const ZMPResult& result) {
    // Size from the labels: an empty table reports zero columns.
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
    OPENSIM_THROW_IF(static_cast<int>(times.size()) <= splineDegree, Exception,
            std::format("Expected at least {} rows to fit splines, but the "
                        "table has {}.",
                    splineDegree + 1, times.size()));
    const auto column = table.getDependentColumn(label);
    std::vector<double> values(column.size());
    for (int i = 0; i < column.size(); ++i) values[i] = column[i];
    return GCVSpline(splineDegree, static_cast<int>(times.size()), times.data(),
            values.data(), label, 0.0);
}

// Reports periods (runs of consecutive frames) in which some side has no
// ground reaction, with one warning per period, as the frames are processed.
class InvalidPeriodReporter {
public:
    void add(double time, const ZMPResult& result) {
        const std::string reason = describe(result);
        if (reason != m_reason) {
            flush();
            m_reason = reason;
            m_start = time;
            m_count = 0;
        }
        m_end = time;
        ++m_count;
    }
    void flush() {
        if (m_reason.empty() || m_count == 0) return;
        const std::string when =
                m_count == 1 ? std::format("at t = {:.4g} s", m_start)
                             : std::format("from t = {:.4g} to {:.4g} s ({} "
                                           "frames)",
                                       m_start, m_end, m_count);
        log_warn("createZMPLoadsFromStates: {} {}; ground reactions are zero "
                 "for those frames.",
                m_reason, when);
        m_count = 0;
    }

private:
    static std::string describe(const ZMPResult& result) {
        switch (result.mode) {
        case ContactMode::NoContact: return "no contact detected";
        case ContactMode::Bilateral:
            return "double support (both sides in contact; splitting the "
                   "load is not yet implemented)";
        case ContactMode::Unilateral:
            for (const auto& side : result.sides) {
                if (side.isInContact && !side.isValid) {
                    return "contact detected but the vertical force is below "
                           "contact_force_threshold";
                }
            }
            return "";
        }
        return "";
    }

    std::string m_reason;
    double m_start = 0;
    double m_end = 0;
    int m_count = 0;
};

// Removes contact (true) and no-contact (false) periods shorter than
// `minimumDuration` from `flags`, shortest first. A removed period takes the
// status of its neighbours and merges with them. Periods at the start or end
// are kept, since the trial may cut them short. The duration of a period is
// from its first frame to the first frame of the next period. Returns the
// number of contact and no-contact periods removed.
std::pair<int, int> removeShortPeriods(std::vector<bool>& flags,
        const std::vector<double>& times, double minimumDuration) {
    struct Period {
        int start, end; // First and last frame (inclusive).
        bool inContact;
    };
    std::vector<Period> periods;
    for (int row = 0; row < static_cast<int>(flags.size()); ++row) {
        if (periods.empty() || periods.back().inContact != flags[row]) {
            periods.push_back({row, row, flags[row]});
        } else {
            periods.back().end = row;
        }
    }

    int removedContacts = 0;
    int removedGaps = 0;
    while (periods.size() >= 3) {
        // The shortest interior period.
        int shortest = -1;
        double shortestDuration = minimumDuration;
        for (int p = 1; p < static_cast<int>(periods.size()) - 1; ++p) {
            const double duration =
                    times[periods[p].end + 1] - times[periods[p].start];
            if (duration < shortestDuration) {
                shortest = p;
                shortestDuration = duration;
            }
        }
        if (shortest < 0) break;
        (periods[shortest].inContact ? removedContacts : removedGaps) += 1;
        // Merge with both neighbours, which share the opposite status.
        periods[shortest - 1].end = periods[shortest + 1].end;
        periods.erase(
                periods.begin() + shortest, periods.begin() + shortest + 2);
    }

    for (const auto& period : periods) {
        for (int row = period.start; row <= period.end; ++row) {
            flags[row] = period.inContact;
        }
    }
    return {removedContacts, removedGaps};
}

// Which columns the table provides for the coordinates that have values.
enum class Method { ValuesOnly, ValuesAndSpeeds, ValuesSpeedsAndAccels };

} // namespace

//=============================================================================
// GROUND REACTIONS
//=============================================================================
TimeSeriesTable createZMPLoadsFromStates(Model model,
        const TableProcessor& states, const std::string& estimatorPath,
        double lowpassCutoffFrequency) {
    // A system is needed to process the table (state names, degrees).
    model.initSystem();
    const TimeSeriesTable table =
            processInputTable(model, states, lowpassCutoffFrequency);

    // Coordinate coverage.
    std::vector<const Coordinate*> withValues;
    std::vector<std::string> missingValues;
    std::vector<std::string> missingSpeeds;
    std::vector<std::string> missingAccels;
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        const std::string path = coord.getAbsolutePathString();
        if (!table.hasColumn(path + "/value")) {
            missingValues.push_back(path);
            continue;
        }
        withValues.push_back(&coord);
        if (!table.hasColumn(path + "/speed")) missingSpeeds.push_back(path);
        if (!table.hasColumn(path + "/accel")) missingAccels.push_back(path);
    }
    OPENSIM_THROW_IF(withValues.empty(), Exception,
            "Expected the states table to contain coordinate values, but no "
            "column matches a coordinate in the model.");
    if (!missingValues.empty()) {
        log_warn("createZMPLoadsFromStates: no values for coordinate(s) {}; "
                 "using their default values and zero speeds.",
                join(missingValues));
    }

    // Choose the method from the columns available (see the header).
    Method method = Method::ValuesOnly;
    if (missingSpeeds.empty()) {
        method = missingAccels.empty() ? Method::ValuesSpeedsAndAccels
                                       : Method::ValuesAndSpeeds;
        if (!missingAccels.empty() &&
                missingAccels.size() < withValues.size()) {
            log_warn("createZMPLoadsFromStates: no accelerations for "
                     "coordinate(s) {}; ignoring all acceleration columns.",
                    join(missingAccels));
        }
    } else if (missingSpeeds.size() < withValues.size()) {
        log_warn("createZMPLoadsFromStates: no speeds for coordinate(s) {}; "
                 "ignoring all speed columns.",
                join(missingSpeeds));
    }

    // Columns used by each method; report the rest.
    std::vector<std::string> used;
    const auto yIndices = createSystemYIndexMap(model);
    for (const auto& label : table.getColumnLabels()) {
        const bool isState = yIndices.count(label) > 0;
        const bool isValue = isState && endsWith(label, "/value");
        const bool isAccel = endsWith(label, "/accel") &&
                             yIndices.count(label.substr(0, label.size() - 6) +
                                            "/speed") > 0;
        if ((method == Method::ValuesOnly && isValue) ||
                (method == Method::ValuesAndSpeeds && isState) ||
                (method == Method::ValuesSpeedsAndAccels &&
                        (isState || isAccel))) {
            used.push_back(label);
        }
    }
    std::vector<std::string> ignored;
    for (const auto& label : table.getColumnLabels()) {
        if (std::find(used.begin(), used.end(), label) == used.end()) {
            ignored.push_back(label);
        }
    }
    if (!ignored.empty()) {
        log_info("createZMPLoadsFromStates: ignoring column(s) {}.",
                join(ignored));
    }

    // Set up how each frame's state and accelerations are obtained.
    const auto& times = table.getIndependentColumn();
    const int numFrames = static_cast<int>(times.size());
    SimTK::State* state = nullptr;
    // Sets the state for a frame and realizes it to Stage::Velocity.
    std::function<void(int)> setFrame;
    // The frame's generalized accelerations; call after setFrame().
    std::function<SimTK::Vector(int)> calcUDot;

    std::vector<std::pair<int, int>> stateColumns;       // (column, Y index)
    std::vector<std::pair<int, int>> accelColumns;       // (column, U index)
    std::vector<std::pair<GCVSpline, int>> speedSplines; // (spline, U index)

    if (method == Method::ValuesOnly) {
        log_info("createZMPLoadsFromStates: using coordinate values; speeds "
                 "and accelerations are derivatives of splines of the "
                 "values (PositionMotion).");

        // Prescribe every coordinate: with a spline of its values if
        // present, otherwise at its default value.
        auto motion = std::make_unique<PositionMotion>("zmp_position_motion");
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            const std::string label = coord.getAbsolutePathString() + "/value";
            if (table.hasColumn(label)) {
                motion->setPositionForCoordinate(
                        coord, createSpline(table, label));
            } else {
                motion->setPositionForCoordinate(
                        coord, Constant(coord.getDefaultValue()));
            }
        }
        model.addComponent(motion.release());
        state = &model.initSystem();

        setFrame = [&](int row) {
            state->setTime(times[row]);
            model.getSystem().prescribe(*state);
            model.realizeVelocity(*state);
        };
        calcUDot = [&](int) {
            model.realizeAcceleration(*state);
            return SimTK::Vector(state->getUDot());
        };
    } else {
        // States as given. Y = [q, u, z], so a speed's mobility index is its
        // Y index minus nq.
        state = &model.initSystem();
        const int nq = state->getNQ();
        const auto& labels = table.getColumnLabels();
        for (int col = 0; col < static_cast<int>(labels.size()); ++col) {
            const std::string& label = labels[col];
            if (const auto it = yIndices.find(label); it != yIndices.end()) {
                stateColumns.emplace_back(col, it->second);
            } else if (method == Method::ValuesSpeedsAndAccels &&
                       endsWith(label, "/accel")) {
                const auto speed = yIndices.find(
                        label.substr(0, label.size() - 6) + "/speed");
                if (speed != yIndices.end()) {
                    accelColumns.emplace_back(col, speed->second - nq);
                }
            }
        }

        if (method == Method::ValuesAndSpeeds) {
            log_info("createZMPLoadsFromStates: using coordinate values and "
                     "speeds; accelerations are derivatives of splines of "
                     "the speeds.");
            for (const Coordinate* coord : withValues) {
                const std::string label =
                        coord->getAbsolutePathString() + "/speed";
                speedSplines.emplace_back(
                        createSpline(table, label), yIndices.at(label) - nq);
            }
        } else {
            log_info("createZMPLoadsFromStates: using coordinate values, "
                     "speeds and accelerations from the table.");
        }

        setFrame = [&](int row) {
            const auto values = table.getRowAtIndex(row);
            state->setTime(times[row]);
            for (const auto& [col, yIndex] : stateColumns) {
                state->updY()[yIndex] = values[col];
            }
            model.realizeVelocity(*state);
        };
        calcUDot = [&](int row) {
            SimTK::Vector udot(state->getNU(), 0.0);
            if (method == Method::ValuesSpeedsAndAccels) {
                const auto values = table.getRowAtIndex(row);
                for (const auto& [col, uIndex] : accelColumns) {
                    udot[uIndex] = values[col];
                }
            } else {
                const std::vector<int> firstDerivative{0};
                const SimTK::Vector time(1, times[row]);
                for (const auto& [spline, uIndex] : speedSplines) {
                    udot[uIndex] = spline.calcDerivative(firstDerivative, time);
                }
            }
            return udot;
        };
    }
    const auto& estimator = findEstimator(model, estimatorPath);
    const int numSides = estimator.getNumContactSides();

    // Pass 1: detect contact in every frame, in order, applying each
    // ContactPoint's hysteresis (which depends on the previous frame).
    std::vector<std::vector<bool>> sideContact(
            numSides, std::vector<bool>(numFrames, false));
    std::vector<std::vector<bool>> pointContact(numSides);
    std::vector<std::string> hysteresisPoints;
    for (int i = 0; i < numSides; ++i) {
        const ContactSide& side = estimator.getContactSide(i);
        pointContact[i].assign(side.getNumContactPoints(), false);
        for (int j = 0; j < side.getNumContactPoints(); ++j) {
            if (side.getContactPoint(j).usesHysteresis()) {
                hysteresisPoints.push_back(side.getContactPoint(j).getName());
            }
        }
    }
    if (!hysteresisPoints.empty()) {
        log_info("createZMPLoadsFromStates: using contact hysteresis for "
                 "point(s) {}.",
                join(hysteresisPoints));
    }
    for (int row = 0; row < numFrames; ++row) {
        setFrame(row);
        for (int i = 0; i < numSides; ++i) {
            const ContactSide& side = estimator.getContactSide(i);
            bool inContact = false;
            for (int j = 0; j < side.getNumContactPoints(); ++j) {
                const bool pointInContact = side.getContactPoint(j).isInContact(
                        *state, pointContact[i][j]);
                pointContact[i][j] = pointInContact;
                inContact = inContact || pointInContact;
            }
            sideContact[i][row] = inContact;
        }
    }

    // Pass 2: remove contact and no-contact periods shorter than each side's
    // minimum_contact_duration.
    for (int i = 0; i < numSides; ++i) {
        const ContactSide& side = estimator.getContactSide(i);
        const double minimumDuration = side.getMinimumContactDuration();
        if (minimumDuration == -1) continue;
        const auto [contacts, gaps] =
                removeShortPeriods(sideContact[i], times, minimumDuration);
        log_info("createZMPLoadsFromStates: side '{}': removed {} contact "
                 "period(s) and {} no-contact period(s) shorter than {} s.",
                side.getName(), contacts, gaps, minimumDuration);
    }

    // Pass 3: estimate the ground reactions with the final contact status.
    TimeSeriesTable loads = createOutputTable(estimator);
    InvalidPeriodReporter reporter;
    std::vector<bool> sidesInContact(numSides);
    for (int row = 0; row < numFrames; ++row) {
        setFrame(row);
        const SimTK::Vector udot = calcUDot(row);
        for (int i = 0; i < numSides; ++i) {
            sidesInContact[i] = sideContact[i][row];
        }
        const ZMPResult result =
                estimator.calcZMPResult(*state, udot, sidesInContact);
        reporter.add(times[row], result);
        appendResult(loads, times[row], result);
    }
    reporter.flush();
    return loads;
}

//=============================================================================
// EXTERNAL LOADS
//=============================================================================
ExternalLoads createZMPExternalLoads(Model model,
        const std::string& dataFileName, const std::string& estimatorPath) {
    // Connect the model so the contact bodies can be resolved.
    model.initSystem();
    const auto& estimator = findEstimator(model, estimatorPath);

    ExternalLoads loads;
    loads.setName("zmp_external_loads");
    loads.setDataFileName(dataFileName);
    for (int i = 0; i < estimator.getNumContactSides(); ++i) {
        const ContactSide& side = estimator.getContactSide(i);
        const std::string& name = side.getName();

        // ExternalForce applies forces to a body.
        const Frame& base = side.getContactBody().findBaseFrame();
        OPENSIM_THROW_IF(dynamic_cast<const Body*>(&base) == nullptr, Exception,
                std::format("Expected the contact body of ContactSide '{}' "
                            "to be attached to a Body, but its base frame is "
                            "'{}'.",
                        name, base.getAbsolutePathString()));
        const std::string body = model.getBodySet().contains(base.getName())
                                         ? base.getName()
                                         : base.getAbsolutePathString();

        auto force = std::make_unique<ExternalForce>();
        force->setName(name);
        force->set_applied_to_body(body);
        force->set_force_expressed_in_body("ground");
        force->set_point_expressed_in_body("ground");
        force->setForceIdentifier(forceIdentifier(name));
        force->setPointIdentifier(pointIdentifier(name));
        force->setTorqueIdentifier(torqueIdentifier(name));
        loads.adoptAndAppend(force.release());
    }
    return loads;
}

} // namespace OpenSim
