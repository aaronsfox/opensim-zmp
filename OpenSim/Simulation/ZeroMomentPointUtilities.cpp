/* -------------------------------------------------------------------------- *
 *          OpenSim: ZeroMomentPointUtilities.cpp                             *
 * -------------------------------------------------------------------------- */

#include "ZeroMomentPointUtilities.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Storage.h>

using namespace OpenSim;

namespace OpenSim {

/**
 * Creates a flat TimeSeriesTable of ground reaction forces, centres of
 * pressure, and free moments from a ZeroMomentPointEstimator evaluated
 * over a states table (TimeSeriesTable of model state values and speeds).
 */
TimeSeriesTable OpenSim::createZMPLoadsFromStates(
        const Model&                    model,
        const TimeSeriesTable&          statesTable,
        const ZeroMomentPointEstimator& estimator) {

    // ------------------------------------------------------------------
    // Build column labels — 9 columns per contact side.
    // Naming matches the OpenSim GRF .mot convention so the table can
    // be consumed directly by ExternalLoads / ExternalForce.
    // ------------------------------------------------------------------
    
    // Get the number of contact sides in ZMP estimator
    const int nSides = estimator.getNumContactSides();

    // Check there are actually contact sides
    OPENSIM_THROW_IF(nSides == 0, Exception,
        "createZMPLoadsFromStates: ZeroMomentPointEstimator '" +
        estimator.getName() + "' has no ContactSides.");

    // Create the labels for the external loads table
    std::vector<std::string> labels;
    labels.reserve(nSides * 9);

    for (int i = 0; i < nSides; ++i) {
        const std::string& side = estimator.getContactSide(i).getName();

        // GRF force components
        labels.push_back(side + "_force_vx");
        labels.push_back(side + "_force_vy");
        labels.push_back(side + "_force_vz");

        // Centre of pressure
        labels.push_back(side + "_force_px");
        labels.push_back(side + "_force_py");
        labels.push_back(side + "_force_pz");

        // Free moment (torque)
        labels.push_back(side + "_torque_x");
        labels.push_back(side + "_torque_y");
        labels.push_back(side + "_torque_z");
    }

    // ------------------------------------------------------------------
    // Convert the TimeSeriesTable to a StatesTrajectory so each row
    // can be realised as a SimTK::State by the model.
    // allowMissingColumns=true allows tables that only contain a
    // subset of state variables (e.g. kinematics-only tables without
    // muscle states). allowExtraColumns=true ignores any columns in
    // the table that don't correspond to model states.
    // ------------------------------------------------------------------
    
    // Check for rows in states table
    OPENSIM_THROW_IF(statesTable.getNumRows() == 0, Exception,
        "createZMPLoadsFromStates: statesTable is empty.");

    // Create the states trajectory from the input table
    const StatesTrajectory states =
        StatesTrajectory::createFromStatesTable(
            model,
            statesTable,
            true, // allowMissingColumns
            true  // allowExtraColumns
        );

    // ------------------------------------------------------------------
    // Generate accurate accelerations from states
    // ------------------------------------------------------------------

    // This function takes a states table that likely contains
    // motions and velocities which result in zero accelerations
    // being input to the ID solver. Here the accelerations to input
    // to inverse dynamics are generated based on the provided states.

    // -----------------------------------------------------------------------
    // Option 1: Accelerations derived from speeds (first derivative of speeds)
    // -----------------------------------------------------------------------
    
    // Get only the speed columns from the states table and create udot labels
    std::vector<std::string> speedLabels;
    std::vector<std::string> accLabels;
    for (const auto& label : statesTable.getColumnLabels()) {
        if (label.find("/speed") != std::string::npos) {
            speedLabels.push_back(label);
            // Replace /speed with /acc for the output table label
            std::string accLabel = label;
            accLabel.replace(accLabel.find("/speed"),
                    std::string("/speed").length(), "/acc");
            accLabels.push_back(accLabel);
        }
    }

    // Build splines from speed columns
    GCVSplineSet speedSplines(statesTable, speedLabels, 5, 0.0);

    // Get time vector
    const auto& speedTimes = statesTable.getIndependentColumn();
    
    // Build acceleration table from speed splines (first derivative)
    TimeSeriesTable udot;
    udot.setColumnLabels(accLabels);
    std::vector<int> firstDeriv = {0}; // first derivative
    for (size_t i = 0; i < speedTimes.size(); ++i) {
        SimTK::RowVector row(static_cast<int>(speedLabels.size()));
        SimTK::Vector timeVec(1, speedTimes[i]);
        for (int j = 0; j < speedSplines.getSize(); ++j) {
            row[j] = speedSplines[j].calcDerivative(firstDeriv, timeVec);
        }
        udot.appendRow(speedTimes[i], row);
    }

    //// -----------------------------------------------------------------------
    //// Option 2: Accelerations derived from values (second derivative of
    ///values) /
    ///-----------------------------------------------------------------------
    //    // Get only the value columns
    // std::vector<std::string> valueLabels;
    // for (const auto& label : statesTable.getColumnLabels()) {
    //    if (label.find("/value") != std::string::npos)
    //        valueLabels.push_back(label);
    //}
    //// Build splines from value columns
    // GCVSplineSet valueSplines(statesTable, valueLabels, 5, 0.0);
    //// Build acceleration table from value splines (second derivative)
    // TimeSeriesTable accFromValues;
    // accFromValues.setColumnLabels(valueLabels);
    // std::vector<int> secondDeriv = {0, 0}; // second derivative
    // for (size_t i = 0; i < times.size(); ++i) {
    //     SimTK::RowVector row(static_cast<int>(valueLabels.size()));
    //     SimTK::Vector timeVec(1, times[i]);
    //     for (int j = 0; j < valueSplines.getSize(); ++j) {
    //         row[j] = valueSplines[j].calcDerivative(secondDeriv, timeVec);
    //     }
    //     accFromValues.appendRow(times[i], row);
    // }
    // std::cout << "Accelerations from values rows: "
    //           << accFromValues.getNumRows() << "\n";
    // std::cout << "Accelerations from values cols: "
    //           << accFromValues.getNumColumns() << "\n";

    // ------------------------------------------------------------------
    // Build the output table and loop over states.
    // ------------------------------------------------------------------

    // Set number of rows and columns variables
    const int nRows = static_cast<int>(states.getSize());
    const int nColumns = static_cast<int>(labels.size()); // nSides * 9

    // Initialise the time series table for external loads
    TimeSeriesTable table;
    table.setColumnLabels(labels);

    // Loop over states, evaluate estimator outputs, and fill table rows
    for (int row = 0; row < nRows; ++row) {

        // Each entry in the StatesTrajectory is a fully realised
        // SimTK::State corresponding to one row of the input statesTable.
        const SimTK::State& workingState = states[row];

        // Realise to Acceleration stage for computations
        // As PositionMotion is active, this will automaticall
        // populate positions, speeds and accelerations in the state
        model.realizeAcceleration(workingState);

        // Get row udot from accelerations table
        // Convert to SimTK::Vector
        // Pass into estimator
        const SimTK::RowVectorView udotRow = udot.getRowAtIndex(row);
        SimTK::Vector udotVec(udotRow.size());
        for (int j = 0; j < udotRow.size(); ++j) udotVec[j] = udotRow[j];
        const_cast<ZeroMomentPointEstimator&>(estimator).setUDot(udotVec);

        //// Check udot directly — if all zeros, ID solve has nothing
        //std::cout << " States udot: " << workingState.getUDot() << "\n";
        //std::cout << " Created udot: " << udotRow << "\n";

        // Build a row of scalar values to fill for current state
        SimTK::RowVector rowData(nColumns, 0.0);
        
        // Set starting column to zero for first side
        int col = 0;
        
        // Loop through contact sides and extract values
        for (int i = 0; i < nSides; ++i) {

            // Get the contact side
            const ContactSide& cs = estimator.getContactSide(i);
            const std::string& side = cs.getName();

            // Get the data from each channel for the current side
            // Ground reaction force (N)
            const SimTK::Vec3 grf = 
                estimator.getGRFForChannel(workingState, side);
            // Centre of pressure (m)
            const SimTK::Vec3 cop =
                    estimator.getCoPForChannel(workingState, side);
            // Free moment / torque (Nm)
            const SimTK::Vec3 trq =
                    estimator.getFreeMomentForChannel(workingState, side);

            // Extract the row data to the appropriate columns
            rowData[col + 0] = grf[0];
            rowData[col + 1] = grf[1];
            rowData[col + 2] = grf[2];
            rowData[col + 3] = cop[0];
            rowData[col + 4] = cop[1];
            rowData[col + 5] = cop[2];
            rowData[col + 6] = trq[0];
            rowData[col + 7] = trq[1];
            rowData[col + 8] = trq[2];

            // Add to column indicator for next body
            col += 9;
        }

        // Append the row at this state's time
        table.appendRow(workingState.getTime(), rowData);
    }

    // ------------------------------------------------------------------
    // Add standard metadata expected by ExternalLoads.
    // ------------------------------------------------------------------

    // Set table metadata
    table.updTableMetaData().setValueForKey("name",
        std::string("ZMP Estimated Ground Reactions"));
    table.updTableMetaData().setValueForKey("inDegrees",
        std::string("no"));
    table.updTableMetaData().setValueForKey("nRows",
        std::to_string(nRows));
    table.updTableMetaData().setValueForKey("nColumns",
        std::to_string(nColumns + 1));  // +1 for time column

    // Return the table
    return table;
}

/**
 * Creates a flat TimeSeriesTable of ground reaction forces, centres of
 * pressure, and free moments from a ZeroMomentPointEstimator evaluated
 * over a values table (TimeSeriesTable of model state values).
 */
TimeSeriesTable OpenSim::createZMPLoadsFromValues(const Model& model,
        const TimeSeriesTable& valuesTable,
        const ZeroMomentPointEstimator& estimator) {

    // Append the speeds into the values table
    
    // Get only the value columns from the table and create speed labels
    std::vector<std::string> valueLabels;
    std::vector<std::string> speedLabels;
    for (const auto& label : valuesTable.getColumnLabels()) {
        if (label.find("/value") != std::string::npos) {
            valueLabels.push_back(label);
            // Replace /speed with /acc for the output table label
            std::string accLabel = label;
            accLabel.replace(accLabel.find("/value"),
                    std::string("/value").length(), "/speed");
            speedLabels.push_back(accLabel);
        }
    }

    // Build splines from speed columns
    GCVSplineSet speedSplines(valuesTable, valueLabels, 5, 0.0);

    // Get time vector
    const auto& valueTimes = valuesTable.getIndependentColumn();

    // Build speeds table from value splines (first derivative)
    TimeSeriesTable speedsTable;
    speedsTable.setColumnLabels(speedLabels);
    std::vector<int> firstDeriv = {0}; // first derivative
    for (size_t i = 0; i < valueTimes.size(); ++i) {
        SimTK::RowVector row(static_cast<int>(valueLabels.size()));
        SimTK::Vector timeVec(1, valueTimes[i]);
        for (int j = 0; j < speedSplines.getSize(); ++j) {
            row[j] = speedSplines[j].calcDerivative(firstDeriv, timeVec);
        }
        speedsTable.appendRow(valueTimes[i], row);
    }

    // Combine the value and speeds to make a states table

    // Extract dimensions and timestamps
    const auto& timePoints = valuesTable.getIndependentColumn();
    const int nRows = static_cast<int>(timePoints.size());
    const int nValuesCols = static_cast<int>(valuesTable.getNumColumns());
    const int nSpeedsCols = static_cast<int>(speedsTable.getNumColumns());

    // Sanity check to ensure the tables have matching row counts and columns
    OPENSIM_THROW_IF(nValuesCols != nSpeedsCols, OpenSim::Exception,
            "Values and speeds tables must have the same number of "
            "columns.");
    OPENSIM_THROW_IF(nRows != static_cast<int>(speedsTable.getNumRows()),
            OpenSim::Exception,
            "Values and speeds tables must have the same number of rows.");

    // Set up the states column labels
    std::vector<std::string> statesLabels;
    statesLabels.reserve(nValuesCols + nSpeedsCols);
    for (int col = 0; col < nValuesCols; ++col) {
        statesLabels.push_back(valueLabels[col]);
        statesLabels.push_back(speedLabels[col]);
    }

    // Populate the new states data table row-by-row
    OpenSim::TimeSeriesTable statesTable;
    statesTable.setColumnLabels(statesLabels);
    for (int row = 0; row < nRows; ++row) {
        // Get time and values
        double currentTime = timePoints[row];
        const auto& valueRow = valuesTable.getRowAtIndex(row);
        const auto& speedRow = speedsTable.getRowAtIndex(row);
        // Create a row vector big enough to hold both values and speeds
        SimTK::RowVector combinedRow(nValuesCols + nSpeedsCols);
        // Insert individual values and states pair by pair into row
        int targetIdx = 0;
        for (int col = 0; col < nValuesCols; ++col) {
            combinedRow[targetIdx++] = valueRow[col];
            combinedRow[targetIdx++] = speedRow[col];
        }
        // Append the row to table
        statesTable.appendRow(currentTime, combinedRow);
    }

    // Pass states table to ZMP loads function
    TimeSeriesTable table =
            createZMPLoadsFromStates(model, statesTable, estimator);

    // Return the table
    return table;
    
}

} // namespace OpenSim
