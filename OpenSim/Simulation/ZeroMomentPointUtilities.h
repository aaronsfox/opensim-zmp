#ifndef OPENSIM_ZEROMOMENTPOINT_UTILITIES_H
#define OPENSIM_ZEROMOMENTPOINT_UTILITIES_H
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ZeroMomentPointUtilities.h                   *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>

namespace OpenSim {

/**
 * Creates a flat TimeSeriesTable of ground reaction forces, centres of
 * pressure, and free moments from a ZeroMomentPointEstimator evaluated
 * over a states table (TimeSeriesTable of model state values and speeds).
 *
 * The states table is converted internally to a StatesTrajectory using
 * StatesTrajectory::createFromStatesTable so that each row can be
 * realised as a SimTK::State and passed to the estimator outputs.
 *
 * The output table has 9 columns per ContactSide:
 *   <side>_force_vx/vy/vz  — GRF components (N)
 *   <side>_CoP_x/y/z       — centre of pressure (m)
 *   <side>_torque_x/y/z    — free moment components (Nm)
 *
 * Rows where a side is not valid (not in contact or vertical force below
 * threshold) have all columns set to zero, consistent with the behaviour
 * of MocoUtilities::createExternalLoadsTableForGait.
 *
 * @param model       The OpenSim model containing the estimator. Must
 *                    have been initialised (initSystem called) before
 *                    calling this function.
 * 
 * @param statesTable A TimeSeriesTable of model state values, e.g. as
 *                    produced by a kinematics solve or MocoSolution.
 *                    Column labels must match the model's state names.
 *                    and include values and speeds.
 * 
 * @param estimator   The ZeroMomentPointEstimator to evaluate. Must
 *                    already be connected to the model.
 *
 * @returns A TimeSeriesTable with rows = number of states, columns = 9 *
 *          number of contact sides, and standard GRF .mot metadata.
 *
 * ### Example
 * @code
 * // After building model and adding ZeroMomentPointEstimator:
 * model.initSystem();
 *
 * // Load a states table from a kinematics .sto file
 * TimeSeriesTable statesTable("walk_states.sto");
 *
 * const auto& estimator =
 *     model.getComponent<ZeroMomentPointEstimator>("zmp_estimator");
 *
 * TimeSeriesTable grfTable =
 *     createZMPLoadsFromStates(model, statesTable, estimator);
 *
 * STOFileAdapter::write(grfTable, "walk_grf_zmp.mot");
 * @endcode
 */
OSIMSIMULATION_API
TimeSeriesTable createZMPLoadsFromStates(
        const Model&                      model,
        const TimeSeriesTable&            statesTable,
        const ZeroMomentPointEstimator&   estimator);

/**
 * Creates a flat TimeSeriesTable of ground reaction forces, centres of
 * pressure, and free moments from a ZeroMomentPointEstimator evaluated
 * over a values table (TimeSeriesTable of model state values).
 *
 * This code effectively works the same as createZMPLoadsFromStates, 
 * but takes the input of a values table instead, calculates the speeds
 * and then passes this table to the createZMPLoadsFromStates function.
 * 
 * Theoretically this method may be slightly less accurate, as errors
 * could propogate through twice differentiating the values to get
 * accelerations.
 */
OSIMSIMULATION_API
TimeSeriesTable createZMPLoadsFromValues(const Model& model,
        const TimeSeriesTable& valuesTable,
        const ZeroMomentPointEstimator& estimator);

} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINT_UTILITIES_H
