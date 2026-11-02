#ifndef OPENSIM_ZERO_MOMENT_POINT_UTILITIES_H_
#define OPENSIM_ZERO_MOMENT_POINT_UTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ZeroMomentPointUtilities.h                     *
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

#include <string>

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/TableProcessor.h>

namespace OpenSim {

/** @name Zero Moment Point utilities
Estimate ground reactions over a motion with a ZeroMomentPointEstimator.

Both functions return a table with 9 columns per ContactSide (named after the
side), using the column naming of OpenSim ground reaction (.mot) files so the
table can be used by ExternalLoads / ExternalForce:
- `<side>_force_vx`, `<side>_force_vy`, `<side>_force_vz`: GRF (N), in Ground.
- `<side>_force_px`, `<side>_force_py`, `<side>_force_pz`: center of pressure
  (m), in Ground.
- `<side>_torque_x`, `<side>_torque_y`, `<side>_torque_z`: free moment (N-m),
  in Ground.

Rows where a side is not valid (not in contact, the other side also in
contact, or vertical force below the estimator's contact_force_threshold) are
zero for that side. The ExternalForce identifiers for a side are
`<side>_force_v`, `<side>_force_p` and `<side>_torque_`, expressed in ground.

### Input tables
The input is a TableProcessor, so it can be a file name, a TimeSeriesTable,
or a table with TableOperator%s applied. Before use, the table is processed
as follows:
1. Column labels are converted to absolute state names with
   updateStateLabels40() (e.g., `pelvis_tilt` -> `/jointset/ground_pelvis/
   pelvis_tilt/value`, `pelvis_tilt_u` -> `.../speed`), so Inverse Kinematics
   (.mot) results can be used directly.
2. If the table's `inDegrees` metadata is "yes", rotational coordinates are
   converted to radians.
3. If `lowpassCutoffFrequency` is positive, all columns are low-pass filtered
   with TabOpLowPassFilter (the table is padded before filtering and trimmed
   back to its original time range). Otherwise no filtering is applied.
   Accelerations amplify noise, so filtering measured kinematics is usually
   needed; choose the cutoff for your data.

Columns are matched to the model by name, so their order does not matter and
columns that do not match a state variable (or, for states, an acceleration)
are ignored, with a message listing them. Coordinates that are missing from
the table take their default values, with a warning listing them.

### Model
The model is copied, so the caller's model is not changed. It must contain a
ZeroMomentPointEstimator: the one at `estimatorPath`, or, if the path is
empty, the only ZeroMomentPointEstimator in the model. If you built the model
in code, call `model.finalizeConnections()` (or `model.initSystem()`) before
passing it, so the copy keeps its connections.

Kinematic constraints (e.g., coordinate couplers) are not enforced; the input
is assumed to satisfy them.

### Example
@code{.cpp}
// Inverse Kinematics output (coordinate names, in degrees), filtered at 6 Hz.
TimeSeriesTable grf = createZMPLoadsFromKinematics(
        model, TableProcessor("ik.mot"), 6.0);
STOFileAdapter::write(grf, "grf_zmp.mot");
@endcode
@ingroup simulationutil */
// @{

/** Estimate ground reactions from coordinate values (e.g., Inverse
Kinematics results).

Each coordinate's values are fit with a quintic GCVSpline (interpolating) and
prescribed with a PositionMotion, so the coordinate speeds and accelerations
are the first and second derivatives of the same spline and are always
consistent with the values (and in the system's own order). Speed and other
state columns in the table are ignored. The ContactPoint velocity criterion
uses these spline speeds.

@param model the model containing the ZeroMomentPointEstimator (copied).
@param kinematics coordinate values (file name, table, or TableProcessor).
@param lowpassCutoffFrequency low-pass cutoff (Hz) applied to the kinematics
       before splining; values <= 0 (the default) mean no filtering.
@param estimatorPath path to the estimator; empty to use the only one.
@returns the ground reaction table described above. */
OSIMSIMULATION_API TimeSeriesTable createZMPLoadsFromKinematics(Model model,
        const TableProcessor& kinematics, double lowpassCutoffFrequency = -1,
        const std::string& estimatorPath = "");

/** Estimate ground reactions from a states table that includes coordinate
values and speeds, and optionally accelerations.

The states are used as given; no PositionMotion is used:
- Coordinate values and speeds (and any other state variables in the table)
  are set in the state at each time.
- Generalized accelerations come from columns named
  `<coordinate path>/accel` (the Moco convention) when the table has one for
  every coordinate. When the table has none, they are the time derivatives of
  quintic GCVSplines (interpolating) of the speeds, so they are consistent
  with the speeds. A table with accelerations for only some coordinates is
  rejected.
- Accelerations are mapped to the system's mobilities by name, so column order
  does not matter.

Missing coordinate values and speeds take their default values (speeds
default to 0), with a warning. Other missing state variables (e.g., muscle
states) take their defaults; they do not affect the estimate, which uses
gravity as the only applied force.

@param model the model containing the ZeroMomentPointEstimator (copied).
@param states states (file name, table, or TableProcessor).
@param lowpassCutoffFrequency low-pass cutoff (Hz) applied to all columns;
       values <= 0 (the default) mean no filtering.
@param estimatorPath path to the estimator; empty to use the only one.
@returns the ground reaction table described above. */
OSIMSIMULATION_API TimeSeriesTable createZMPLoadsFromStates(Model model,
        const TableProcessor& states, double lowpassCutoffFrequency = -1,
        const std::string& estimatorPath = "");

// @}

} // namespace OpenSim

#endif // OPENSIM_ZERO_MOMENT_POINT_UTILITIES_H_
