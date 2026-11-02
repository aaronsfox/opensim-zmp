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
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/TableProcessor.h>

namespace OpenSim {

/** @name Zero Moment Point utilities
Estimate ground reactions over a motion with a ZeroMomentPointEstimator, and
create the ExternalLoads that apply them.

### Typical use
@code{.cpp}
// Ground reactions from Inverse Kinematics results, filtered at 6 Hz.
TimeSeriesTable grf = createZMPLoadsFromStates(
        model, TableProcessor("ik.mot"), "", 6.0);
STOFileAdapter::write(grf, "grf_zmp.mot");

// ExternalLoads that apply them to the contact bodies.
ExternalLoads loads = createZMPExternalLoads(model, "grf_zmp.mot");
loads.print("grf_zmp_loads.xml");
@endcode
@ingroup simulationutil */
// @{

/** Estimate ground reactions over a motion from a table of model states.

The returned table has 9 columns per ContactSide of the estimator, named after
the side, using the column naming of OpenSim ground reaction (.mot) files:
- `<side>_force_vx`, `<side>_force_vy`, `<side>_force_vz`: GRF (N), in Ground.
- `<side>_force_px`, `<side>_force_py`, `<side>_force_pz`: center of pressure
  (m), in Ground.
- `<side>_torque_x`, `<side>_torque_y`, `<side>_torque_z`: free moment (N-m),
  in Ground.
createZMPExternalLoads() creates ExternalLoads that read these columns.

A side's columns are zero at times when it is not valid: the side is not in
contact, both sides are in contact (splitting the load in double support is
not yet implemented), or the vertical force is below the estimator's
contact_force_threshold. Each such period is reported with a warning giving
its time range.

### Input table
`states` can be a file name, a TimeSeriesTable, or a TableProcessor. Before
use, the table is processed as follows:
1. Column labels are converted to absolute state names with
   updateStateLabels40() (e.g., `pelvis_tilt` ->
   `/jointset/ground_pelvis/pelvis_tilt/value`, `pelvis_tilt_u` -> `.../speed`),
   so Inverse Kinematics (.mot) results can be used directly.
2. If the table's `inDegrees` metadata is "yes", rotational coordinates
   (values, speeds and accelerations) are converted to radians. A table with no
   `inDegrees` metadata is assumed to be in radians (with a warning).
3. If `lowpassCutoffFrequency` is given and positive, all columns are low-pass
   filtered with TabOpLowPassFilter (padded before filtering, then trimmed to
   the original time range). Otherwise no filtering is applied.

Columns are matched to the model by name, so their order does not matter.
Columns that do not match a state variable or a coordinate acceleration are
ignored, with a message listing them.

### How the speeds and accelerations are obtained
The method depends on which columns the table has, for the coordinates that
have values. The method chosen is reported with a message.
1. **Values only** (e.g., Inverse Kinematics results; also used when some of
   these coordinates have no speeds): each coordinate's values are fit with a
   quintic GCVSpline (interpolating) and prescribed with a PositionMotion.
   Speeds and accelerations are the first and second derivatives of the same
   spline, so they are always consistent with the values. Speed and other
   state columns are not used.
2. **Values and speeds** (e.g., a simulation's states): values, speeds and any
   other state variables in the table are used as given. Accelerations are the
   time derivatives of quintic GCVSplines (interpolating) of the speeds.
3. **Values, speeds and accelerations**: as 2, with accelerations from columns
   named `<coordinate path>/accel` (the Moco convention). If only some
   coordinates have accelerations, they are ignored and method 2 is used.
In every case, accelerations are matched to the model's mobilities by name.

Coordinates with no values in the table take their default values (and zero
speed), with a warning listing them. Other missing state variables (e.g.,
muscle states) take their defaults; they do not affect the estimate, which uses
gravity as the only applied force.

### Contact detection over time
Contact is first detected for every frame, in order, then the ground
reactions are estimated with that contact status. This allows two optional
smoothing steps that depend on neighbouring frames (both off by default):
- **Hysteresis** per ContactPoint (`distance_off_threshold`,
  `velocity_off_threshold`): a point in contact stays in contact until it
  exceeds the off thresholds, so it cannot switch on and off from frame to
  frame near the on thresholds.
- **Minimum duration** per ContactSide (`minimum_contact_duration`): contact
  and no-contact periods shorter than it are removed (except at the start or
  end of the trial); the number removed is reported.
Hysteresis is applied first, then the minimum duration. The warnings about
periods without ground reactions describe the final contact status.

### Model
The model is copied, so the caller's model is not changed. It must contain a
ZeroMomentPointEstimator: the one at `estimatorPath`, or, if the path is empty,
the only ZeroMomentPointEstimator in the model. If you built the model in code,
call `model.finalizeConnections()` (or `model.initSystem()`) before passing it,
so the copy keeps its connections. Kinematic constraints (e.g., coordinate
couplers) are not enforced; the input is assumed to satisfy them.

@param model the model containing the ZeroMomentPointEstimator (copied).
@param states the states table (file name, table, or TableProcessor).
@param estimatorPath path to the estimator; empty to use the only one.
@param lowpassCutoffFrequency low-pass cutoff (Hz); values <= 0 (the default)
       mean no filtering.
@returns the ground reaction table described above. */
OSIMSIMULATION_API TimeSeriesTable createZMPLoadsFromStates(Model model,
        const TableProcessor& states, const std::string& estimatorPath = "",
        double lowpassCutoffFrequency = -1);

/** Create ExternalLoads that apply the ground reactions in a table created by
createZMPLoadsFromStates() and written to `dataFileName`.

There is one ExternalForce per ContactSide of the estimator, named after the
side, with:
- `applied_to_body`: the body that the side's `contact_body` is attached to
  (its base frame), by name if it is in the model's BodySet, otherwise by
  path;
- `force_identifier` `<side>_force_v`, `point_identifier` `<side>_force_p` and
  `torque_identifier` `<side>_torque_`, matching the table's column labels;
- force and point expressed in ground.

The ExternalLoads' data file is `dataFileName`; if it is a relative path, it is
located relative to the ExternalLoads file once that is printed and loaded, or
relative to the working directory.

@param model the model containing the ZeroMomentPointEstimator (copied).
@param dataFileName the file containing the ground reaction table.
@param estimatorPath path to the estimator; empty to use the only one. */
OSIMSIMULATION_API ExternalLoads createZMPExternalLoads(Model model,
        const std::string& dataFileName, const std::string& estimatorPath = "");

// @}

} // namespace OpenSim

#endif // OPENSIM_ZERO_MOMENT_POINT_UTILITIES_H_
