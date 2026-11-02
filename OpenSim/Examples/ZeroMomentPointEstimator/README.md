# Example: Zero Moment Point Estimator

This example demonstrates how to estimate ground reaction forces (GRFs),
centers of pressure (CoPs) and free moments from model kinematics using the
Zero Moment Point (ZMP) method, with the `ZeroMomentPointEstimator`,
`ContactSide` and `ContactPoint` components and the ZMP utility functions
`createZMPLoadsFromStates()` and `createZMPExternalLoads()`.

## Background

Without force plates, the ground reactions can be estimated from the motion
itself. Inverse dynamics with gravity as the only applied force gives the
residual force and moment at the joint that connects the model to Ground
(typically the pelvis). This residual is the whole-body external load that the
motion requires, which during walking or running is supplied by the ground.
The ZMP is the point on the ground plane where the horizontal moment of this
load is zero; while one foot is on the ground, it is that foot's center of
pressure. The estimate does not need a contact model, only the kinematics and
the model's inertial properties.

The ground contact of each foot is detected from the motion with points on
the foot: a foot is in contact when any of its points is close enough to the
ground (and, optionally, moving slowly enough).

The method takes inspiration from and builds on:

Xiang, Y., Arora, J. S., Rahmatalla, S., & Abdel-Malek, K. (2009).
[Optimization-based dynamic human walking prediction: One step formulation](https://doi.org/10.1002/nme.2575).
*International Journal for Numerical Methods in Engineering*, *79*(6),
667-695.

Dijkstra, E. J., & Gutierrez-Farewik, E. M. (2015).
[Computation of ground reaction force using zero moment point](https://doi.org/10.1016/j.jbiomech.2015.08.027).
*Journal of Biomechanics*, *48*(14), 3776-3781.

Karčnik, T. (2003).
[Using motion analysis data for foot-floor contact detection](https://doi.org/10.1007/BF02345310).
*Medical and Biological Engineering and Computing*, *41*(5), 509-512.

## Data

This example uses data from:

Hamner, S. R., & Delp, S. L. (2013).
[Muscle contributions to fore-aft and vertical body mass center accelerations over a range of running speeds](https://doi.org/10.1016/j.jbiomech.2012.11.024).
*Journal of Biomechanics*, *46*(4), 780-787.

The following files are in this folder:

| File | Description |
| ---- | ----------- |
| `subject01_run3.osim` | OpenSim musculoskeletal model. |
| `subject01_run3_trackingSolution.sto` | Moco tracking solution of one running stride on a treadmill, with coordinate values and speeds (and controls). From [Fox (2024)](https://doi.org/10.1098/rsos.231909). |
| `Run_3_grf.mot` | Measured GRFs, for comparison. |
| `Geometry/` | Mesh files for visualizing the model. |

## Components

### `ZeroMomentPointEstimator`

Estimates the GRF, CoP and free moment for up to two contact sides. Results are
cached, so the inverse dynamics solve runs once per state.

**Properties**

- `contact_sides`: the `ContactSide`s (one or two).
- `free_joint_name`: the joint between Ground and the root body (default
  `ground_pelvis`).
- `contact_force_threshold`: minimum vertical GRF (N) for a side's outputs to
  be valid (default 20 N).
- `ground_height`: height of the horizontal ground plane along the Ground y
  axis (default 0 m).

**Outputs** (list outputs have one channel per side, named after the side)

- `ground_reaction_force`, `center_of_pressure`, `free_moment` (Vec3, in
  Ground).
- `is_in_contact`, `is_valid` (bool).
- `contact_mode` (int: 0 no contact, 1 one side, 2 both sides).

### `ContactSide`

One foot. Groups the `ContactPoint`s that detect its contact and names the body
its GRF is applied to.

- `contact_body` (socket): the body the GRF is applied to (here `calcn_r` /
  `calcn_l`).
- `contact_points`: the `ContactPoint`s.
- `minimum_contact_duration`: contact and no-contact periods shorter than this
  (s) are removed (default -1, unused).

### `ContactPoint`

A `Station` on the foot that detects contact with the ground.

- `location` and `parent_frame` (inherited from `Station`).
- `contact_checking_method`: `distance` (height only) or `velocity` (height and
  speed; Karcnik 2003).
- `distance_threshold`, `velocity_threshold`: the point comes into contact
  below these (defaults 0.008 m and 0.015 m/s, from Karcnik 2003).
- `distance_off_threshold`, `velocity_off_threshold`: optional hysteresis; a
  point in contact stays in contact until it exceeds these (default -1,
  unused).

## Example Outline

### Step 1: Load the model

The running model is loaded, and its connections are finalized so its markers'
frames can be used.

### Step 2: Create the ZeroMomentPointEstimator

The estimator is created for the `ground_pelvis` joint, keeping the default
contact force threshold (20 N) and ground height (y = 0, as in the measured
data).

### Step 3: Build the contact sides

A `ContactSide` is created for each foot (`right` on `calcn_r`, `left` on
`calcn_l`), with six `ContactPoint`s at the foot markers on the sole of the
shoe:

| Points | Markers | Location |
| ------ | ------- | -------- |
| `RCAL` / `LCAL` | `RCAL_ground` / `LCAL_ground` | Heel |
| `RTOE` / `LTOE` | `RTOE_ground` / `LTOE_ground` | Toe tip |
| `RMT5` / `LMT5` | `RMT5_ground` / `LMT5_ground` | 5th metatarsal head |
| `RTOE_mid` / `LTOE_mid` | `RTOE_mid_ground` / `LTOE_mid_ground` | Mid toe |
| `RMT5_mid` / `LMT5_mid` | `RMT5_mid_ground` / `LMT5_mid_ground` | Mid metatarsal |
| `R_end` / `L_end` | `R_end_ground` / `L_end_ground` | Distal end of the foot |

The contact settings used for this trial are:

- **Distance method.** The trial was recorded on a treadmill, so the stance
  foot moves backward with the belt and the velocity method would not detect
  contact.
- **Hysteresis.** A point comes into contact within 2 cm of the ground
  (`distance_threshold`) and leaves contact above 4 cm
  (`distance_off_threshold`). The default 8 mm threshold suits calibrated
  markers, but is too small for these marker-based points, which do not all
  reach the floor during stance.
- **Minimum contact duration of 20 ms.** None are removed in this smooth trial,
  but it guards against brief spurious contacts in noisier data.

The sides take ownership of the points (`adoptContactPoint()`), and the
estimator takes ownership of the sides (`adoptContactSide()`).

### Step 4: Add the estimator to the model

The estimator is added with `model.addComponent()`, and the model is printed
to `subject01_run3_with_zmp.osim`. The estimator, sides and points are saved
with the model, so this file can be loaded later without repeating Steps 2
and 3.

### Step 5: Query the estimator

The example shows how to access the estimator after `initSystem()`:

- `getNumContactSides()`, `getContactSide(index)`, `getContactSide(name)`.
- `getNumContactPoints()`, `getContactPoint(index)` and each point's settings.
- `getOutputNames()` and `getOutput(name)`.

### Step 6: Estimate the GRFs

The tracking solution is passed to `createZMPLoadsFromStates()`, which chooses
how to obtain the accelerations from the columns in the table:

| Columns (for the coordinates) | Method |
| ----------------------------- | ------ |
| Values only (e.g., Inverse Kinematics results) | Values are fit with splines and prescribed with a `PositionMotion`; speeds and accelerations are their derivatives. |
| Values and speeds (e.g., a tracking solution) | Values and speeds are used as given; accelerations are derivatives of splines of the speeds. |
| Values, speeds and accelerations (`<coordinate>/accel`) | All are used as given. |

The GRFs are estimated twice: from the solution's states (values and speeds),
and from its coordinate values only. Contact is detected for every frame in
order (applying the hysteresis), periods shorter than the minimum duration are
removed, and then the GRFs are estimated. Columns that are not used (here, the
controls) are listed, and periods with no GRF are reported with their time
ranges: here, the flight phases, and single frames at the start and end of
contact where the vertical force is below `contact_force_threshold`.

The solution is smooth, so no filtering is applied. For measured kinematics,
pass a low-pass cutoff frequency as the last argument, e.g.
`createZMPLoadsFromStates(model, states, "", 6.0)`.

Each table has 9 columns per side, in the OpenSim GRF file format:

- `<side>_force_vx/vy/vz`: GRF (N).
- `<side>_force_px/py/pz`: CoP (m).
- `<side>_torque_x/y/z`: free moment (N-m).

### Step 7: Create ExternalLoads

`createZMPExternalLoads()` creates an `ExternalForce` for each side, applied to
the side's contact body and reading the columns of the GRF file. The
ExternalLoads file can be used like one made for force plate data (e.g., with
the Inverse Dynamics tool, or `ModOpAddExternalLoads` in Moco).

### Step 8: Compare with the measured GRFs

For each side, the example reports how many frames the detected contact agrees
with the force plates (vertical force above 20 N), and the root-mean-square
differences between the estimated and measured vertical and fore-aft forces
during contact. Detected contact agrees with the force plates in 100-101 of
the 101 frames. Using the solution's speeds, the vertical forces differ by
about 1 N (root mean square) at peaks of about 1800 N; using the values only,
the extra differentiation increases the difference to about 3 N.

## Output Files

| File | Description |
| ---- | ----------- |
| `subject01_run3_with_zmp.osim` | Model with the estimator, sides and points. |
| `subject01_run3_zmp_grf.mot` | Estimated GRFs from the states (values and speeds). |
| `subject01_run3_zmp_grf_fromValues.mot` | Estimated GRFs from the coordinate values only. |
| `subject01_run3_zmp_grf.xml` | ExternalLoads that apply `subject01_run3_zmp_grf.mot` to the feet. |

## Notes

- The ground is a horizontal plane with the Ground y axis pointing up, at
  `ground_height`. Data in another coordinate system must be converted first.
- Splitting the GRF between the feet when both are in contact (double support)
  is statically indeterminate and is not yet implemented. During double
  support, both sides' GRFs are zero and the period is reported with a warning.
  This running trial has no double support.
- The estimate attributes all of the whole-body external load to the ground,
  so it is only as good as the kinematics and the model's inertial
  properties. Noise in the kinematics is amplified by differentiation (filter
  measured data), and any other external loads (e.g., handrails or assistive
  devices) are included in the estimated GRFs.
- Contact thresholds depend on the trial, the model and where the points are
  placed; check the reported contact periods against the expected gait events.
