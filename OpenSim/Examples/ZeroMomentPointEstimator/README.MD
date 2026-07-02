# Example: Zero Moment Point Estimator

This example demonstrates the `ZeroMomentPointEstimator` component for
estimating ground reaction forces (GRFs) and centres of pressure (CoPs)
from model kinematics using the Zero Moment Point (ZMP) method.

## Background

The Zero Moment Point method estimates the net ground reaction forces
acting on a body by computing the residual forces at the model's free
joint (typically the pelvis) via inverse dynamics, and then projecting
the net moment to the ground plane to find the centre of pressure. This
approach avoids the need for contact sphere models or explicit ground
contact formulations, making it well-suited to kinematics-driven analyses.

The method implemented here takes inspiration and leverages methods from as series of works:

Xiang, Y., Arora, J. S., Rahmatalla, S., & Abdel‐Malek, K. (2009). [Optimization‐based dynamic human walking prediction: One step formulation](https://doi.org/10.1002/nme.2575). *International Journal for Numerical Methods in Engineering*, *79*(6), 667-695.

Dijkstra, E. J., & Gutierrez-Farewik, E. M. (2015). [Computation of ground reaction force using zero moment point]([Redirecting](https://doi.org/10.1016/j.jbiomech.2015.08.027)). *Journal of Biomechanics*, *48*(14), 3776-3781.

Karčnik, T. (2003). [Using motion analysis data for foot-floor contact detection]([Client Challenge](https://doi.org/10.1007/BF02345310)). *Medical and Biological Engineering and Computing*, *41*(5), 509-512.

## Data

This example uses some sample data from:

Hamner, S. R., & Delp, S. L. (2013). [Muscle contributions to fore-aft and vertical body mass center accelerations over a range of running speeds]([Redirecting](https://doi.org/10.1016/j.jbiomech.2012.11.024)). *Journal of Biomechanics*, *46*(4), 780-787.

Specifically, the example includes the following files in the `data/` subdirectory:

| File                                  | Description                                                                                                                                                     |
| ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `subject01_run3.osim`                 | OpenSim musculoskeletal model                                                                                                                                   |
| `Run_3_grf.mot`                       | Measured ground reaction forces (for reference and comparison)                                                                                                  |
| `subject01_run3_trackingSolution.sto` | Moco tracking solution to export kinematics as a states table. The tracking solution specifically comes from [Fox (2024)](https://doi.org/10.1098/rsos.231909). |

## Component Overview

### `ZeroMomentPointEstimator`

The top-level component. Owns a list of up to two `ContactSide`
subcomponents and runs the ZMP computation pipeline at each state
evaluation. Results are cached so the inverse dynamics solve runs only
once per unique state.

**Properties**

- `free_joint_name`: name of the joint connecting the model to Ground
  (e.g. `"ground_pelvis"`)
- `contact_force_threshold`: minimum vertical GRF (N) required to mark
  a side's output as valid (default 20 N)

### `ContactSide`

Represents one limb side (e.g. right or left foot). Owns a
`ContactPointSet` of detection points and identifies the body to which
the estimated GRF is applied.

**Properties:**

- `contact_body`: the body to which ZMP derived GRF are applied to
- `ContactPointSet`: set of `ContactPoint` objects

### `ContactPoint`

A `Station` subclass that detects ground contact by comparing the
point's vertical height against a threshold. In this example, points are
placed at the locations of foot markers on the model.

**Properties:**

- `location`: position in the parent body's local frame (inherited from `Station` base class)
- `distance_threshold`: vertical height threshold for contact (m)
- `velocity_threshold`: velocity (m/s) threshold for determining contact when using combined distance and velocity method
- `contact_checking_method` — `"distance"` (height only) or
  `"velocity"` (height and velocity)

## Example Outline

### Step 1: Load the model

The Hamner running model is loaded from `data/subject01_run3.osim`.
Basic model statistics (number of bodies, joints, markers) are printed
to confirm successful loading.

### Step 2: Create the ZeroMomentPointEstimator

A `ZeroMomentPointEstimator` is created specifying the free joint name
(`"ground_pelvis"`) and a contact force threshold of 20 N.

### Step 3: Build contact sides

Two `ContactSide` objects are created, one for the right foot and one
for the left. Each side has six `ContactPoint` objects placed at the locations of foot markers:

| Marker                  | Body                  | Notes               |
| ----------------------- | --------------------- | ------------------- |
| `RCAL` / `LCAL`         | `calcn_r` / `calcn_l` | Heel                |
| `RTOE` / `LTOE`         | `calcn_r` / `calcn_l` | Toe tip             |
| `RMT5` / `LMT5`         | `calcn_r` / `calcn_l` | 5th metatarsal head |
| `RTOE_mid` / `LTOE_mid` | `calcn_r` / `calcn_l` | Mid toe             |
| `RMT5_mid` / `LMT5_mid` | `calcn_r` / `calcn_l` | Mid metatarsal      |
| `R_end` / `L_end`       | `calcn_r` / `calcn_l` | Distal foot end     |

### Step 4: Connect to model and print

The estimator is added to the model via `model.addComponent()` and the
updated model is printed to `subject01_run3_with_zmp.osim`. This file
could be reloaded in future without repeating the setup.

### Step 5: Query component properties

The example demonstrates how to interrogate the component after
initialisation:

- `getNumContactSides()`: number of contact sides
- `getContactSide(i)`: access side by index
- `getContactSide(name)`: access side by side name
- `getNumContactPoints()`: number of points per side
- `getContactPoint(i)`: access individual contact points
- `getOutput("GRF").getChannelNames()`: list output channels

### Step 6: Generate ZMP GRF table

The Moco tracking solution is loaded and exported to states as a `TimeSeriesTable`. It is then passed to the utility function `createZMPLoadsFromStates()`. This function:

- Converts the states table to a `StatesTrajectory`

- Loops over each time point

- Evaluates the estimator outputs at each state

- Assembles a flat `TimeSeriesTable` with 9 columns per side:
  
  - `<side>_force_vx/vy/vz`: GRF components (N)
  
  - `<side>_CoP_x/y/z`: centre of pressure (m)
  
  - `<side>_torque_x/y/z`: free moment components (Nm)

The table is written to `subject01_run3_zmp_grf.mot` in the standard
OpenSim GRF format, which can be loaded in the OpenSim GUI for visualisation, compared against the measured GRFs in `Run_3_grf.mot`, or used as input to further analyses.

## Output Files

| File                           | Description                                   |
| ------------------------------ | --------------------------------------------- |
| `subject01_run3_with_zmp.osim` | Model with ZMP estimator serialised           |
| `subject01_run3_zmp_grf.mot`   | ZMP-estimated GRFs in standard OpenSim format |

## Notes

- The ZMP method assumes a flat ground plane at y = 0. If using a different model
  with a different vertical axis or ground height it would need to be converted to the same coordinate system and configuration for the estimator to work.

- The running task included here only contains unilateral contact. Bilateral contact (i.e. double support) is not yet fully implemented with the Zero Moment Point method as the problem is indeterminant. During double support phases the estimator will simply print a warning and return zeroed outputs for both sides. 

- The ZMP-estimated GRFs are most accurate when the kinematics are
  smooth, well-filtered and containing minimal residuals. The ZMP method essentially considers any residuals remaining when GRFs are removed as the ground reactions, therefore any residuals remaining in kinematics will be included.
