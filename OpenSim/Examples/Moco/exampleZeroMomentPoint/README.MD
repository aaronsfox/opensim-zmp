# Example: Zero Moment Point Tracking

This example demonstrates how the `ZeroMomentPointEstimator` can be combined with other components (i.e. `BodyWrenchActuator`, `MocoZeroMomentPointConstraint`) in a `MocoTrack` problem to track states and ground reaction forces while producing a dynamically consistent solution. The tracking problem leverages the equations governing the Zero Moment Point to ensure that external ground reactions applied by the `BodyWrenchActuator` are consistent with the method. 

## Background

The Zero Moment Point method estimates the net ground reaction forces acting on a body by computing the residual forces at the model's free joint (typically the pelvis) via inverse dynamics, and then projecting the net moment to the ground plane to find the centre of pressure. This approach avoids the need for contact sphere models or explicit ground contact formulations, making it well-suited to kinematics-driven analyses. This example demonstrates using a ZMP-driven approach to a tracking simulation that would normally use contact spheres to track GRF signals.

The method implemented here takes inspiration and leverages methods from as series of works:

Xiang, Y., Arora, J. S., Rahmatalla, S., & Abdel‐Malek, K. (2009). [Optimization‐based dynamic human walking prediction: One step formulation](https://doi.org/10.1002/nme.2575). *International Journal for Numerical Methods in Engineering*, *79*(6), 667-695.

Dijkstra, E. J., & Gutierrez-Farewik, E. M. (2015). [Computation of ground reaction force using zero moment point]([Redirecting](https://doi.org/10.1016/j.jbiomech.2015.08.027)). *Journal of Biomechanics*, *48*(14), 3776-3781.

Karčnik, T. (2003). [Using motion analysis data for foot-floor contact detection]([Client Challenge](https://doi.org/10.1007/BF02345310)). *Medical and Biological Engineering and Computing*, *41*(5), 509-512.

## Data

This example uses some sample data from:

Hamner, S. R., & Delp, S. L. (2013). [Muscle contributions to fore-aft and vertical body mass center accelerations over a range of running speeds]([Redirecting](https://doi.org/10.1016/j.jbiomech.2012.11.024)). *Journal of Biomechanics*, *46*(4), 780-787.

Specifically, the example includes the following files in the `data/` subdirectory:

| File                        | Description                                                                                                                                                                         |
| --------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `subject01_run3.osim`       | OpenSim musculoskeletal model.                                                                                                                                                      |
| `Run_3_grf.mot`             | Measured ground reaction forces (for reference and comparison). Used for contact tracking reference.                                                                                |
| `Run_3_grf.xml`             | `ExternalLoads` associated with the experimental ground reactions. Used for contact tracking reference.                                                                             |
| `subject01_run3_states.sto` | States data (values and speeds) that came from an existing Moco tracking solution. The tracking solution specifically comes from [Fox (2024)](https://doi.org/10.1098/rsos.231909). |

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

**`BodyWrenchActuator`**

A `Force` subclass that is very similar to the `BodyActuator` but adds in an additional control signal of the point location in the ground frame which gets transformed to the body to indicate where to apply the force and torque. 

**Properties:**

- `contact_body_name`: the name of the body that the force applies to (e.g. 'calcn_r'). If being used with the `MocoZeroMomentPointConstraint`, this contact body will need to match with a contact body name in the `ZeroMomentPointEstimator`.

**`MocoZeroMomentPointConstraint`**

A `MocoPathConstraint` subclass that effectively enforces that the `BodyWrenchActuator` objects used to apply the external ground reactions align to dynamics governed by the Zero Moment Point method. 

is very similar to the `BodyActuator` but adds in an additional control signal of the point location in the ground frame which gets transformed to the body to indicate where to apply the force and torque.

**Properties:**

- `zmp_estimator_path`: the full path to the `ZeroMomentPointEstimator` included in the model. 
- `body_wrench_paths`: a list of the full paths to the `BodyWrenchActuator` objects used to apply the ground reactions. 

**`MocoContactPointConstraint`**

A `MocoPathConstraint` subclass that takes a list of `ContactPoint` object paths and enforces that these points remain above a specified ground plane height. This would enforce something similar to a standard `MocoOutputConstraint` focused on the point location, however adding an individual constraint for each contact point could slow down computation. Encapsulating these in a single constraint likely speeds up computation.

**Properties:**

- `contact_point_paths`: a list of the full paths to the `ContactPoint` objects to consider with this constraint.
- `ground_plane_height`: the value in the y-axis to enforce as the ground plane. This defaults to 0.0 but could be altered to assign a higher ground plane depending on lab setups.

## Example Outline

### Step 1: Create the Model with ZMP Components

This step runs the `createModelWithZMPForces()` function, adding the `ZeroMomentPointEstimator` with its relevant sub-components, and the `BodyWrenchActuator` objects. The saved model is used in subsequent tracking simulations.

#### Output Files

| File                           | Description                                              |
| ------------------------------ | -------------------------------------------------------- |
| `subject01_run3_with_zmp.osim` | Created model with relevant ZMP and actuator components. |

### Step 2: Run a States <u>and</u> GRF Tracking Simulation

This step runs the `runTrackingProblem()` function with the GRF contact tracking flag set to `true`. This sets up a torque-driven tracking simulation which aims to minimise torque control signals while tracking the model states (i.e. values and speeds) and tracking the experimental GRFs with the `BodyWrenchActuator` objects. The force, GRF contact tracking error is therefore govenred by the difference in the force, torque and point signals of the `BodyWrenchActuator` objects against the experimental GRFs. The `MocoZeroMomentPointConstraint` is also included to ensure that the signals of the actuators are governed by the ZMP equations as well. The signals of the actuators are not included in the control minimisation goal to avoid conflict between this and the tracking goal. A `MocoContactPointConstraint` is also included in the tracking problem to govern 'ground contact' in some form given the lack of contact spheres. This constraint likely isn't needed given the model is supported by external actuators that should prevent ground penetration — but still serves a likely useful purpose in avoiding the solver attempting to find a solution where ground contact occurs below the ground plane.  

#### Output Files

| File                                        | Description                                                                                                                                                                        |
| ------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `initial_guess.sto`                         | The initial guess created for setting in the tracking problem.                                                                                                                     |
| `states-grf_tracking-solution_with-zmp.sto` | The solution from the Moco tracking problem. The controls for the `BodyWrenchActuator` objects included with the model can essentially be viewed as the external ground reactions. |

### Step 3: Run a States Tracking Simulation <u>without</u> GRF Tracking

This step re-runs the `runTrackingProblem()` function with the GRF contact tracking flag set to `false`. Everything is the same as the above step with the exception of the control goal that tracks the actuator signals not being included. This demonstrates how simply governing the `BodyWrenchActuator` control signals with the `MocoZeroMomentPointConstraint` can result in external ground reactions reflective of the experimental data.

#### Output Files

| File                                         | Description                                                                                                                                                                        |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `initial_guess.sto`                          | The initial guess created for setting in the tracking problem. This will overwrite what is created in step 3.                                                                      |
| `states-only_tracking-solution_with-zmp.sto` | The solution from the Moco tracking problem. The controls for the `BodyWrenchActuator` objects included with the model can essentially be viewed as the external ground reactions. |

## Notes

- The ZMP method assumes a flat ground plane at y = 0. If using a different model
  with a different vertical axis or ground height it would need to be converted to the same coordinate system and configuration for the estimator to work.

- The running task included here only contains unilateral contact. Bilateral contact (i.e. double support) is not yet fully implemented with the Zero Moment Point method as the problem is indeterminant. During double support phases the estimator will simply print a warning and return zeroed outputs for both sides, and the tracking problem will likely struggle to solve as the constraints can't be met.
