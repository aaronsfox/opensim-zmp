# Example: Fluit 2014 Contact Point Actuator

This example provides a simple demonstration for adding the `Fluit2014ContactPointActuator` to a model.

## Background

The formulation of this actuator class stems from implementation across two papers:

Fluit R, Andersen MS, Kolk S, Verdonschot N, Koopman HFJM (2014). [Prediction of ground reaction forces and moments during various activities of daily living](http://dx.doi.org/10.1016/j.jbiomech.2014.04.030). *Journal of Biomechanics*, 47(10), 2321-2329.

Skals S, Jung MK, Damsgaard M, Andersen MS (2017). [Prediction of ground
reaction forces and moments during sports-related movements](http://dx.doi.org/10.1007/s11044-016-9537-4). *Multibody
System Dynamics*, 39, 175-195.

Some modifications to the approach used in these papers is made to hopefully make this component work better with gradient-based solvers, including the use of a smoothing function to scale the force capacity based on the distance and velocity thresholds for identifying ground contact.

## Data

This example uses some sample data from:

Hamner, S. R., & Delp, S. L. (2013). [Muscle contributions to fore-aft and vertical body mass center accelerations over a range of running speeds]([Redirecting](https://doi.org/10.1016/j.jbiomech.2012.11.024)). *Journal of Biomechanics*, *46*(4), 780-787.

Specifically, the example includes a single model from this dataset (i.e. `subject01_run3.osim`) generated through the work of [Fox (2024)](https://doi.org/10.1098/rsos.231909).

## Component Overview

### `Fluit2014ContactPointActuator`

```xml
<ContactPointActuator name="RHeel_contact">
    <body>calcn_r</body>
    <point>0.01 0.0 0.0</point>
    <point_is_global>false</point_is_global>
    <max_normal_force>280.0</max_normal_force>
    <ground_height>0.0</ground_height>
    <contact_height_threshold>0.03</contact_height_threshold>
    <contact_velocity_threshold>0.8</contact_velocity_threshold>
    <smoothing>20.0</smoothing>
    <friction_coefficient>0.5</friction_coefficient>
    <friction_transition_velocity>0.01</friction_transition_velocity>
    <min_control>0.0</min_control>
    <max_control>1.0</max_control>
</ContactPointActuator>
```

At a fixed point on a body, this actuator computes:

1. The point's height above a horizontal ground plane (`ground_height`, a Y-coordinate in Ground) and its 3D speed in the ground frame.
2. Converts these into the dimensionless ratios used by Fluit (Eq. 1) and Skals (Eq. 2): `z_ratio = height / contact_height_threshold`, `v_ratio = speed / contact_velocity_threshold`.
3. Computes a smooth strength-scaling factor `c = sigmoid(z_ratio) * sigmoid(v_ratio)` using logistic functions centered at ratio = 1, with steepness set by the single `smoothing` property (`k`). As `k -> infinity`, `c` converges to Fluit's original hard 0/1 threshold; smaller `k` gives a gentler, more differentiable ramp -- useful for gradient-based solvers, which is the reason this generalizes (rather than literally reimplements) the cosine-windowed smoothing in the two papers, which relied on knowing gait-event timing in advance.
4. Sets the **normal** (vertical, ground **+Y**) force to `F_n = control * max_normal_force * c`. This is this actuator's scalar "actuation" (`getActuation()`), and is the *only* quantity driven directly by the actuator's single control input.
5. Sets the **shear** (medio-lateral / antero-posterior, ground **X-Z**) force so that its magnitude is bounded by `friction_coefficient * F_n` (Skals, Sec. 2.5), opposing the point's horizontal slip velocity, with the magnitude ramped smoothly (via `tanh`) from 0 up to that bound over a `friction_transition_velocity`, which replicates the style of smoothing `SmoothSphereHalfSpaceForce` uses for its friction term.
6. Applies the combined 3-D force (`F_shear_x`, `F_n`, `F_shear_z`) to the body at the specified point, and reports it as an Actuator "record" (so it shows up automatically in `ForceReporter`/`AnalyzeTool` output) with labels `<name>.force.X`, `<name>.force.Y`, `<name>.force.Z`.

The idea for this class is to place one actuator at relevant contact points along the feet in a similar way to that of Fluit et al. (Fig. 1: 12 points per foot) or Skals et al. (Fig. 2: 18 points per foot).

#### The Smoothing Parameter

`smoothing` (`k`) is dimensionless and applies identically to both the position- and velocity-based ratios. As a starting point:

- `k` in the 10-30 range should give a visibly smooth, well-conditioned transition (recommended starting point for optimal control).
- `k` > ~200 makes the transition steep enough to behave almost exactly like Fluit's original hard threshold (useful if you want to sanity-check against, or reproduce, static-optimization-based inverse dynamics results).

Because both `z_ratio` and `v_ratio` are dimensionless, the actual spatial "softness" (in meters) of the position transition is roughly `contact_height_threshold / k`, and the velocity "softness" (in m/s) is roughly `contact_velocity_threshold / k`; scale `k` accordingly if you change those thresholds substantially.
