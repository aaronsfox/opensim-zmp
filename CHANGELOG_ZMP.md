Zero Moment Point (ZMP) Change Log
===============

The changes to the OpenSim core code associated with the Zero Moment Point approaches included as part of this repository are summarised below.

## Classes

**TODO: Class names should be more generic (i.e. ContactBody) so they could be leveraged by other components/classes later on...**

- The `ZeroMomentPointGroundReactions` class was added as a `ModelComponent`. This class serves as a base for most calculations associated with the ZMP estimates of ground reactions.
- The `ZeroMomentPointContactBodySet` was added as a `ModelComponentSet`. This serves as a convenience class to store `ZeroMomentContactBody` objects within the main `ZeroMomentPointGroundReactions` component.
- The `ZeroMomentPointContactBody` class was added as a `ModelComponent`. This class sits underneath the `ZeroMomentPointGroundReactions` object within a `ZeroMomentPointContactBodySet` to identify bodies in the model that are expected and checked for ground contact.
- The `ZeroMomentPointContactPointSet` was added as a `ModelComponentSet`. This serves as a convenience class to store `ZeroMomentContactPoint` objects within each `ZeroMomentPointContactBody` component.
- The `ZeroMomentPointContactPoint` class was added as a `Station`. This class sits underneath the `ZeroMomentPointContactBody` object within a `ZeroMomentContactPointSet` to specify points on a body that are queried with respect to their distance relative to the ground and (if requested) velocity. Thresholds are provided with each point to determine if the point is deemed in contact with the ground (i.e. the values are under the thresholds). 
- **TODO:** `ZeroMomentPointContactForce` as a `Force` class
  - A force will be connected to each contact body, subsequently reading the forces to apply to the body at the designated point
- **TODO:** `ZeroMomentPointContactPointConstraint` as a `UnilateralConstraint` class
  - Points that prevent contact penetration with the ground
- **TODO:** `MocoZeroMomentPointContactTrackingGoal`
  - `MocoGoal` that tracks the forces from the `ZeroMomentPointContactForce` object in a similar way that is done to the contact tracking goal with contact spheres
  - Can consider that this could track both forces and COP
- TODO: `ZeroMomentPointCopConstraint`
  - Some sort of `Constraint` that somehow constrains the COP to remain within the boundaries or near the contact points. If nearby it would need a distance parameter to ensure the COP isn't more than a certain distance from estimated point. 
- **NOTE:** potentially a little bloated with the number of classes needed for this...



## Examples

### ZMP Component Demo

This example demonstrates how to create and connect the various new classes to a model, alongside a few demonstrations of key functions related to the ZMP estimates of ground reactions (e.g. checking for ground contact). 

### ZMP Ground Reactions from Motion

This example uses the `ZeroMomentPointGroundReactions` class with the `ZeroMomentPointContactBody` and `ZeroMomentPointContactBodyPoint` classes to estimate ground reactions (forces, centre of pressure, moments) from a specified motion. The motion comes from a Moco tracking solution, with a states trajectory and table of accelerations created to calculate the ground reactions. The data comes from treadmill running, therefore the ground contact points are checked using the distance only method as the velocity of the contact points is influenced by the foot moving on the treadmill. The output of this example is a `.mot` file similar to what you would create from experimental ground reaction forces (and subsequently create an associated `ExternalLoads` `.xml` file for).

**TODO:**

- `exampleZmpGroundReactionsFromState.cpp`: estimating GRFs and COP from a singular state — which would probably require more complete simulation inputs (i.e. underlying forces etc.)
- `exampleZmpContactForce.cpp`: working through a series of states apply the forces from the ZMP estimates to a model. Run inverse dynamics on this plus the original model with external loads included, and theoretically the inverse dynamics outputs should be similar.
- `exampleZmpTrackingSimulation.cpp`: running a Moco tracking problem using the component
- `exampleZmpPredictiveSimulation.cpp`: running a Moco predictive problem and exporting the GRFs and COP estimated from the component?



## Bindings

- Added the various `ZeroMomentPoint*` classes to `OpenSimHeaders_simulation.h`



## Tests

- Added `test_zmp_component.py` to Python bindings test. **TODO: add details on tests in here...**