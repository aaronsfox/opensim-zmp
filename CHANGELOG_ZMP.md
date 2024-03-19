Zero Moment Point (ZMP) Change Log
===============

**ADDED:**

- `ZeroMomentPointGroundReactions` component to Model components
  - See `ZeroMomentPointGroundReactions.cpp/h` for details

**TODO:**

- **<u>Class structures</u>**
  - `ZeroMomentPointGroundReactions` as a *model component*
    - `ZeroMomentPointContactBody` as an *object* that can be used within the overarching component
      - `ZeroMomentPointContactBodyPoint` as an *object* that can be used within the contact body object (i.e. store points that are queried for ground contact)
  - `ZeroMomentPointForce` as a *force* that is connected to the overarching component
    - Can a point force apply multiple separate forces, or does one need to be connected to each contact body? Everything probably needs to be connected to the overarching component as there are times when multiple forces/contact points are applied.
    - Alternatively, it might be easier to have a single force for each contact body, and connect it up to that contact body via setting names or some other parameter. The index of the contact body could be found in some way, and therefore the force and COP components for that body be identified and used with the force.
  - `MocoZeroMomentPointTrackingGoal` as a *Moco Goal* that can track forces, moments and COP
  - `ZeroMomentPointCopConstrain` as a *constraint* that somehow constrains the COP to remain within boundaries of contact points, or maybe even nearby (i.e. what if only 1 or 2 points are in contact with ground)
    - If nearby, it would need a distance parameter to ensure the COP isn't more than X distance from a contact point
- **<u>Create example scripts that demonstrate component use:</u>**
  - `exampleZeroMomentPoint1_createAndConnectComponents.cpp`: simple script to show how components can be created and connected to a blank model
    - ***Done OK - consider adding force class to example***
  - `exampleZeroMomentPoint2_estimateFromMotion.cpp`: estimating GRFs and COP from existing kinematics (i.e. passing udot through generated from speeds) — much simpler than state-to-state computation
  - `exampleZeroMomentPoint3_estimateFromState.cpp`: estimating GRFs and COP from a singular state — which would probably require more complete simulation inputs (i.e. underlying forces etc.)
  - `exampleZeroMomentPoint4_trackingSimulation.cpp`: running a Moco tracking problem using the component
  - `exampleZeroMomentPoint5_predictiveSimulation.cpp`: running a Moco predictive problem and exporting the GRFs and COP estimated from the component?

