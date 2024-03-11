Zero Moment Point (ZMP) Additional Change Log
===============

**TODO:**

- Consider need for component vs. model outputs
- Consider generating model outputs that quantify ZMP ground reactions based on inputs:
  - `Model::ZeroMomentPointGroundReactions(s, "bodyName")`
    - Calculate on the basis of the model state and a single listed body
    - Would ignore any considerations around splitting ground reactions across bodies and any force threshold
    - Outputs the Fx, Fy, Fz, Mx, My, Mz, Px, Py, Pz in a `SimTK::Vector`
    - Without extra inputs model needs to be able to determine additional parameters (e.g. free body)
    - Would this need a contact distance threshold of sorts if contact points aren't there?
  - `Model::ZeroMomentPointGroundReactions(s, "bodyName", forceThreshold)`
    - As above, but with an integer input for the force threshold to consider ground contact to have occurred
  - `Model::ZeroMomentPointGroundReactions(s, ArrayStr())`
    - As above, but multiple body names input in array string format
    - Would consider distribution of ground reactions to bodies on some basis of contact points
      - Would therefore probably need additional input
- A model output solution could probably be more easily ported across to other tools (e.g. MocoTracking goal)?

