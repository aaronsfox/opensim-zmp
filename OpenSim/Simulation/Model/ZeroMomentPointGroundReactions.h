#ifndef OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H
#define OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H
/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointGroundReactions.h                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Aaron Fox                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// TODO: what to include?

#include <OpenSim/Moco/osimMocoDLL.h>
#include <unordered_map>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {

/** Object class that holds the parameters required to calculate zero moment
point estimates of ground reactions and centre of pressure. */
class OSIMSIMULATION_API ZeroMomentPointGroundReactions_ZMPBodyList 
        : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ZeroMomentPointGroundReactions_ZMPBodyList, Object);

public:

    //=========================================================================
    // PROPERTIES (for individual contact bodies)
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(body_name, std::string,
            "The name of the contact body to which the ZeroMomentPointGroundReactions "
            " is connected to. Each body is checked for gound contact when calculating "
            "ZMP ground reactions.");
    
    // NOTE: these could be better structured by sitting stations within an object that
    // sits within the ZMP body list object

    OpenSim_DECLARE_LIST_PROPERTY(zmp_checkpoints, SimTK::Vec3,
        "The series of points associated with the contact body which will "
        "be queried to determine whether the points are in contact with the "
        "ground.");

    ZeroMomentPointGroundReactions_ZMPBodyList();

    //=========================================================================
    // METHODS
    //=========================================================================

    /*const Body& getContactBody() const { return getConnectee<Body>("body"); }*/

    /** Add a ZMP checkpoint for reviewing ground contact. */
    void addCheckpoint(const SimTK::Vec3& zmp_checkpoint);

private:
    void constructProperties();

};

/** TODO: include description and info

TODO: include papers...

Xiang et al. 2009: https://doi.org/10.1002/nme.2575 
Dijkstra & Gutierrez-Farewik 2015: https://doi.org/10.1016/j.jbiomech.2015.08.027
*/

class OSIMSIMULATION_API ZeroMomentPointGroundReactions
        : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ZeroMomentPointGroundReactions, ModelComponent);

public:

    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_LIST_PROPERTY(zmp_body_list,
            ZeroMomentPointGroundReactions_ZMPBodyList,
            "Parameters for each ground contact body specified in component.");

    OpenSim_DECLARE_PROPERTY(free_joint_name, std::string,
            "The name of the free joint (i.e. the joint that connects the "
            "model to the ground) in the model (defaults to ground_pelvis).");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(distance_threshold, double,
            "The absolute distance that checkpoints associated with ZMP contact "
            "bodies are checked against to determine whether the body is in contact "
            "with the ground plane (i.e. distance along y-axis) (defaults to 0.01).");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_threshold, double,
            "Specify the threshold for vertical force (in N) where we consider ground "
            "contact to have occurred (i.e. vertical force above this threshold "
            "will indicate ground contact) and when force is considered (defaults to "
            "20N).");

    //=========================================================================
    // SOCKETS
    //=========================================================================


    //=========================================================================
    // OUTPUTS --- TODO: do these outputs ignore multiple contact bodies?
    // Should they be in array format to store relative to each contact body?
    // They should probably be calculated with respect to a chosen contact body?
    //=========================================================================

    OpenSim_DECLARE_OUTPUT(ground_reactions, SimTK::Vector,
            calcGroundReactions, SimTK::Stage::Dynamics);
    
    /*OpenSim_DECLARE_OUTPUT(ground_reaction_forces, SimTK::Vec3,
            getGroundReactionForces, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(ground_reaction_moments, SimTK::Vec3,
            getGroundReactionMoments, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(ground_reaction_points, SimTK::Vec3,
            getGroundReactionPoints, SimTK::Stage::Dynamics);;*/

    //=========================================================================
    // METHODS
    //=========================================================================

    ZeroMomentPointGroundReactions();

    // TODO: more convenience constructors for the entire component?

    /** Get the number of contact bodies added to the component by one of the 
    `addContactBody()` overloads. */
    int getNumContactBodiesZMP() const;

    /** Specify a contact body that should be considered when reviewing if
    the model has come into contact with the ground plane. */
    void addContactBodyZMP(
            const std::string& name, const std::string& body_name);

    // TODO: other ways in which contact body can be added? Other parameters...?
    
    /** Set the free joint name in the component. */
    void setFreeJointName(const std::string& free_joint_name);

    /** Set the distance threshold for a checkpoint to be considered
    in contact with the ground plane. */
    void setDistanceThreshold(const double& distance_threshold);

    /** Set the force threshold for considering when ground contact
    has occurred. */
    void setForceThreshold(const double& force_threshold);
    
    /** These functions calculate the Zero Moment Point of the model based
    on the Model state. It identifies the ground reaction forces, moments
    and centre of pressure for each contact body connected to the component.
    The output is returned as a Vector which size is based on the number of
    contact bodies and the separate force, moment and point components,
    
    i.e.

        FXn, FYn, FZn, MXn, MYn, MZn, PXn, PYn, PZn
    
    where n is repeated for the number of contact bodies specified.*/
    SimTK::Vector calcGroundReactions(const SimTK::State& s) const;
    SimTK::Vector calcGroundReactions(const SimTK::State& s,
        const SimTK::Vector& udot, bool unilateralContact) const;


    /*SimTK::Vec3 getGroundReactionForces(const SimTK::State& s) const;
    SimTK::Vec3 getGroundReactionMoments(const SimTK::State& s) const;
    SimTK::Vec3 getGroundReactionPoints(const SimTK::State& s) const;*/

private:
    void constructProperties();
    void extendFinalizeFromProperties() override;
    
};

} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H
