#ifndef OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H
#define OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H

/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointGroundReactions.h                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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
#include <OpenSim/Simulation/Model/ZeroMomentPointContactBodySet.h>

#include "OpenSim/Moco/MocoTrajectory.h"

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * The ZeroMomentPointGroundReactions is a model component used to calculate
 * estimates of ground reactions (i.e. forces, moments and point of application)
 * from a model at a particular state. This allows a prediction of the ground 
 * reactions from a motion where experimental measures were unavailable. The
 * component requires the bodies that are expected to come into contact with the
 * ground to be specified, alongside points on the contact bodies (or nearby bodies)
 * that are checked for specifying when ground contact has occurred. The component
 * can also be connected to the ZeroMomentPointContactForce class so that the
 * estimated ground reactions can be applied as external forces to the model during
 * dynamic simulations.
 * 
 * Ground reactions are estimated via the Zero Moment Point method when ground contact
 * is identified. Ground contact is identified by first checking if the contact
 * points meet the criteria for ground contact - either based on distance from the 
 * ground plane (which is assumed to be the XZ plane) or by both distance to the 
 * ground plane and meeting a velocity threshold. A force threshold is also specified
 * for if ground contact has occurred, whereby if the vertical force (assumed to be 
 * in the y-direction) is greater than the force threshold then ground contact is
 * occurring. 
 *
 * The Zero Moment Point method is outlined in Xiang et al. (2009) and an implementation
 * to predicting GRFs during gait explored by Dijkstra & Gutierrez-Farewik (2015).
 * The method for checking ground contact with points on a body is similar to the
 * method implemented in Karcnik (2003).
 *  
 * Xiang et al. (2009): https://doi.org/10.1002/nme.2575
 * Dijkstra & Gutierrez-Farewik (2015): https://doi.org/10.1016/j.jbiomech.2015.08.027
 * Karcnik  (2003): https://doi.org/10.1007/BF02345310).
 * 
 * @authors Aaron Fox
 * @version 1.0
 */
class OSIMSIMULATION_API ZeroMomentPointGroundReactions
        : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ZeroMomentPointGroundReactions, ModelComponent);

public:

    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_UNNAMED_PROPERTY(ZeroMomentPointContactBodySet,
            "Component set that holds the parameters for each ground contact "
            "body "
            "specified in component.");

    OpenSim_DECLARE_PROPERTY(free_joint_name, std::string,
            "The name of the free joint (i.e. the joint that connects the "
            "model to the ground) in the model (defaults to ground_pelvis).");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_threshold, double,
            "Specify the threshold for vertical force (in N) where we consider "
            "ground contact to have occurred (i.e. vertical force above this "
            "threshold will indicate ground contact) and when force is considered "
            "(defaults to 20N).");

    //=========================================================================
    // OUTPUTS
    //=========================================================================

    // TODO: make this more like the muscle outputs from Bhargava model with unordered map etc.
    // TODO: create separate options for forces, moments and points for each contact body (Vec3)
        // > Again, see how this is done with metabolic rate of muscles in Bhargava models

    // TODO: is this actually needed here, or just get from force object?

    OpenSim_DECLARE_LIST_OUTPUT(contact_body_forces, SimTK::Vec3,
            getContactBodyForces, SimTK::Stage::Dynamics);

    OpenSim_DECLARE_LIST_OUTPUT(contact_body_moments, SimTK::Vec3,
            getContactBodyMoments, SimTK::Stage::Dynamics);

    OpenSim_DECLARE_LIST_OUTPUT(contact_body_point, SimTK::Vec3,
            getContactBodyPoint, SimTK::Stage::Dynamics);

    //=========================================================================
    // CONSTRUCTORS
    //=========================================================================

    /** Default constructor */
    ZeroMomentPointGroundReactions();

    /** Constructor specifying free joint name with other defaults. */
    ZeroMomentPointGroundReactions(const std::string& freeJointName);

    /** Constructor specifying free joint name and force threshold. */
    ZeroMomentPointGroundReactions(const std::string& freeJointName,
            const double& forceThreshold);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Method for returning contact point set */
    ZeroMomentPointContactBodySet& updZeroMomentPointContactBodySet() {
        return upd_ZeroMomentPointContactBodySet();
    }

    /** Get the ZeroMomentPointContactBody based on the name.*/
    /*ZeroMomentPointContactBody getContactBody(const std::string& name);*/

    /** Specify a contact body that should be considered when reviewing if
    the model has come into contact with the ground plane. Uses default
    value of velocity for contact checking method.*/
    void addContactBody(
            const std::string& name, const std::string& bodyName);

    /** Specify a contact body that should be considered when reviewing if
    the model has come into contact with the ground plane, while also
    specifying the contact checking method.*/
    void addContactBody(
            const std::string& name, const std::string& bodyName,
            const std::string& contactCheckingMethod);

    /** Get and set the free joint name in the component. */
    const std::string& getFreeJointName() const {
        return get_free_joint_name();
    }    
    void setFreeJointName(const std::string& free_joint_name);

    /** Get and set the force threshold for considering when ground contact
    has occurred. */
    const double& getForceThreshold() const {
        return get_force_threshold();
    }    
    void setForceThreshold(const double& force_threshold);

    //=========================================================================
    // CALCULATIONS
    //=========================================================================

    // TODO: return ground reactions as a Vector of SpatialVecs associated
    // with each contact body

    /** Check ground contact of each contact body specified. Returns a vector
    of 1 or 0 for each contact body specifying if contact has or has not
    occurred. */
    SimTK::Vector checkGroundContact(const SimTK::State& s) const; 

    /** The below functions calculate the Zero Moment Point of the model based
    on the Model state and any other provided inputs. It identifies the ground 
    reaction forces, moments and centre of pressure for each contact body
    listed in the component. The output is returned as a Vector which size is
    based on the number of contact bodies. Each component of the Vector contains
    a spatial vector that contains the separate force, moment and point components.

    i.e.
        
        FX, FY, FZ, PX, PY, PZ, MX, MY, MZ

    which is repeated for the number of contact bodies in the component.*/

    /** Calculate ground reactions with state only.*/
    SimTK::Vector getGroundReactions(const SimTK::State& s) const;

    /** Calculate ground reactions with state and udot accelerations vector.*/
    SimTK::Vector getGroundReactions(
            const SimTK::State& s,
            const SimTK::Vector& udot) const;
 
    /** These functions calculate the Zero Moment Point of the model based
    on a series of states from a predefined motion. It identifies the ground
    reaction forces, moments and centre of pressure for each contact body 
    listed in the component across the states provided. The output is returned
    as a Storage with the number of columns based on the number of contact bodies
    and the separate force, moment and point components.

    i.e.

        FXn, FYn, FZn, PXn, PYn, PZn, MXn, MYn, MZn

    where n is repeated for the number of contact bodies specified.*/

    /** Calculate ground reactions from a provided states trajectory and
    accelerations table. */
    Storage getGroundReactionsFromMotion(
        const StatesTrajectory& states, const TimeSeriesTable& udot) const; 

    //=========================================================================
    // OUTPUTS
    //=========================================================================

    // NOTE: bodyName for outputs is name of ZeroMomentPointContactBodyObject...

    /** Get the forces estimated for the specified contact body. */
    SimTK::Vec3 getContactBodyForces(
            const SimTK::State& s, const std::string& bodyName) const;

    /** Get the moments estimated for the specified contact body. */
    SimTK::Vec3 getContactBodyMoments(
            const SimTK::State& s, const std::string& bodyName) const;

    /** Get the point of application estimated for the specified contact body. */
    SimTK::Vec3 getContactBodyPoint(
            const SimTK::State& s, const std::string& bodyName) const;


private:

    void constructProperties();
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendRealizeTopology(SimTK::State&) const override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // Create map for accessing contact body via names
    mutable std::unordered_map<std::string, int> m_contactBodyIndices;

    // Cache variables
    /*mutable CacheVariable<SimTK::Vector> _groundReactionsCV;*/

};



} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H
