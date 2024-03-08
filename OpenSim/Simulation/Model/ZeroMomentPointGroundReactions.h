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

// #include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
// #include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

/** Object class that holds the parameters required to calculate zero moment
point estimates of ground reactions and centre of pressure. */
class OSIMSIMULATION_API ZeroMomentPointGroundReactions_ContactBodyParameters 
        : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ZeroMomentPointGroundReactions_ContactBodyParameters, Component);
public:

    //=========================================================================
    // PROPERTIES --- TODO: properties for contact body?
    //=========================================================================

    /*OpenSim_DECLARE_PROPERTY(density, double,
        "The density of the muscle (kg/m^3, default is 1059.7).");*/
    /*OpenSim_DECLARE_OPTIONAL_PROPERTY(provided_muscle_mass, double,
        "The user specified muscle mass (kg, default is NaN). When this "
        "property is NaN, the muscle mass is calculated as follows: "
        "(volume * density) / specific_tension) where "
        "volume = maximal_isometric_force * optimal_fiber_length.");*/

    //=========================================================================
    // SOCKETS --- TODO: nneeded if using names in parameters?
    //=========================================================================

    OpenSim_DECLARE_SOCKET(contact_body, Body,
            "The contact body to which the ZeroMomentPointGroundReactions is "
            "connected to. This body specifies contact with the ground plane.");

    ZeroMomentPointGroundReactions_ContactBodyParameters();

    const Body& getContactBody() const { return getConnectee<Body>("body"); }

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

    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_threshold, double,
            "Specify the threshold for vertical force (in N) where we consider ground "
            "contact to have occurred (i.e. vertical force above this threshold "
            "will indicate ground contact) and when force is considered (defaults to "
            "20N).");
    OpenSim_DECLARE_LIST_PROPERTY(contact_body_parameters,
            ZeroMomentPointGroundReactions_ContactBodyParameters,
            "Parameters for each ground contact body specified in component.");

    //=========================================================================
    // SOCKETS
    //=========================================================================

    OpenSim_DECLARE_SOCKET(free_body, Body,
            "The body that the component is connected to, which is the free body "
            "(i.e. connected to the ground) in the model.");
    OpenSim_DECLARE_SOCKET(half_space, ContactHalfSpace,
            "A half-space contact geometry that is indicative of the ground "
            "plane.");


    //=========================================================================
    // OUTPUTS --- TODO: these outputs ignore multiple contact bodies?
    // Should they be in array format to store relative to each contact body?
    // They should probably be calculated with respect to a chosen contact body?
    //=========================================================================

    OpenSim_DECLARE_OUTPUT(ground_reaction_forces, SimTK::Vec3,
            getGroundReactionForces, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(ground_reaction_moments, SimTK::Vec3,
            getGroundReactionMoments, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(ground_reaction_points, SimTK::Vec3,
            getGroundReactionPoints, SimTK::Stage::Dynamics);;

    OpenSim_DECLARE_LIST_OUTPUT(muscle_metabolic_rate, double,
            getMuscleMetabolicRate, SimTK::Stage::Dynamics);

    ZeroMomentPointGroundReactions();

    /** Get the number of contact bodies added to the component by one of the
    `addContactBody()` overloads. */
    int getNumContactBodies() const;

    /** Specify a contact body that should be considered when reviewing if
    the model has come into contact with the ground plane. */
    void addContactBody(
            const std::string& name, const std::string& contact_body);
    
    // TODO: other ways in which contact body can be added? Other parameters...?
    
    
    //** Bulit-in functions */

    SimTK::Vec3 getGroundReactionForces(const SimTK::State& s) const;
    SimTK::Vec3 getGroundReactionMoments(const SimTK::State& s) const;
    SimTK::Vec3 getGroundReactionPoints(const SimTK::State& s) const;

private:
    void constructProperties();
    void extendFinalizeFromProperties() override;
    void extendRealizeTopology(SimTK::State&) const override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void calcMetabolicRateForCache(const SimTK::State& s) const;
    const SimTK::Vector& getMetabolicRate(const SimTK::State& s) const;
    const SimTK::Vector& getActivationRate(const SimTK::State& s) const;
    const SimTK::Vector& getMaintenanceRate(const SimTK::State& s) const;
    const SimTK::Vector& getShorteningRate(const SimTK::State& s) const;
    const SimTK::Vector& getMechanicalWorkRate(const SimTK::State& s) const;
    void calcMetabolicRate(const SimTK::State& s,
            SimTK::Vector& totalRatesForMuscles,
            SimTK::Vector& activationRatesForMuscles,
            SimTK::Vector& maintenanceRatesForMuscles,
            SimTK::Vector& shorteningRatesForMuscles,
            SimTK::Vector& mechanicalWorkRatesForMuscles) const;
    mutable std::unordered_map<std::string, int> m_muscleIndices;
    using ConditionalFunction =
            double(const double&, const double&, const double&, const double&,
                    const int&);
    PiecewiseLinearFunction m_fiberLengthDepCurve;
    mutable std::function<ConditionalFunction> m_conditional;
    mutable std::function<ConditionalFunction> m_tanh_conditional;
};

} // namespace OpenSim

#endif // OPENSIM_ZEROMOMENTPOINTGROUNDREACTIONS_H
