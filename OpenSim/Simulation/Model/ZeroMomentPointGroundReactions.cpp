/* -------------------------------------------------------------------------- *
 * OpenSim: ZeroMomentPointGroundReactions.cpp                                *
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

// TODO: any other inclusions?

// TODO: consider overall component structure as to what will work best
// e.g. names vs. sockets, contact point specification etc. ...

#include "ZeroMomentPointGroundReactions.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
//  ZeroMomentPointGroundReactions_ContactBodyParameters
//=============================================================================

ZeroMomentPointGroundReactions_ContactBodyParameters::
        ZeroMomentPointGroundReactions_ContactBodyParameters() {
    constructProperties();
}

void ZeroMomentPointGroundReactions_ContactBodyParameters::
        constructProperties() {

    /*TODO: include properties here...*/

    //// Specific tension of mammalian muscle (Pascals (N/m^2)).
    //constructProperty_specific_tension(0.25e6);
    
}

//=============================================================================
//  ZeroMomentPointGroundReactions
//=============================================================================

ZeroMomentPointGroundReactions::ZeroMomentPointGroundReactions() {
    constructProperties();
    
    /*const int curvePoints = 5;
    const double curveX[] = {0.0, 0.5, 1.0, 1.5, 10.0};
    const double curveY[] = {0.5, 0.5, 1.0, 0.0, 0.0};
    m_fiberLengthDepCurve = PiecewiseLinearFunction(curvePoints, curveX,
            curveY, "defaultCurve");*/

}

//=============================================================================
//  Adding contact bodies
//=============================================================================

// Add a contact body specified with a name label and the contact body name
// TODO: there are likely other parameters to include here around contact points
// Contact points maybe default to (0,0,0) origin of body?
void ZeroMomentPointGroundReactions::addContactBody(
        const std::string& name,
        const Body& contact_body) {
   
    // Get updated parameters for contact body
    auto& cbp =
        upd_contact_body_parameters(getProperty_contact_body_parameters().size() - 1);

    /*TODO: consider what contact body parameters are coming across...*/

    // Set the name as specified
    cbp.setName(name);

    // Connect the body
    cbp.connectSocket_body(muscle);

}

// TODO: add any other addContactBody constructors...

// Construct base component properties

//=============================================================================
//  Constructing and finalizing component
//=============================================================================

void ZeroMomentPointGroundReactions::constructProperties() {
    
    constructProperty_contact_body_parameters();

    // Free body joint name (default = ground_pelvis)
    constructProperty_specific_tension("ground_pelvis");

    // Vertical force axis (default = SimTK::Vec3(0,1,0) i.e. y-axis)
    constructProperty_vertical_force_axis(SimTK::Vec3(0, 1, 0));

    // Vertical force threshold specifying ground contact (default = 20N)
    constructProperty_force_threshold(20.0);

}

void Bhargava2004SmoothedMuscleMetabolics::extendFinalizeFromProperties() {
    if (get_use_smoothing()) {
        m_tanh_conditional = [](const double& cond, const double& left,
                const double& right, const double& smoothing, const int&) {
            const double smoothed_binary = 0.5 + 0.5 * tanh(smoothing * cond);
            return left + (-left + right) * smoothed_binary;
        };
        if (get_smoothing_type() == "tanh") {
            m_conditional = m_tanh_conditional;

        } else if (get_smoothing_type() == "huber") {
            m_conditional = [](const double& cond, const double& left,
                    const double& right, const double& smoothing,
                    const int& direction) {
                const double offset = (direction == 1) ? left : right;
                const double scale = (right - left) / cond;
                const double delta = 1.0;
                const double state = direction * cond;
                const double shift = 0.5 * (1 / smoothing);
                const double y = smoothing * (state + shift);
                double f = 0;
                if (y < 0) f = offset;
                else if (y <= delta) f = 0.5 * y * y + offset;
                else  f = delta * (y - 0.5 * delta) + offset;
                return scale * (f/smoothing + offset * (1.0 - 1.0/smoothing));
            };
        }
    } else {
        m_conditional = [](const double& cond, const double& left,
                const double& right, const double& smoothing, const int&) {
            if (cond <= 0) {
                return left;
            } else {
                return right;
            }
        };
        m_tanh_conditional = m_conditional;
    }
}

//=============================================================================
//  Main calculations
//=============================================================================

// TODO: output variable type for this calculation?
double ZeroMomentPointGroundReactions::calcZeroMomentPointGroundReactions(
        const SimTK::State& s) const {

    // This function calculates the Zero Moment Point of the model based on
    // the Model state. It identifies the ground reaction forces, moments
    // and centre of pressure for each contact body connected to the component.
    // The output is returned as a ... 
    // TODO: ...something like a vector:
    //    
        // FXn, FYn, FZn, MXn, MYn, MZn, PXn, PYn, PZn
    // 
    // where n is repeated for the number of contact bodies specified.

    // Get the model
    const auto& model = getModel();

    // Get the free body component and associated joint
    const auto& freeBody = getConnectee<Body>("free_body");
    const auto& freeJoint = model.updJointSet().get(freeBody);

    // Get the number of contact bodies
    const int nCB = getProperty_contact_body_parameters().size();

    //TODO:
    //    > Am I Getting variables appropriately?

    // OpenSim::Coordinates represent degrees of freedom for a model.
    // Each Coordinate's value and speed maps to an index
    // in the model's underlying SimTK::State (value to a slot in the
    // State's q, and speed to a slot in the State's u).
    // So we need to map each OpenSim::Coordinate value and speed to the
    // corresponding SimTK::State's q and u indices, respectively.
    auto coords = model.getCoordinatesInMultibodyTreeOrder();
    int nq = s.getNQ();
    int nu = s.getNU();
    int nCoords = (int)coords.size(); // TODO: needed?
    // int intUnusedSlot = -1; // TODO: needed?

    // TODO: does the mapCoordinateToQ vector from ID Tool need to be created here?

    // Initialise an inverse dynamics solver with model
    InverseDynamicsSolver ivdSolver(model);

    // Realize to appropriate stage
    // TODO: best stage for this...?
    model.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

    // Compute accelerations for current state
    // NOTE: unsure whether this produces the desired accelerations?
    // Different results using this versus an entire MocoTrajectory?
    SimTK::Vector udot = model.getMatterSubsystem().getUDot(s);

    // Solve inverse dynamics given current states and udot
    // The output vector contains the generalised coordinate forces
    // to generate the accelerations based on the current state.
    // Note that these aren't necessarily in the order of the
    // coordinate set, but rather the multibody tree order.
    SimTK::Vector genForceTraj = ivdSolver.solve(s, udot);

    ///* TODO: mapping if q != u in index(i.e.tree vs.model)... */

    // Calculate the equivalent body force at the free joint in the model
    SimTK::SpatialVec equivalentBodyForceAtJoint = freeJoint.calcEquivalentSpatialForce(s, genForceTraj);

    // Extract the body and torque components
    SimTK::Vec3 freeBodyTorque = equivalentBodyForceAtJoint.get(0);
    SimTK::Vec3 freeBodyForce = equivalentBodyForceAtJoint.get(1);
    
    // Get the position of the free body in the ground frame
    // TODO: how to identify this in states?
    SimTK::Vec3 rp;

    // Take the cross product of free body position and force vector to get moment at origin in ground
    SimTK::Vec3 groundM = Vec3( (rp.get(1) * freeBodyForce.get(2)) - (rp.get(2) * freeBodyForce.get(1)),
        -((rp.get(0) * freeBodyForce.get(2)) - (rp.get(2) * freeBodyForce.get(0))),
        (rp.get(0) * freeBodyForce.get(1)) - (rp.get(1) * freeBodyForce.get(0)) );

    // Calculate X & Z cZMP, noting that yZMP is set as 0 (must use standard OpenSim system)
    // Formulas come from Xiang et al. 2009: https://doi.org/10.1002/nme.2575
    SimTK::Vec3 zmpCOP = SimTK::Vec3(groundM.get(2) / freeBodyF.get(1), 0, -groundM.get(0) / freeBodyF.get(1));

    // TODO: determine whether contact bodies are in contact with ground...
    // Subsequent calculation steps are fairly dependent on this...
    // Xiang et al. denotes contact points on the foot that seemingly can't penetrate the ground
        // (e.g. some sort of constraint on these?)
    // When the vertical height (y-axis) of contact points = 0, ground contact is present
    // The ZMP is constrained to remain within these boundaries --- but that doesn't seem feasible
    // in a basic calculation...maybe in a dynamic simulation though...

}



double Bhargava2004SmoothedMuscleMetabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {

    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass).
    // ---------------------------------------------------------------------
    double Bdot = get_basal_coefficient()
            * pow(getModel().getMatterSubsystem().calcSystemMass(s),
                    get_basal_exponent());
    return getMetabolicRate(s).sum() + Bdot;
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalActivationRate(
        const SimTK::State& s) const {
    return getActivationRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalMaintenanceRate(
        const SimTK::State& s) const {
    return getMaintenanceRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalShorteningRate(
        const SimTK::State& s) const {
    return getShorteningRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalMechanicalWorkRate(
        const SimTK::State& s) const {
    return getMechanicalWorkRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getMuscleMetabolicRate(
        const SimTK::State& s, const std::string& channel) const {
    return getMetabolicRate(s).get(m_muscleIndices.at(channel));
}

void Bhargava2004SmoothedMuscleMetabolics::extendRealizeTopology(
        SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    m_muscleIndices.clear();
    for (int i = 0; i < getProperty_muscle_parameters().size(); ++i) {
        const auto& muscle = get_muscle_parameters(i).getMuscle();
        if (muscle.get_appliesForce()) {
            m_muscleIndices[muscle.getAbsolutePathString()] = i;
        }
    }
}

void Bhargava2004SmoothedMuscleMetabolics::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    SimTK::Vector rates = SimTK::Vector((int)m_muscleIndices.size(), 0.0);
    addCacheVariable<SimTK::Vector>("metabolic_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("activation_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("maintenance_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("shortening_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("mechanical_work_rate", rates,
            SimTK::Stage::Dynamics);
}

void Bhargava2004SmoothedMuscleMetabolics::calcMetabolicRateForCache(
    const SimTK::State& s) const {
    calcMetabolicRate(s,
            updCacheVariableValue<SimTK::Vector>(s, "metabolic_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "activation_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "maintenance_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "shortening_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "mechanical_work_rate")
            );
    markCacheVariableValid(s, "metabolic_rate");
    markCacheVariableValid(s, "activation_rate");
    markCacheVariableValid(s, "maintenance_rate");
    markCacheVariableValid(s, "shortening_rate");
    markCacheVariableValid(s, "mechanical_work_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getMetabolicRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "metabolic_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "metabolic_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getActivationRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "activation_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "activation_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getMaintenanceRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "maintenance_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "maintenance_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getShorteningRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "shortening_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "shortening_rate");
}

const SimTK::Vector&
Bhargava2004SmoothedMuscleMetabolics::getMechanicalWorkRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "mechanical_work_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "mechanical_work_rate");
}

void Bhargava2004SmoothedMuscleMetabolics::calcMetabolicRate(
        const SimTK::State& s, SimTK::Vector& totalRatesForMuscles,
        SimTK::Vector& activationRatesForMuscles,
        SimTK::Vector& maintenanceRatesForMuscles,
        SimTK::Vector& shorteningRatesForMuscles,
        SimTK::Vector& mechanicalWorkRatesForMuscles) const {
    totalRatesForMuscles.resize((int)m_muscleIndices.size());
    activationRatesForMuscles.resize((int)m_muscleIndices.size());
    maintenanceRatesForMuscles.resize((int)m_muscleIndices.size());
    shorteningRatesForMuscles.resize((int)m_muscleIndices.size());
    mechanicalWorkRatesForMuscles.resize((int)m_muscleIndices.size());
    double activationHeatRate, maintenanceHeatRate, shorteningHeatRate;
    double mechanicalWorkRate;
    activationHeatRate = maintenanceHeatRate = shorteningHeatRate =
        mechanicalWorkRate = 0;

    for (const auto& muscleIndex : m_muscleIndices) {

        const auto& index = muscleIndex.second;
        const auto& muscleParameter = get_muscle_parameters(index);
        const auto& muscle = muscleParameter.getMuscle();

        const double maximalIsometricForce = muscle.getMaxIsometricForce();
        const double activation =
            get_muscle_effort_scaling_factor() * muscle.getActivation(s);
        const double excitation =
            get_muscle_effort_scaling_factor() * muscle.getControl(s);
        const double fiberForcePassive =  muscle.getPassiveFiberForce(s);
        const double fiberForceActive =
            get_muscle_effort_scaling_factor() * muscle.getActiveFiberForce(s);
        const double fiberForceTotal =
            fiberForceActive + fiberForcePassive;
        const double fiberLengthNormalized =
            muscle.getNormalizedFiberLength(s);
        const double fiberVelocity = muscle.getFiberVelocity(s);
        const double slowTwitchExcitation =
            muscleParameter.get_ratio_slow_twitch_fibers()
            * sin(SimTK::Pi/2 * excitation);
        const double fastTwitchExcitation =
            (1 - muscleParameter.get_ratio_slow_twitch_fibers())
            * (1 - cos(SimTK::Pi/2 * excitation));
        // This small constant is added to the fiber velocity to prevent
        // dividing by 0 (in case the actual fiber velocity is null) when using
        // the Huber loss smoothing approach, thereby preventing singularities.
        const double eps = 1e-16;

        // Get the unnormalized total active force, isometricTotalActiveForce
        // that 'would' be developed at the current activation and fiber length
        // under isometric conditions (i.e., fiberVelocity=0).
        const double isometricTotalActiveForce =
            activation * muscle.getActiveForceLengthMultiplier(s)
            * maximalIsometricForce;

        // ACTIVATION HEAT RATE (W).
        // -------------------------
        // This value is set to 1.0, as used by Anderson & Pandy (1999),
        // however, in Bhargava et al., (2004) they assume a function here.
        // We will ignore this function and use 1.0 for now.
        const double decay_function_value = 1.0;
        activationHeatRate =
            muscleParameter.getMuscleMass() * decay_function_value
            * ( (muscleParameter.get_activation_constant_slow_twitch()
                        * slowTwitchExcitation)
                + (muscleParameter.get_activation_constant_fast_twitch()
                        * fastTwitchExcitation) );

        // MAINTENANCE HEAT RATE (W).
        // --------------------------
        const double fiber_length_dependence = m_fiberLengthDepCurve.calcValue(
                    SimTK::Vector(1, fiberLengthNormalized));
        maintenanceHeatRate =
            muscleParameter.getMuscleMass() * fiber_length_dependence
                * ( (muscleParameter.get_maintenance_constant_slow_twitch()
                            * slowTwitchExcitation)
                + (muscleParameter.get_maintenance_constant_fast_twitch()
                            * fastTwitchExcitation) );

        // SHORTENING HEAT RATE (W).
        // --> note that we define fiberVelocity<0 as shortening and
        //     fiberVelocity>0 as lengthening.
        // ---------------------------------------------------------
        double alpha;
        if (get_use_force_dependent_shortening_prop_constant()) {
            // Even when using the Huber loss smoothing approach, we still rely
            // on a tanh approximation for the shortening heat rate when using
            // the force dependent shortening proportional constant. This is
            // motivated by the fact that the shortening heat rate is defined
            // by linear functions but with different non-zero constants of
            // proportionality for concentric and eccentric contractions. It is
            // therefore easier to smooth the transition between both
            // contraction types with a tanh function than with a Huber loss
            // function.
            alpha = m_tanh_conditional(fiberVelocity + eps,
                    (0.16 * isometricTotalActiveForce)
                    + (0.18 * fiberForceTotal),
                    0.157 * fiberForceTotal,
                    get_velocity_smoothing(),
                    -1);
        } else {
            // This simpler value of alpha comes from Frank Anderson's 1999
            // dissertation "A Dynamic Optimization Solution for a Complete
            // Cycle of Normal Gait".
            alpha = m_conditional(fiberVelocity + eps,
                    0.25 * fiberForceTotal,
                    0,
                    get_velocity_smoothing(),
                    -1);
        }
        shorteningHeatRate = -alpha * (fiberVelocity + eps);

        // MECHANICAL WORK RATE for the contractile element of the muscle (W).
        // --> note that we define fiberVelocity<0 as shortening and
        //     fiberVelocity>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_include_negative_mechanical_work())
        {
            mechanicalWorkRate = -fiberForceActive * fiberVelocity;
        } else {
            mechanicalWorkRate = m_conditional(fiberVelocity + eps,
                    -fiberForceActive * fiberVelocity,
                    0,
                    get_velocity_smoothing(),
                    -1);
        }

        // NAN CHECKING
        // ------------------------------------------
        if (SimTK::isNaN(activationHeatRate))
            std::cout << "WARNING::" << getName() << ": activationHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(maintenanceHeatRate))
            std::cout << "WARNING::" << getName() << ": maintenanceHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(shorteningHeatRate))
            std::cout << "WARNING::" << getName() << ": shorteningHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(mechanicalWorkRate))
            std::cout << "WARNING::" << getName() << ": mechanicalWorkRate ("
                    <<  muscleParameter.getName() << ") = NaN!" << std::endl;

        // If necessary, increase the shortening heat rate so that the total
        // power is non-negative.
        if (get_forbid_negative_total_power()) {
            const double Edot_W_beforeClamp = activationHeatRate
                + maintenanceHeatRate + shorteningHeatRate
                + mechanicalWorkRate;
            if (get_use_smoothing()) {
                const double Edot_W_beforeClamp_smoothed = m_conditional(
                        -Edot_W_beforeClamp,
                        0,
                        Edot_W_beforeClamp,
                        get_power_smoothing(),
                        1);
                shorteningHeatRate -= Edot_W_beforeClamp_smoothed;
            } else {
                if (Edot_W_beforeClamp < 0)
                    shorteningHeatRate -= Edot_W_beforeClamp;
            }
        }

        // This check is adapted from Umberger(2003), page 104: the total heat
        // rate (i.e., activationHeatRate + maintenanceHeatRate
        // + shorteningHeatRate) for a given muscle cannot fall below 1.0 W/kg.
        // If the total heat rate falls below 1.0 W/kg, the sum of the reported
        // individual heat rates and work rate does not equal the reported
        // metabolic rate.
        // --------------------------------------------------------------------
        double totalHeatRate = activationHeatRate + maintenanceHeatRate
            + shorteningHeatRate;
        if (get_use_smoothing()) {
            if (get_enforce_minimum_heat_rate_per_muscle())
            {
                totalHeatRate = m_conditional(
                        -totalHeatRate + 1.0 * muscleParameter.getMuscleMass(),
                        totalHeatRate,
                        1.0 * muscleParameter.getMuscleMass(),
                        get_heat_rate_smoothing(),
                        1);
            }
        } else {
            if (get_enforce_minimum_heat_rate_per_muscle()
                    && totalHeatRate < 1.0 * muscleParameter.getMuscleMass())
            {
                totalHeatRate = 1.0 * muscleParameter.getMuscleMass();
            }
        }

        // TOTAL METABOLIC ENERGY RATE (W).
        // --------------------------------
        double Edot = totalHeatRate + mechanicalWorkRate;

        totalRatesForMuscles[index] = Edot;
        activationRatesForMuscles[index] = activationHeatRate;
        maintenanceRatesForMuscles[index] = maintenanceHeatRate;
        shorteningRatesForMuscles[index] = shorteningHeatRate;
        mechanicalWorkRatesForMuscles[index] = mechanicalWorkRate;
    }
}

//=============================================================================
//  Supplementary calculations
//=============================================================================

// Get number of contact bodies
int ZeroMomentPointGroundReactions::getNumContactBodies() const {
    return getProperty_contact_body_parameters().size();
}
