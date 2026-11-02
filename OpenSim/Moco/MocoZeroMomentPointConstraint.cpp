/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoZeroMomentPointConstraint.cpp                            *
 * -------------------------------------------------------------------------- */

// TODO: could use some checks to see if Acutators are present
// TODO: might need a mode property to detect different force actuators if using?

#include "MocoZeroMomentPointConstraint.h"
#include "MocoProblemInfo.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Actuators/BodyWrenchSmoothContactActuator.h>

using namespace OpenSim;

//=========================================================================
// CONSTRUCTION
//=========================================================================

/** Default constructor. */
MocoZeroMomentPointConstraint::MocoZeroMomentPointConstraint() {
    constructProperties();
}

/** Constructor with object name and and contact body name */
MocoZeroMomentPointConstraint::MocoZeroMomentPointConstraint(
        const std::string& name, const std::string& zmpEstimatorPath) {
    constructProperties();
    setName(name);
    set_zmp_estimator_path(zmpEstimatorPath);
}

/** Property construction */
void MocoZeroMomentPointConstraint::constructProperties() {
    setName("zmp_constraint");
    constructProperty_zmp_estimator_path("");
    constructProperty_actuator_paths();
}

//=========================================================================
// MOCOPATHCONSTRAINT INTERFACE
//=========================================================================

/** Discovers component layouts and dynamically sizes constraint equations. */
void MocoZeroMomentPointConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {

	// Resolve the estimator path string and cache the estimator
    OPENSIM_THROW_IF_FRMOBJ(get_zmp_estimator_path().empty(), Exception,
        "zmp_estimator_path property was left blank. You must provide a valid component path.");

    // Look up the component in the model tree and cache its pointer location
    _estimator =
            &const_cast<Model&>(model).updComponent<ZeroMomentPointEstimator>(
                    get_zmp_estimator_path());

	// Clear stale states and dynamically gather all force actuators in the model tree
    _forceComponents.clear();
    m_controlIndices.clear();

    // Error check for body wrench actuator components
    int numForces = getProperty_actuator_paths().size();    
    OPENSIM_THROW_IF_FRMOBJ(numForces == 0, Exception,
        "actuator_paths list property is empty. You must specify at least one target path.");

    // Get the system control map for force inputs
    auto controlIndexMap = createSystemControlIndexMap(model);

    // Trace profiles and map index locations into matrix structure
    // At the same time get the body wrench actuator components
    for (int i = 0; i < numForces; ++i) {

        // Actuator components
        const std::string& forcePath = get_actuator_paths(i);
        const BodyWrenchSmoothContactActuator& actuator =
                model.getComponent<BodyWrenchSmoothContactActuator>(forcePath);
        _forceComponents.push_back(&actuator);

        // Control indices
        std::string path = actuator.getAbsolutePathString();
        ForceControlIndices indices;
        // Forces
        indices.fx_idx = controlIndexMap.at(path + "_0");
        indices.fy_idx = controlIndexMap.at(path + "_1");
        indices.fz_idx = controlIndexMap.at(path + "_2");
        // COPs
        indices.cop_x_idx = controlIndexMap.at(path + "_3");
        indices.cop_y_idx = controlIndexMap.at(path + "_4");
        indices.cop_z_idx = controlIndexMap.at(path + "_5");
        // Torques
        indices.tx_idx = controlIndexMap.at(path + "_6");
        indices.ty_idx = controlIndexMap.at(path + "_7");
        indices.tz_idx = controlIndexMap.at(path + "_8");
        // Push into structure
        m_controlIndices.push_back(indices);

        // Matching contact side names
        std::string matchedContactName;
        std::string targetBody = actuator.get_contact_body_name();
        // Look for target body in estimator
        for (int i = 0; i < _estimator->getNumContactSides(); ++i) {
            const auto& side = _estimator->getContactSide(i);
            std::string sideBodyName = side.get_contact_body_name();
            if (sideBodyName == targetBody) {
                matchedContactName = side.getName();
                break;
            }
        }
        // Throw error if unable to find the matching contact side
        OPENSIM_THROW_IF_FRMOBJ(matchedContactName.empty(), Exception,
                "Could not find a ContactSide matching contact_body_name '" +
                        targetBody + "' inside the connected estimator.");
        // Push into cache variable for matching side name
        m_matchedSideNames.push_back(matchedContactName);

    }

    // Size constraint equations. There are three constrain equations that are derived
    // from Xiang et al. that enforce consistency around the force and torque components:
    // 
    // Mx + My * COPz = 0
    // Mz - Fy * COPx = 0
    // My - Mfree - COPz * Fx + COPx * Fz = 0
    // 
    // where Mx,My,Mz is the moment in the ground frame; Fx, Fy, Fz is the force applied;
    // COP is the point location; and Mfree is the y-component of the applied torque. 
    //
    // The 3 equations are applied to each of the body wrenches listed. It is IMPORTANT TO
    // NOTE that the current implementation considers these separately, and hence will 
    // really only work when one force is active (i.e. unilateral ground contact). 
    //unsigned int totalEquations = static_cast<unsigned int>(_forceComponents.size() * 3);
    
    // TODO: the unilateral aspect above isn't true anymor with new formulation...

    // TODO: does basic 3 equation formulation work
    unsigned int totalEquations = 3;
    
    // Safety bypass to modify internal structural size during OpenSim lifecycle initialization
    const_cast<MocoZeroMomentPointConstraint*>(this)->setNumEquations(totalEquations);
	
}

/** Computes residual error bounds between Moco controls and physics outputs.*/
void MocoZeroMomentPointConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& s, SimTK::Vector& errors) const {

    // Bring the system up to the Acceleration stage to read forces and
    // kinematics
    getModel().realizeAcceleration(s);

    // Fetch the optimization controls at this state mesh point
    const auto& inputControls = getInputControls(s);

    // Set a small tuning parameter
    const double tune_fmag = 1e-3;

    // Initialise the variables for subsequent calculations
    SimTK::Vec3 net_f(0.0);
    //double f_mag = 0.0;
    SimTK::Vec3 net_t(0.0);
    //SimTK::Vec3 weightedPointSum(0.0);
    //SimTK::Vec3 net_p(0.0);
    SimTK::Vec3 mo(0.0);
    SimTK::Vec3 weighted_cop(0.0);
    double moment_from_cop_xz = 0.0;
    //std::vector<SimTK::Vec3> forces;
    //std::vector<SimTK::Vec3> points;
    //std::vector<SimTK::Vec3> torques;

    // Loop through each actuator input to calculate force and point variables
    for (size_t f = 0; f < getProperty_actuator_paths().size(); ++f) {

        // Get the current actuator
        const auto& actuator =
                getModel().getComponent<BodyWrenchSmoothContactActuator>(
                    get_actuator_paths(f));

        // Get the control indices for the current actuator
        const auto& indices = m_controlIndices[f];

        // Extract the data based on system control input map
        SimTK::Vec3 inputForce(inputControls[indices.fx_idx],
                inputControls[indices.fy_idx],
                inputControls[indices.fz_idx]);
        SimTK::Vec3 inputCoP(inputControls[indices.cop_x_idx],
                inputControls[indices.cop_y_idx],
                inputControls[indices.cop_z_idx]);
        SimTK::Vec3 inputTorque(inputControls[indices.tx_idx],
                inputControls[indices.ty_idx],
                inputControls[indices.tz_idx]);

        //// Add to the net force and overall force magnitude
        //// Uses a smoothed norm formulation to avoid hardness in IPOPT solve
        //net_f += inputForce;
        //double smooth_fmag =
        //        std::sqrt(inputForce.normSqr() + tune_fmag * tune_fmag);
        //f_mag += smooth_fmag;

        //// Compute weighted point sum for force
        //weightedPointSum += inputCoP * smooth_fmag;

        //// Store values in vectors for later calculations
        //forces.push_back(inputForce);
        //points.push_back(inputCoP);
        //torques.push_back(inputTorque);

        // Add to total net force and net torque
        net_f += inputForce;
        net_t += inputTorque;

        // Add to moment contribution about ground origin
        mo += SimTK::cross(inputCoP, inputForce);

        // Accumulate force-weighted COP directly
        weighted_cop[0] += inputCoP[0] * inputForce[1]; // x * Fy
        weighted_cop[2] += inputCoP[2] * inputForce[1]; // z * Fy

        // Calculate vertical moment contribution from individual CoPs: 
        // (cop_x * Fz - cop_z * Fx)
        moment_from_cop_xz += (inputCoP[0] * inputForce[2]) - (inputCoP[2] * inputForce[0]);

    }

    //// Compute weighted point location
    //// Add a small factor to smooth out and avoid potential zero division errors
    //net_p = weightedPointSum / (f_mag + tune_fmag);

    //// After establishing the net point location, compute net torque about this point
    //// The moment about the ground origin is also calculated here iteratively
    //for (size_t f = 0; f < getProperty_actuator_paths().size(); ++f) {
    //    // Calculate position vector from net_p to current point
    //    SimTK::Vec3 r = points[f] - net_p;
    //    // Calculate torque about this point
    //    SimTK::Vec3 t = SimTK::cross(r, forces[f]);
    //    // Add to net torque
    //    net_t += t;
    //    // Calculate position vector from ground origin to current point
    //    SimTK::Vec3 ro = points[f] - SimTK::Vec3(0.0);
    //    // Compute moment about ground origin
    //    SimTK::Vec3 m = SimTK::cross(ro, forces[f]);
    //    // Add to total ground moment
    //    mo += m;
    //}

    //// Equation constraints here must equal zero to satisfy
    //// Hence errors are simply the values of these equations
    //errors[0] = mo[0] + (net_f[1] * net_p[2]);
    //errors[1] = mo[2] - (net_f[1] * net_p[0]);
    //errors[2] = mo[1] - net_t[1] -
    //    (net_p[2] * net_f[0]) +
    //    (net_p[0] * net_f[2]);
    // Polynomial direct formulation
    errors[0] = (mo[0] + weighted_cop[2]); // TODO: possibly scale these given they are raw values?
    errors[1] = (mo[2] - weighted_cop[0]); // TODO: possibly scale these given they are raw values?
    errors[2] = ((mo[1] - net_t[1] - moment_from_cop_xz) * net_f[1]);

}
