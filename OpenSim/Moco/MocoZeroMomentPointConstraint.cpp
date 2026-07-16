/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoZeroMomentPointConstraint.cpp                            *
 * -------------------------------------------------------------------------- */

#include "MocoZeroMomentPointConstraint.h"
#include "MocoProblemInfo.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Actuators/BodyWrenchActuator.h>

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
    constructProperty_body_wrench_paths();
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
    int numForces = getProperty_body_wrench_paths().size();    
    OPENSIM_THROW_IF_FRMOBJ(numForces == 0, Exception,
        "body_wrench_paths list property is empty. You must specify at least one target path.");

    // Get the system control map for force inputs
    auto controlIndexMap = createSystemControlIndexMap(model);

    // Trace profiles and map index locations into matrix structure
    // At the same time get the body wrench actuator components
    for (int i = 0; i < numForces; ++i) {

        // ZMP force components
        const std::string& forcePath = get_body_wrench_paths(i);
        const auto& bodyWrench =
                model.getComponent<BodyWrenchActuator>(forcePath);
        _forceComponents.push_back(&bodyWrench);

        // Control indices
        std::string path = bodyWrench.getAbsolutePathString();
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
        std::string targetBody = bodyWrench.get_contact_body_name();
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
    unsigned int totalEquations = static_cast<unsigned int>(_forceComponents.size() * 3);
    
    // Safety bypass to modify internal structural size during OpenSim lifecycle initialization
    const_cast<MocoZeroMomentPointConstraint*>(this)->setNumEquations(totalEquations);
	
}

/** Computes residual error bounds between Moco controls and physics outputs. */
void MocoZeroMomentPointConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& s, SimTK::Vector& errors) const {

    // Bring the system up to the Acceleration stage to read forces and kinematics
    getModel().realizeAcceleration(s);

    // Fetch the optimization controls at this state mesh point
    const auto& inputControls = getInputControls(s);
    int equationIdx = 0;

    // Process each contact sides actuator inputs
    for (size_t f = 0; f < _forceComponents.size(); ++f) {

        // Get the cached data for this ZMP force component
        const auto* forceComp = _forceComponents[f];
        const auto& indices = m_controlIndices[f];

        // Read which frame this specific actuator acts upon
        const std::string& targetBody = forceComp->get_contact_body_name();

        // Query the estimator to see which internal results channel targets this body
        // This uses the matched contact side name cached earlier for the force
        int estimatorSideIdx = _estimator->getSideIndex(m_matchedSideNames[f]);

        // Extract the direct data based on system control input map
        SimTK::Vec3 inputForce(
            inputControls[indices.fx_idx],
            inputControls[indices.fy_idx],
            inputControls[indices.fz_idx]);
        SimTK::Vec3 inputCoP(
            inputControls[indices.cop_x_idx],
            inputControls[indices.cop_y_idx],
            inputControls[indices.cop_z_idx]);
        SimTK::Vec3 inputTorque(
            inputControls[indices.tx_idx],
            inputControls[indices.ty_idx],
            inputControls[indices.tz_idx]);

        // Calculated ground moment
        SimTK::Vec3 calcGroundMoment =
                SimTK::Vec3((inputCoP[2] * -1) * inputForce[1],
                        inputTorque[1] + (inputCoP[2] * inputForce[0]) -
                                (inputCoP[0] * inputForce[2]),
                        inputCoP[0] * inputForce[1]);

        // Equation constraints here must equal zero to satisfy
        // Hence errors are simply the values of these equations
        errors[equationIdx++] =
                calcGroundMoment[0] + (inputForce[1] * inputCoP[2]);
        errors[equationIdx++] =
                calcGroundMoment[2] - (inputForce[1] * inputCoP[0]);
        errors[equationIdx++] =
                calcGroundMoment[1] - inputTorque[1] - (inputCoP[2] * inputForce[0]) + 
            (inputCoP[0] * inputForce[2]);

    }

}
