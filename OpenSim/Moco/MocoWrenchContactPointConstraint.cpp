/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoWrenchContactPointConstraint.cpp                         *
 * -------------------------------------------------------------------------- */

#include "MocoWrenchContactPointConstraint.h"
#include "MocoProblemInfo.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Actuators/BodyWrenchSmoothContactActuator.h>

#include <cmath>
#include <algorithm>

// TODO: the polygon aspect of this is probably unnecessary, given this is 
// simply a centroid point distance constraint now...

// TODO: this should really be labelled something like a foot boundary COP constraint...

using namespace OpenSim;

//=========================================================================
// CONSTRUCTION
//=========================================================================

/** Default constructor. */
MocoWrenchContactPointConstraint::MocoWrenchContactPointConstraint() {
    constructProperties();
}

/** Constructor with object name and body wrench actuator path listed. */
MocoWrenchContactPointConstraint::MocoWrenchContactPointConstraint(
        const std::string& name, const std::string& actuatorPath) {
    constructProperties();
    setName(name);
    set_actuator_path(actuatorPath);
}

/** Property construction */
void MocoWrenchContactPointConstraint::constructProperties() {
    setName("point_constraint");
    constructProperty_actuator_path("");
    constructProperty_distance_tolerance(SimTK::Vec3(0.05, 0.01, 0.05));
    constructProperty_big_m(0.2);
}

//=========================================================================
// MOCOPATHCONSTRAINT INTERFACE
//=========================================================================

/** Discovers component layouts and dynamically sizes constraint equations. */
void MocoWrenchContactPointConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {

    // Check for empty body actuator path
	OPENSIM_THROW_IF_FRMOBJ(get_actuator_path().empty(), Exception,
            "actuator_path property was left blank. You must "
            "provide a valid component path.");

    // Check for positive values in distance tolerances
    OPENSIM_THROW_IF_FRMOBJ(get_distance_tolerance().get(0) < 0.0, Exception,
            "Negative value for distance_tolerance in X-axis. Distance tolerance "
            "must be positive.");
    OPENSIM_THROW_IF_FRMOBJ(get_distance_tolerance().get(1) < 0.0, Exception,
            "Negative value for distance_tolerance in Y-axis. Distance tolerance "
            "must be positive.");
    OPENSIM_THROW_IF_FRMOBJ(get_distance_tolerance().get(2) < 0.0, Exception,
            "Negative value for distance_tolerance in Z-axis. Distance tolerance "
            "must be positive.");

    // Look up the actuator component in the model tree and cache its pointer
    _actuator = &const_cast<Model&>(model).updComponent<BodyWrenchSmoothContactActuator>(
            get_actuator_path());

    // Two one-sided inequalities per axis (upper and lower bound),
    // 3 axes => 6 scalar equations, each of the form error <= 0
    unsigned int totalEquations = static_cast<unsigned int>(6);
    const_cast<MocoWrenchContactPointConstraint*>(this)->setNumEquations(
            totalEquations);

    // Set constraint bounds
    // Equations are written in the form of constraint being satisfied
    // if error is less than or equal to zero, hence these are bounded 
    // that way
    MocoConstraintInfo info = getConstraintInfo();
    std::vector<MocoBounds> bounds(6, MocoBounds(-SimTK::Infinity, 0.0));
    const_cast<MocoWrenchContactPointConstraint*>(this)
            ->updConstraintInfo()
            .setBounds(bounds);
	
}

/** Computes residual error bounds between Moco controls and physics outputs.*/
void MocoWrenchContactPointConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& s, SimTK::Vector& errors) const {

    // Realize to acceleration stage
    // TODO: not really sure if needed...
    getModel().realizeAcceleration(s);

    // Pull the actuator's ground-frame outputs
    const SimTK::Vec3 pGround =
            _actuator->getOutputValue<SimTK::Vec3>(s, "applied_point");
    const SimTK::Vec3 centroidGround =
            _actuator->getOutputValue<SimTK::Vec3>(s, "contact_centroid");
    const double C =
            _actuator->getOutputValue<double>(s, "contact_indicator");

    // Convert locations into body frame so that distance tolerances are
    // relevant to body axes and size
    const auto& bodyFrame = _actuator->getBodyFrame();
    const SimTK::Vec3 pBody =
            getModel().getGround().findStationLocationInAnotherFrame(
                    s, pGround, bodyFrame);
    const SimTK::Vec3 centroidBody =
            getModel().getGround().findStationLocationInAnotherFrame(
                    s, centroidGround, bodyFrame);

    // Calculate difference between point and centroid
    const SimTK::Vec3 diff = pBody - centroidBody;

    // Get max difference and parameters to calculate relaxation term
    const SimTK::Vec3& maxDist = get_distance_tolerance();
    const double bigM = get_big_m();
    const double relax = bigM * (1.0 - C);

    // TODO: don't know if the non big-M formulation works that well?

    //// Calculate upper and lower bound constraint errors for each axis
    //for (int i = 0; i < 3; ++i) {
    //    // Upper bound: diff[i]  <= max_distance[i] (relaxed when C -> 0)
    //    errors[2 * i] = diff[i] - maxDist[i] - relax;
    //    // Lower bound: -diff[i] <= max_distance[i] (relaxed when C -> 0)
    //    errors[2 * i + 1] = -diff[i] - maxDist[i] - relax;
    //}
    // Smoothed formulation for relaxation with no need for big-M
    for (int i = 0; i < 3; ++i) {
        // When C -> 0 (flight phase), the error smoothly scales to 0 <= 0,
        // naturally turning off the constraint without large derivative spikes.
        // Upper bound: diff[i]  <= max_distance[i] (relaxed when C -> 0)
        errors[2 * i] = C * (diff[i] - maxDist[i]);
        // Lower bound: -diff[i] <= max_distance[i] (relaxed when C -> 0)
        errors[2 * i + 1] = C * (-diff[i] - maxDist[i]);
    }

}