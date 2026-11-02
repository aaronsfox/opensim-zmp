/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoWrenchPointProximityGoal.cpp                             *
 * -------------------------------------------------------------------------- */

#include "MocoWrenchPointProximityGoal.h"

#include <OpenSim/Simulation/Model/Model.h>


// TODO: add comments

using namespace OpenSim;

//=========================================================================
// CONSTRUCTION
//=========================================================================

MocoWrenchPointProximityGoal::MocoWrenchPointProximityGoal() {
    constructProperties();
}

MocoWrenchPointProximityGoal::MocoWrenchPointProximityGoal(
        const std::string& name) : MocoGoal(name) {
    constructProperties();
}

MocoWrenchPointProximityGoal::MocoWrenchPointProximityGoal(
        const std::string& name, double weight) : MocoGoal(name, weight) {
    constructProperties();
}

void MocoWrenchPointProximityGoal::constructProperties() {
    constructProperty_actuator_path("");
    constructProperty_axis_weights(SimTK::Vec3(1.0, 1.0, 1.0));
}

//=========================================================================
// MOCOGOAL INTERFACE
//=========================================================================

void MocoWrenchPointProximityGoal::initializeOnModelImpl(
        const Model& model) const {

    OPENSIM_THROW_IF_FRMOBJ(get_actuator_path().empty(), Exception,
        "actuator_path property was left blank. You must provide a valid "
        "component path.");

    _actuator = &model.getComponent<BodyWrenchSmoothContactActuator>(
            get_actuator_path());

    setRequirements(1, 1, SimTK::Stage::Dynamics);
}

void MocoWrenchPointProximityGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {

    //Realize state
    getModel().realizeDynamics(input.state);

    const SimTK::Vec3 pGround =
            _actuator->getOutputValue<SimTK::Vec3>(input.state, "applied_point");
    const SimTK::Vec3 centroidGround = _actuator->getOutputValue<SimTK::Vec3>(
            input.state, "contact_centroid");
    const double C =
            _actuator->getOutputValue<double>(input.state, "contact_indicator");

    const auto& bodyFrame = _actuator->getBodyFrame();
    const SimTK::Vec3 pBody =
            getModel().getGround().findStationLocationInAnotherFrame(
                    input.state, pGround, bodyFrame);
    const SimTK::Vec3 centroidBody =
            getModel().getGround().findStationLocationInAnotherFrame(
                    input.state, centroidGround, bodyFrame);

    const SimTK::Vec3 diff = pBody - centroidBody;
    const SimTK::Vec3& w = get_axis_weights();

    double weightedSqDist = 0.0;
    for (int i = 0; i < 3; ++i) {
        weightedSqDist += w[i] * diff[i] * diff[i];
    }

    integrand = C * weightedSqDist;
	
}

void MocoWrenchPointProximityGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
}