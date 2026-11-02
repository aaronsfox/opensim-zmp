/* -------------------------------------------------------------------------- *
 *                    OpenSim: BodyWrenchActuator.cpp                         *
 * -------------------------------------------------------------------------- */

#include "BodyWrenchActuator.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

using namespace OpenSim;

//============================================================================//
// CONSTRUCTION
//============================================================================//

/** Default constructor */
BodyWrenchActuator::BodyWrenchActuator() { constructProperties(); }

/** Constructor with object name and and contact body name */
BodyWrenchActuator::BodyWrenchActuator(const std::string& name,
        const std::string& contactBodyName) {
    constructProperties();
    setName(name);
    set_contact_body_name(contactBodyName);
}

/** Property construction */
void BodyWrenchActuator::constructProperties() {
    setName("body_wrench_actuator");
    constructProperty_contact_body_name("");
}

//=========================================================================
// METHODS
//=========================================================================

/** Set & get contact name methods. */
void BodyWrenchActuator::setContactBodyName(
    const std::string& bodyName) {
    set_contact_body_name(bodyName);
}
const std::string& BodyWrenchActuator::getContactBodyName() const { 
    return get_contact_body_name();
}

//============================================================================//
// MODEL COMPONENT INTERFACE
//============================================================================//

// Connect to model
void BodyWrenchActuator::extendConnectToModel(Model& model) {
 
    // Base class first
    Super::extendConnectToModel(model); 

    // Check contact body name
    const std::string& bodyName = get_contact_body_name();
    OPENSIM_THROW_IF_FRMOBJ(bodyName.empty(), Exception,
            "contact_body_name property is empty.");

    // Look up and cache the PhysicalFrame from the model using
    // the body name string
    try {
        const auto& frame =
                getModel().getComponent<PhysicalFrame>(get_contact_body_name());
        _bodyFrame = &frame;
    } catch (const std::exception&) {
        // Fallback: If it's a relative path name, search through the
        // component repo
        const auto& frame =
                getModel().getComponent<PhysicalFrame>(
                    "/bodyset/" + get_contact_body_name());
        _bodyFrame = &frame;
    }
    // If body can't be foundthrow an error
    OPENSIM_THROW_IF_FRMOBJ(!_bodyFrame, Exception,
            "Could not find a PhysicalFrame named '" + get_contact_body_name() +
                    "' inside the model.");

}

//=========================================================================
// FORCE API INTERFACE
//=========================================================================

/** Implements the `ForceProducer` API by computing ZMP forces. */
void BodyWrenchActuator::implProduceForces(
        const SimTK::State& s, ForceConsumer& forceConsumer) const {

    // Check if the component has fully connected frames before reading inputs
    if (!_bodyFrame) return;

    // Get the control signals
    const SimTK::Vector controlValues = getControls(s);

    // Map the controls to spatial vectors
    SimTK::Vec3 force(controlValues[0], controlValues[1], controlValues[2]);
    SimTK::Vec3 cop(controlValues[3], controlValues[4], controlValues[5]);
    SimTK::Vec3 torque(controlValues[6], controlValues[7], controlValues[8]);

    // Current use cases will have COP in ground frame
    // Convert COP from ground to body frame to apply point force
    SimTK::Vec3 copInBody =
             getModel().getGround().findStationLocationInAnotherFrame(
                     s, cop, *_bodyFrame);

    // Dispatch load values to the framework consumer mechanism
    forceConsumer.consumePointForce(s, *_bodyFrame, copInBody, force);
    forceConsumer.consumeTorque(s, *_bodyFrame, torque);

}

/** Compute power consumed by moving the body via applied spatial force. */
double BodyWrenchActuator::getPower(const SimTK::State& s) const {
    
    // Get the control signals
    const SimTK::Vector controls = getControls(s);
    SimTK::Vec3 force(controls[0], controls[1], controls[2]);
    SimTK::Vec3 cop(controls[3], controls[4], controls[5]);
    SimTK::Vec3 torque(controls[6], controls[7], controls[8]);

    // Fetch the body's rigid body velocities out of the Simbody mobilized
    const SimTK::MobilizedBody& mobBody = _bodyFrame->getMobilizedBody();

    // Angular velocity of the body frame in Ground
    SimTK::Vec3 angularVelocity = mobBody.getBodyAngularVelocity(s);

    // Linear velocity of the exact moving point (CoP station) tracked on that
    // body in Ground Note: If your CoP input is already expressed in Ground,
    // you would use mobBody.findStationVelocityInGround(s, copInBody) Assuming
    // your CoP is expressed relative to the target body frame:
    SimTK::Vec3 copVelocity = mobBody.findStationVelocityInGround(s, cop);

    // Compute Power: (Force dot PointVelocity) + (Torque dot
    // AngularVelocity) The ~ operator performs a vector transpose (dot product)
    // in SimTK
    double power = (~force * copVelocity) + (~torque * angularVelocity);

    return power;
}