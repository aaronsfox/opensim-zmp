/* -------------------------------------------------------------------------- *
 *               OpenSim: BodyWrenchSmoothContactActuator.cpp                 *
 * -------------------------------------------------------------------------- */

#include "BodyWrenchSmoothContactActuator.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

using namespace OpenSim;

//============================================================================//
// CONSTRUCTION
//============================================================================//

/** Default constructor */
BodyWrenchSmoothContactActuator::BodyWrenchSmoothContactActuator() {
    constructProperties();
}

/** Constructor with object name and and contact body name */
BodyWrenchSmoothContactActuator::BodyWrenchSmoothContactActuator(
        const std::string& name,
        const std::string& contactBodyName) {
    constructProperties();
    setName(name);
    set_contact_body_name(contactBodyName);    
}

/** Constructor with object name, contact body name, smoothing and distance
threshold values.*/
BodyWrenchSmoothContactActuator::BodyWrenchSmoothContactActuator(
        const std::string& name,
        const std::string& contactBodyName, 
        const double& smoothing, 
        const SimTK::Vec3& distanceOffset) {
    constructProperties();
    setName(name);
    set_contact_body_name(contactBodyName);
    set_contact_smoothing(smoothing);
    set_max_point_offset(distanceOffset);
}

/** Property construction */
void BodyWrenchSmoothContactActuator::constructProperties() {
    setName("body_wrench_actuator");
    constructProperty_contact_body_name("");
    constructProperty_contact_smoothing(0.01);
    constructProperty_max_point_offset(SimTK::Vec3(0.05, 0.01, 0.05));
    constructProperty_contact_point_paths();
}

//============================================================================//
// MODEL COMPONENT INTERFACE
//============================================================================//

// Connect to model
void BodyWrenchSmoothContactActuator::extendConnectToModel(Model& model) {
 
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

    // Look up and cache the contact points
    _contactPoints.clear();
    const int numPaths = getProperty_contact_point_paths().size();
    // Check if any paths are provided
    OPENSIM_THROW_IF_FRMOBJ(
            numPaths == 0, Exception, 
        "No contact_point_paths were specified.");
    // Loop through paths to cache pointers
    for (int i = 0; i < numPaths; ++i) {
        const std::string& path = get_contact_point_paths(i);
        try {
            const auto& cp = model.getComponent<ContactPoint>(path);
            _contactPoints.push_back(&cp);
        } catch (const std::exception&) {
            OPENSIM_THROW_FRMOBJ(
                    Exception, "Could not find a ContactPoint at path '" +
                                       path + "' inside the model.");
        }
    }

}

//=========================================================================
// OUTPUTS
//=========================================================================

// Get the location of the contact centroid in the ground frame
SimTK::Vec3 BodyWrenchSmoothContactActuator::getContactCentroidInGround(
        const SimTK::State& s) const {

    // Get the number of contact points and smoothing parameter
    const int nPoints = getProperty_contact_point_paths().size();
    const double eps = get_contact_smoothing();

    // Calculated weighted centroid location based on contact probability
    double sumWeight = 0.0;
    SimTK::Vec3 weightedSum(0.0);
    for (int i = 0; i < nPoints; ++i) {
        const ContactPoint& cp = *_contactPoints.at(i);
        const double h = cp.getLocationInGround(s)[1];
        const double h0 = cp.getDistanceThreshold();
        const double s_i = 1.0 / (1.0 + std::exp((h - h0) / eps));
sumWeight += s_i;
        weightedSum += s_i * cp.getLocationInGround(s);
    }

    return (nPoints > 0) ? weightedSum / (sumWeight + 1e-6) : SimTK::Vec3(0.0);
}

// Get the applied point in the ground frame
// TODO: is this necessary if the signal is simply the point in ground?
SimTK::Vec3 BodyWrenchSmoothContactActuator::getAppliedPointInGround(
        const SimTK::State& s) const {
    SimTK::Vec3 force, torque, pointInBody;
    double C;
    calcAppliedWrench(s, force, torque, pointInBody, C);
    return _bodyFrame->findStationLocationInAnotherFrame(
            s, pointInBody, getModel().getGround());
}

// Get the contact indicator probability value
double BodyWrenchSmoothContactActuator::getContactIndicator(
    const SimTK::State& s) const {
    SimTK::Vec3 force, torque, pointInBody;
    double C;
    calcAppliedWrench(s, force, torque, pointInBody, C);
    return C;
}

//=========================================================================
// FORCE API INTERFACE
//=========================================================================

/** Calculates the applied wrench forces based on smooth contact conditions. */
void BodyWrenchSmoothContactActuator::calcAppliedWrench(
    const SimTK::State& s,
    SimTK::Vec3& force, SimTK::Vec3& torque, SimTK::Vec3& pointInBody,
    double& C) const {

    // Get the raw control signals from the actuator
    const SimTK::Vector controls = getControls(s);
    SimTK::Vec3 rawForce(controls[0], controls[1], controls[2]);
    SimTK::Vec3 rawPoint(controls[3], controls[4], controls[5]);
    SimTK::Vec3 rawTorque(controls[6], controls[7], controls[8]);

    // Get the number of contact points in the list
    const int nPoints = getProperty_contact_point_paths().size();

    // Get contact smoothing parameter
    const double eps = get_contact_smoothing();

    // Set the starting values for probability of contact
    double prodOneMinusS = 1.0;
    // Loop through contact points and determine contact
    // Here instead of determine yes/no for contact the probability
    // of contact is smoothed based on exponential smoothing.
    for (int i = 0; i < nPoints; ++i) {
        // Get the contact point and its properties
        const ContactPoint& cp = *_contactPoints.at(i);
        const double h = cp.getLocationInGround(s)[1];
        const double h0 = cp.getDistanceThreshold();
        const double s_i = 1.0 / (1.0 + std::exp((h - h0) / eps));
        // Append current points properties to contact probabilites
        prodOneMinusS *= (1.0 - s_i);
    }
    // Compute total probability of contact and weighted centroid
    C = (nPoints > 0) ? (1.0 - prodOneMinusS) : 0.0;

    // Scale force and torque by the raw control signals and the contact
    // probability
    force = C * rawForce;
    torque = C * rawTorque;

    // Calculate point in body based o
    pointInBody = getModel().getGround().findStationLocationInAnotherFrame(
            s, rawPoint, *_bodyFrame);
}

/** Implements the `ForceProducer` API by computing ZMP forces. */
void BodyWrenchSmoothContactActuator::implProduceForces(
        const SimTK::State& s, ForceConsumer& forceConsumer) const {

    // Check if the component has fully connected frames before reading inputs
    if (!_bodyFrame) return;

    // Get the values to apply
    SimTK::Vec3 force, torque, pointInBody;
    double C;
    calcAppliedWrench(s, force, torque, pointInBody, C);

    // Dispatch load values to the framework consumer mechanism
    forceConsumer.consumePointForce(s, *_bodyFrame, pointInBody, force);
    forceConsumer.consumeTorque(s, *_bodyFrame, torque);

}

/** Compute power consumed by moving the body via applied spatial force. */
double BodyWrenchSmoothContactActuator::getPower(const SimTK::State& s) const {
    
    // Get the control signals for in ground COP
    const SimTK::Vector controls = getControls(s);
    //SimTK::Vec3 force(controls[0], controls[1], controls[2]);
    SimTK::Vec3 cop(controls[3], controls[4], controls[5]);
    //SimTK::Vec3 torque(controls[6], controls[7], controls[8]);

    // Get the force and torque values to apply
    SimTK::Vec3 force, torque, pointInBody;
    double C;
    calcAppliedWrench(s, force, torque, pointInBody, C);

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