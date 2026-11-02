/* -------------------------------------------------------------------------- *
 *               OpenSim:  Fluit2014ContactPointActuator.cpp                  *
 * -------------------------------------------------------------------------- */

#include "Fluit2014ContactPointActuator.h"

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

#include <algorithm>
#include <cmath>

using namespace OpenSim;

//=============================================================================
// LOCAL HELPERS
//=============================================================================

namespace {

// Logistic ("sigmoid") function centered at ratio = 1, with steepness k:
//   f(1)   = 0.5
//   f(0)   -> 1 as k increases
//   f(inf) -> 0
// As k -> infinity, this approaches a step function that is 1 for
// ratio < 1 and 0 for ratio > 1, recovering the hard threshold used by
// Fluit et al. (2014, Eq. 1) and Skals et al. (2017, Eq. 2).
double smoothStepDown(double ratio, double k) {
    return 1.0 / (1.0 + std::exp(k * (ratio - 1.0)));
}

} // namespace

//=============================================================================
// CONSTRUCTORS
//=============================================================================

/** Default constructor */
Fluit2014ContactPointActuator::Fluit2014ContactPointActuator() {
    setNull();
    constructProperties();
}

/** Construct with the name and body name of the Body this actuator's contact
point is attached to.**/
Fluit2014ContactPointActuator::Fluit2014ContactPointActuator(
    const std::string& name,
    const std::string& bodyName) {
    
    setNull();
    constructProperties();
    setName(name);
    set_body_name(bodyName);

}

/** Property construction */
void Fluit2014ContactPointActuator::setNull() {
    setAuthors("Aaron Fox");
}
void Fluit2014ContactPointActuator::constructProperties() {
    constructProperty_body_name("");
    constructProperty_point(SimTK::Vec3(0));
    constructProperty_point_is_global(false);
    constructProperty_max_force(1.0);
    constructProperty_ground_height(0.0);
    constructProperty_contact_height_threshold(0.03);
    constructProperty_contact_velocity_threshold(0.8);
    constructProperty_smoothing(8.0);
    constructProperty_use_velocity_in_scaling(true);
    constructProperty_friction_coefficient(0.5);
    constructProperty_min_control(0.0);
    constructProperty_max_control(1.0);
}

//=========================================================================
// MODEL COMPONENT INTERFACE
//=========================================================================

// Finalize from properties
void Fluit2014ContactPointActuator::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    OPENSIM_THROW_IF_FRMOBJ(get_friction_coefficient() < 0, Exception,
            "friction_coefficient must be non-negative.");

    OPENSIM_THROW_IF_FRMOBJ(get_max_control() < get_min_control(), Exception,
            "max_control must not be less than min_control.");
}

// Add to system and cache variables
void Fluit2014ContactPointActuator::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    
    // Base class first
    Super::extendAddToSystem(system);

    // Cache the computed speed and height of the contact point.
    this->_speedCV = addCacheVariable("speed", 0.0, SimTK::Stage::Velocity);
    this->_heightCV = addCacheVariable("height", 0.0, SimTK::Stage::Position);

    // Cache the contact force variable
    this->_forceCV = addCacheVariable(
            "contact_force", SimTK::Vec3(0), SimTK::Stage::Dynamics);

}

// Connect to model
void Fluit2014ContactPointActuator::extendConnectToModel(Model& model) {
    
    // Base class first
    Super::extendConnectToModel(model);

    // Check contact point body name
    const std::string& bodyName = get_body_name();
    OPENSIM_THROW_IF_FRMOBJ(bodyName.empty(), Exception,
            "body_name property is empty.");

    // Look up and cache the PhysicalFrame from the model using
    // the body name string
    try {
        const auto& frame =
                getModel().getComponent<PhysicalFrame>(get_body_name());
        _bodyFrame = &frame;
    } catch (const std::exception&) {
        // Fallback: If it's a relative path name, search through the
        // component repo
        const auto& frame = getModel().getComponent<PhysicalFrame>(
                "/bodyset/" + get_body_name());
        _bodyFrame = &frame;
    }
    // If body can't be foundthrow an error
    OPENSIM_THROW_IF_FRMOBJ(!_bodyFrame, Exception,
            "Fluit2014ContactPointActuator: Could not find a PhysicalFrame named '" 
        + get_body_name() + "' inside the model.");

}

//=============================================================================
// GET AND SET METHODS
//=============================================================================

/** Set and get the 'body_name' property. **/
void Fluit2014ContactPointActuator::setBodyName(
    const std::string& bodyName) {
    set_body_name(bodyName);
}
const std::string& Fluit2014ContactPointActuator::getBodyName() const {
    return get_body_name();
}

/** Set and get the 'max_normal_force' property. **/
void Fluit2014ContactPointActuator::setMaxForce(const double& maxForce) {
    set_max_force(maxForce);
}
const double& Fluit2014ContactPointActuator::getMaxForce() const {
    return get_max_force();
}

/** Set and get the 'ground_height' property. **/
void Fluit2014ContactPointActuator::setGroundHeight(const double& groundHeight) {
    set_ground_height(groundHeight);
}
const double& Fluit2014ContactPointActuator::getGroundHeight() const {
    return get_ground_height();
}

/** Set and get the 'contact_height_threshold' property. **/
void Fluit2014ContactPointActuator::setContactHeightThreshold(
        const double& height) {
    set_contact_height_threshold(height);
}
const double& Fluit2014ContactPointActuator::getContactHeightThreshold() const {
    return get_contact_height_threshold();
}

/** Set and get the 'contact_velocity_threshold' property. **/
void Fluit2014ContactPointActuator::setContactVelocityThreshold(const double& velocity) {
    set_contact_velocity_threshold(velocity);
}
const double& Fluit2014ContactPointActuator::getContactVelocityThreshold() const {
    return get_contact_velocity_threshold();
}

/** Set and get the 'smoothing' property. **/
void Fluit2014ContactPointActuator::setSmoothingCoefficient(
        const double& smoothing) {
    set_smoothing(smoothing);
}
const double& Fluit2014ContactPointActuator::getSmoothingCoefficient() const {
    return get_smoothing();
}

/** Set and get the 'friction_coefficient' property. **/
void Fluit2014ContactPointActuator::setFrictionCoefficient(const double& friction) {
    set_friction_coefficient(friction);
}
const double& Fluit2014ContactPointActuator::getFrictionCoefficient() const {
    return get_friction_coefficient();
}

/** Set and get the strength scaling mode property. **/
void Fluit2014ContactPointActuator::setUseVelocityInScaling(const bool& mode) {
    set_use_velocity_in_scaling(mode);
}
const bool& Fluit2014ContactPointActuator::getUseVelocityInScaling() const {
    return get_use_velocity_in_scaling();
}

//=============================================================================
// COMPUTATION
//=============================================================================

/** Contact point location, in the body frame (resolving point_is_global
if necessary). **/
SimTK::Vec3 Fluit2014ContactPointActuator::getPointInBodyFrame(
        const SimTK::State& s) const {    
    // Simply grab the point in the body frame if it is set to local
    if (!get_point_is_global()) { return get_point(); }
    // The point property is expressed in Ground; convert it into the body
    // frame so it can be used consistently by
    // findStationLocationInGround()/findStationVelocityInGround() below.
    return getModel().getGround().findStationLocationInAnotherFrame(
            s, get_point(), *_bodyFrame);
}

/** Location/velocity of the contact point, expressed in Ground. **/
SimTK::Vec3 Fluit2014ContactPointActuator::calcPointLocationInGround(
        const SimTK::State& s) const {
    return _bodyFrame->findStationLocationInGround(s, getPointInBodyFrame(s));
}
SimTK::Vec3 Fluit2014ContactPointActuator::calcPointVelocityInGround(
        const SimTK::State& s) const {
    return _bodyFrame->findStationVelocityInGround(s, getPointInBodyFrame(s));
}

/** Height of the contact point above the ground plane (i.e., its
ground +Y coordinate minus ground_height), in meters. **/
double Fluit2014ContactPointActuator::getContactPointHeight(
        const SimTK::State& s) const {
    const SimTK::Vec3 posInGround = calcPointLocationInGround(s);
    return posInGround[1] - get_ground_height();
}
double Fluit2014ContactPointActuator::getHeight(const SimTK::State& s) const {
    if (isCacheVariableValid(s, _heightCV)) {
        return getCacheVariableValue(s, _heightCV);
    }
    const double height = getContactPointHeight(s);
    updCacheVariableValue(s, _heightCV) = height;
    markCacheVariableValid(s, _heightCV);
    return height;
}

/** Gets the contact point velocity, in meters per second. **/
SimTK::Vec3 Fluit2014ContactPointActuator::getContactPointVelocity(
        const SimTK::State& s) const {
    const SimTK::Vec3 velInGround = calcPointVelocityInGround(s);
    return velInGround;
}

/** Gets the contact point location expressed in ground. **/
SimTK::Vec3 Fluit2014ContactPointActuator::getContactPointLocationInGround(
        const SimTK::State& s) const {
    return calcPointLocationInGround(s);
}

/** Speed (norm of the 3D velocity, in Ground) of the contact point. **/
double Fluit2014ContactPointActuator::calcSpeed(const SimTK::State& s) const {
    return calcPointVelocityInGround(s).norm();
}
double Fluit2014ContactPointActuator::getSpeed(const SimTK::State& s) const {
    if (isCacheVariableValid(s, _speedCV)) {
        return getCacheVariableValue(s, _speedCV);
    }
    const double speed = calcSpeed(s);
    updCacheVariableValue(s, _speedCV) = speed;
    markCacheVariableValid(s, _speedCV);
    return speed;
}

/** Value (in the open interval (0, 1)) of the smooth strength-scaling
factor evaluated from the current position and velocity of the contact
point (see class documentation). This is the fraction of
'max_force' this actuator would deliver if fully activated
(control = max) at the current state. **/
double Fluit2014ContactPointActuator::getStrengthScaleFactor(
        const SimTK::State& s) const {
    return calcStrengthScaleFactor(s);
}

/** Smooth strength-scaling factor, in (0, 1); see class documentation. **/
double Fluit2014ContactPointActuator::calcStrengthScaleFactor(
        const SimTK::State& s) const {
    const double k = get_smoothing();
    const double zCrit = std::max(get_contact_height_threshold(), 1e-8);

    // getContactPointHeight() already returns height *relative to*
    // ground_height, so no further offset is applied here.
    const double zRatio = getHeight(s) / zCrit;
    const double sZ = smoothStepDown(zRatio, k);

    // Calculate the velocity component if using
    if (get_use_velocity_in_scaling()) {

        // "distance_and_velocity" (default): matches Fluit et al. (2014, Eq. 1)
        // / Skals et al. (2017, Eq. 2) -- both height AND speed must be below
        // their respective thresholds to reach full strength.
        const double vCrit = std::max(get_contact_velocity_threshold(), 1e-8);
        const double vRatio = getSpeed(s) / vCrit;
        const double sV = smoothStepDown(vRatio, k);

        return sZ * sV;

    } else {

        // Height-only gating: appropriate for treadmill/relative-motion
        // trials, where a planted foot still carries substantial
        // ground-frame velocity (the belt speed) throughout stance, and a
        // velocity criterion would incorrectly suppress strength then.
        return sZ;
    
    }

}

/** Fixed unit direction (in Ground) of the given control index's line of
action; depends only on friction_coefficient. **/
SimTK::Vec3 Fluit2014ContactPointActuator::getActuatorDirection(
        int controlIndex) const {
    const double mu = get_friction_coefficient();
    const double norm = std::sqrt(1.0 + mu * mu);
    switch (controlIndex) {
    case Control_Normal: return SimTK::Vec3(0.0, 1.0, 0.0);
    case Control_ShearPosX: return SimTK::Vec3(mu, 1.0, 0.0) / norm;
    case Control_ShearNegX: return SimTK::Vec3(-mu, 1.0, 0.0) / norm;
    case Control_ShearPosZ: return SimTK::Vec3(0.0, 1.0, mu) / norm;
    case Control_ShearNegZ: return SimTK::Vec3(0.0, 1.0, -mu) / norm;
    default:
        OPENSIM_THROW_FRMOBJ(Exception,
                "Fluit2014ContactPointActuator: invalid control index.");
    }
}

/** Force currently produced by a single one of the five actuators. **/
SimTK::Vec3 Fluit2014ContactPointActuator::getActuatorForce(
        const SimTK::State& s, int controlIndex) const {
    const double q = SimTK::clamp(getControls(s)[controlIndex],
        get_min_control(), get_max_control());
    const double magnitude =
            q * get_max_force() * calcStrengthScaleFactor(s);
    return magnitude * getActuatorDirection(controlIndex);
}

/** Calculation for contact force over the five actuators. **/
SimTK::Vec3 Fluit2014ContactPointActuator::calcContactForce(
        const SimTK::State& s) const {
    const double scale = calcStrengthScaleFactor(s);
    const double fMax = get_max_force();
    const auto& controls = getControls(s);
    SimTK::Vec3 total(0.0);
    for (int i = 0; i < NumContactControls; ++i) {
        const double q = SimTK::clamp(
            get_min_control(), controls[i], get_max_control());
        total += (q * fMax * scale) * getActuatorDirection(i);
    }
    return total;
}

/** Total (normal + shear) force from all five actuators, expressed in
the Ground frame. Reads from cache or calculates. **/
SimTK::Vec3 Fluit2014ContactPointActuator::getContactForce(
        const SimTK::State& s) const {
    if (isCacheVariableValid(s, _forceCV)) {
        return getCacheVariableValue(s, _forceCV);
    }
    const SimTK::Vec3 force = calcContactForce(s);
    updCacheVariableValue(s, _forceCV) = force;
    markCacheVariableValid(s, _forceCV);
    return force;
}

/** Total (ground +Y, vertical) component of the summed force from all
five actuators. **/
double Fluit2014ContactPointActuator::getNormalForce(
        const SimTK::State& s) const {
    return getContactForce(s)[1];
}

/** Total (medio-lateral / antero-posterior, ground X-Z) component of the
summed force from all five actuators. **/
SimTK::Vec3 Fluit2014ContactPointActuator::getShearForce(
        const SimTK::State& s) const {
    const SimTK::Vec3 total = getContactForce(s);
    return SimTK::Vec3(total[0], 0.0, total[2]);
}

/** Total mechanical power delivered by this actuator (dot product of its
total force and the contact point's velocity in Ground). **/
double Fluit2014ContactPointActuator::getPower(const SimTK::State& s) const {
    const SimTK::Vec3 force = getContactForce(s);
    const SimTK::Vec3 vel = calcPointVelocityInGround(s);
    return force[0] * vel[0] + force[1] * vel[1] + force[2] * vel[2];
}

/**
 * Implements the `ForceProducer` interface by applying the combined force
 * of all five actuators to the specified body, at the specified point.
 */
void Fluit2014ContactPointActuator::implProduceForces(
        const SimTK::State& s, ForceConsumer& forceConsumer) const {
    // Check if the component has fully connected frames before reading
    // inputs.
    if (!_bodyFrame) return;

    const SimTK::Vec3 totalForce = getContactForce(s);

    // Apply the combined force to the body at the designated point.
    forceConsumer.consumePointForce(
            s, *_bodyFrame, getPointInBodyFrame(s), totalForce);
}

//=============================================================================
// REPORTING
//=============================================================================

/** Record labels and values.**/
OpenSim::Array<std::string>
Fluit2014ContactPointActuator::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName() + ".force.X");
    labels.append(getName() + ".force.Y");
    labels.append(getName() + ".force.Z");
    return labels;
}
OpenSim::Array<double> Fluit2014ContactPointActuator::getRecordValues(
        const SimTK::State& s) const {
    OpenSim::Array<double> values(3);
    if (!_model || !_bodyFrame) {
        values.append(0.0);
        values.append(0.0);
        values.append(0.0);
        return values;
    }
    const SimTK::Vec3 force = getContactForce(s);
    values.append(force[0]);
    values.append(force[1]);
    values.append(force[2]);
    return values;
}