/* -------------------------------------------------------------------------- *
 *                   OpenSim: ZeroMomentPointContactForce.cpp                 *
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

#include "ZeroMomentPointContactForce.h"
#include "ZeroMomentPointGroundReactions.h"

#include <OpenSim/Simulation/Model/Model.h>

// TODO: the contact half space force may be too complex for this force
// Consider a simpler example that takes recorded values and applies?

using namespace OpenSim;
//=============================================================================
//=============================================================================
/**
 * TODO: add instructional content for ZeroMomentPointContactForce
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ZeroMomentPointContactForce::ZeroMomentPointContactForce() {
    
    // Construct default properties
    setNull();
    constructProperties();

}

/** Constructor with object name and ZMP Ground Reactions socket */
ZeroMomentPointContactForce::ZeroMomentPointContactForce(const std::string& name,
        const ZeroMomentPointGroundReactions& zmpGroundReactions) {

    // Construct default properties
    setNull();
    constructProperties();

    // Set name of force object
    setName(name);

    // Connect to the ZMP ground reactions socket
    connectSocket_zmp_ground_reactions(zmpGroundReactions);

}

/** Constructor with object name, ZMP Ground Reactions socket and contact
body name */
ZeroMomentPointContactForce::ZeroMomentPointContactForce(
        const std::string& name,
        const ZeroMomentPointGroundReactions& zmpGroundReactions,
        const std::string zeroMomentPointContactBodyName) {

    // Construct default properties
    setNull();
    constructProperties();

    // Set name of force object
    setName(name);

    // Set the name of the contact body to connect with
    // This should be the same as in the ZeroMomentPointContactBody object
    set_zero_moment_point_contact_body_name(zeroMomentPointContactBodyName);

    // Connect to the ZMP ground reactions socket
    connectSocket_zmp_ground_reactions(zmpGroundReactions);
    
}

/** Constructor with object name, ZMP Ground Reactions socket, contact
body name and applied to body name */
ZeroMomentPointContactForce::ZeroMomentPointContactForce(
        const std::string& name,
        const ZeroMomentPointGroundReactions& zmpGroundReactions,
        const std::string zeroMomentPointContactBodyName,
        const std::string appliedToBodyName) {

    // Construct default properties
    setNull();
    constructProperties();

    // Set name of force object
    setName(name);

    // Set the name of the contact body to connect with
    // This should be the same as in the ZeroMomentPointContactBody object
    set_zero_moment_point_contact_body_name(zeroMomentPointContactBodyName);

    // Set the name of the body to apply the force to
    set_applied_to_body(appliedToBodyName);

    // Connect to the ZMP ground reactions socket
    connectSocket_zmp_ground_reactions(zmpGroundReactions);

}

/** Set the data members of this force to their null values. */
void ZeroMomentPointContactForce::setNull() 
{ 
    setAuthors("Aaron Fox");
    _appliedToBody = nullptr;
}

/** Construct properties and initialize to their default values */
void ZeroMomentPointContactForce::constructProperties()
{
    // Set default properties to unassigned
    constructProperty_zero_moment_point_contact_body_name("unassigned");
    constructProperty_applied_to_body("unassigned");

    /*constructProperty_force_visualization_radius(0.01);
    constructProperty_force_visualization_scale_factor();*/

}

//=============================================================================
// Connect this force element to the rest of the model.
//=============================================================================

/** Create a SimTK::Force which implements this Force. */
void ZeroMomentPointContactForce::extendConnectToModel(Model& model)
{

    // TODO: anything need to be done to connect to ZMP ground reactions socket?
    // It doesn't seem like anything extra is needed...

    // Let base class connect first
    Super::extendConnectToModel(model);

    // Look up the body being specified as the applied to body
    // TODO: consider using sockets?
    const std::string& appliedToBodyName = getAppliedToBody();

    // Hook up pointers from names
    _appliedToBody.reset();

    // Frame for applying body forces
    if (hasModel()) {
        if (getModel().hasComponent<PhysicalFrame>(appliedToBodyName))
            _appliedToBody =
                    &_model->getComponent<PhysicalFrame>(appliedToBodyName);
        else if (getModel().hasComponent<PhysicalFrame>(
                         "./bodyset/" + appliedToBodyName))
            _appliedToBody = &getModel().getComponent<PhysicalFrame>(
                    "./bodyset/" + appliedToBodyName);
    }

    // Check for finding body frame
    if (!_appliedToBody) {
        throw(Exception("ZeroMomentPointContactForce: Could not find body '" +
                        appliedToBodyName + "' to apply force to."));
    }

}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================

void ZeroMomentPointContactForce::extendAddToSystem(
        SimTK::MultibodySystem& system) const {

    // Base class first
    Super::extendAddToSystem(system);

    //// Create cached variable for force magnitude
    //this->_forceMagnitudeCV =
    //        addCacheVariable("force_magnitude", 0.0, SimTK::Stage::Dynamics);

    // Beyond the const Component get access to underlying SimTK elements
    ZeroMomentPointContactForce* mutableThis =
            const_cast<ZeroMomentPointContactForce*>(this);

    //// Get underlying bodies
    //mutableThis->_contactBody = _contactBody->getMobilizedBody();

}

//void OpenSim::ZeroMomentPointContactForce::extendRealizeInstance(
//        const SimTK::State& state) const {
//
//    Super::extendRealizeInstance(state);
//    if (!getProperty_force_visualization_scale_factor().empty()) {
//        m_forceVizScaleFactor = get_force_visualization_scale_factor();
//    } else {
//        const Model& model = getModel();
//        const double mass = model.getTotalMass(state);
//        const double weight = mass * model.getGravity().norm();
//        m_forceVizScaleFactor = 1 / weight;
//    }
//}

//=============================================================================
// COMPUTING
//=============================================================================

/** Compute the ZMP ground reactions and apply it to the model */
void ZeroMomentPointContactForce::computeForce(const SimTK::State& state,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& generalizedForces) const
{

    // Ensure applied to body is OK
    OPENSIM_ASSERT_FRMOBJ(_appliedToBody != nullptr);

    // TODO: use calcGroundReactions to get ZMP estimates
    //ZeroMomentPointGroundReactions::calcGroundReactions(state);

    // Get the force, point and torque data from ZMP ground reactions
    // TODO: currently just setting as zeros
    SimTK::Vec3 force = SimTK::Vec3(0.0, 0.0, 0.0);
    SimTK::Vec3 point = SimTK::Vec3(0.0, 0.0, 0.0);
    SimTK::Vec3 torque = SimTK::Vec3(0.0, 0.0, 0.0);

    // Apply the force
    // TODO: frame of reference for point?
    applyForceToPoint(state, *_appliedToBody, point, force, bodyForces);

    // Apply the torque
    applyTorque(state, *_appliedToBody, torque, bodyForces);

}

/** Get the ZMP ground reactions for the component */
SimTK::Vector ZeroMomentPointContactForce::getGroundReactions(
    const SimTK::State& state, const SimTK::Vector& udot) const {

    std::cout << "Calculate ground reactions";
    SimTK::Vector groundReactions(9);

    // TODO: ground reactions should be calculated here using the 
    // contact forces component and then the reactions for the 
    // specific force component extracted:
        // > Get the socket connectee for the force
        // > Use this to calculate the ground reactions with state and udot
        // > Identify the appropriate ground reactions for the particular force

    // Then another function can be created to get the separate force, point and torque values...

    return groundReactions;

}

// Get the force magnitude that has already been computed
//const double& ZeroMomentPointContactForce::getForceMagnitude(
//        const SimTK::State& s) {
//    return getCacheVariableValue(s, _forceMagnitudeCV);
//}

//=============================================================================
// REPORTING
//=============================================================================

// Provide names of the quantities (column labels) of the force value(s)
Array<std::string> ZeroMomentPointContactForce::getRecordLabels() const {

    OpenSim::Array<std::string> labels("");

    labels.append(getName() + ".ZMP" + ".force.X");
    labels.append(getName() + ".ZMP" + ".force.Y");
    labels.append(getName() + ".ZMP" + ".force.Z");
    labels.append(getName() + ".ZMP" + ".point.X");
    labels.append(getName() + ".ZMP" + ".point.Y");
    labels.append(getName() + ".ZMP" + ".point.Z");
    labels.append(getName() + ".ZMP" + ".torque.X");
    labels.append(getName() + ".ZMP" + ".torque.Y");
    labels.append(getName() + ".ZMP" + ".torque.Z");

    return labels;
}

// Provide the value(s) to be reported that correspond to the labels
Array<double> ZeroMomentPointContactForce::getRecordValues(
        const SimTK::State& state) const {

    // TODO: currently just defaulting to a bunch of 1's until calculations...

    Array<double> values(1.0, 9);

    //const auto& sphere = getConnectee<ContactSphere>("sphere");
    //const auto sphereIdx = sphere.getFrame().getMobilizedBodyIndex();

    //SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    //calcBodyForces(state, bodyForces);

    //// On contact body
    //const auto& thisBodyForce1 = bodyForces(sphereIdx);
    //SimTK::Vec3 forces1 = thisBodyForce1[1];
    //SimTK::Vec3 torques1 = thisBodyForce1[0];
    //values.append(3, &forces1[0]);
    //values.append(3, &torques1[0]);

    return values;

}

//SimTK::Vector ZeroMomentPointContactForce::getContactBodyForce(
//        const SimTK::State& state) const {
//
//    SimTK::Vector contactBodyForces(9);
//
//    /*const auto& sphere = getConnectee<ContactSphere>("sphere");
//    const auto sphereIdx = sphere.getFrame().getMobilizedBodyIndex();
//
//    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
//    calcBodyForces(state, bodyForces);*/
//
//    return contactBodyForces;
//
//}

//void ZeroMomentPointContactForce::calcBodyForces(
//        const SimTK::State& state,
//        SimTK::Vector& bodyForces) const {
//
//    /*const Model& model = getModel();
//    const auto& forceSubsys = model.getForceSubsystem();
//    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
//    const auto& simtkForce =
//            static_cast<const SimTK::SmoothSphereHalfSpaceForce&>(
//                    abstractForce);
//
//    SimTK::Vector_<SimTK::Vec3> particleForces(0);
//    SimTK::Vector mobilityForces(0);
//
//    simtkForce.calcForceContribution(
//            state, bodyForces, particleForces, mobilityForces);*/
//
//}

//void ZeroMomentPointContactForce::generateDecorations(bool fixed,
//        const ModelDisplayHints& hints, const SimTK::State& state,
//        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
//
//    //Super::generateDecorations(fixed, hints, state, geometry);
//
//    //if (!fixed && (state.getSystemStage() >= SimTK::Stage::Dynamics) &&
//    //        hints.get_show_forces()) {
//    //    // Compute the body forces.
//    //    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
//    //    calcBodyForces(state, bodyForces);
//
//    //    // Get the index to the associated contact sphere.
//    //    const auto& sphere = getConnectee<ContactSphere>("sphere");
//    //    const auto& sphereIdx = sphere.getFrame().getMobilizedBodyIndex();
//
//    //    // Get the translational force for the contact sphere associated with
//    //    // this force element.
//    //    const auto& sphereForce = bodyForces(sphereIdx)[1];
//
//    //    // Scale the contact force vector and compute the cylinder length.
//    //    const auto& scaledContactForce =
//    //            m_forceVizScaleFactor * sphereForce;
//    //    const SimTK::Real length(scaledContactForce.norm());
//
//    //    // Compute the force visualization transform.
//    //    const SimTK::Vec3 contactSpherePosition =
//    //            sphere.getFrame().findStationLocationInGround(
//    //                    state, sphere.get_location());
//    //    const SimTK::Transform forceVizTransform(
//    //            SimTK::Rotation(SimTK::UnitVec3(scaledContactForce),
//    //                    SimTK::YAxis),
//    //            contactSpherePosition + scaledContactForce / 2.0);
//
//    //    // Construct the force decoration and add it to the list of geometries.
//    //    SimTK::DecorativeCylinder forceViz(get_force_visualization_radius(),
//    //            0.5 * length);
//    //    forceViz.setTransform(forceVizTransform);
//    //    forceViz.setColor(SimTK::Vec3(0.0, 0.6, 0.0));
//    //    geometry.push_back(forceViz);
//    //}
//
//}
