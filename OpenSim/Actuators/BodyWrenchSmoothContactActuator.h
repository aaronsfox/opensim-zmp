#ifndef OPENSIM_BODY_WRENCH_SMOOTH_CONTACT_ACTUATOR_H
#define OPENSIM_BODY_WRENCH_SMOOTH_CONTACT_ACTUATOR_H

/* -------------------------------------------------------------------------- *
 *                OpenSim: BodyWrenchSmoothContactActuator.h                  *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>

// TODO: probably rename to SmoothContactBodyWrenchActuator...fits OpenSim notation
// better...

namespace OpenSim {

class Model;

//============================================================================//
//                      BodyWrenchSmoothContactActuator                       //
//============================================================================//
/**
 * The body wrench actuator is a force subclass very similar to the 
 * BodyActuator class, with the exception that it takes three additional 
 * control signals that represent the point of force/torque application in 
 * the ground frame. The actuator applies the force and torque to the 
 * designated body based on the control signals given by force, torque and 
 * the point of application. This is the smoothed contact version of the body
 * wrench actuator, which takes some extra inputs to work in the context of
 * Moco (i.e. direct collocation optimisation). 
 * 
 * TODO: add implementation notes...particularly around what the point value
 * means now...it's now an offset from the weighted contact point centroid...
 * Info above therefore needs to change...
 * 
 * @author Aaron Fox
 */
 
class OSIMACTUATORS_API BodyWrenchSmoothContactActuator : public Actuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(BodyWrenchSmoothContactActuator, Actuator);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(contact_body_name, std::string,
        "Name of the body this force applies to (e.g. 'calcn_r'). ");

    OpenSim_DECLARE_PROPERTY(contact_smoothing, double,
            "Width of the smooth contact-indicator sigmoid (m). Default 0.01.");

    OpenSim_DECLARE_PROPERTY(max_point_offset, SimTK::Vec3,
            "Max distance (m) the application point may be offset from the "
            "weighted contact-point centroid in the body frame XYZ axis "
            "directions. Defaults to (0.05, 0.01, 0.05).");

    OpenSim_DECLARE_LIST_PROPERTY(contact_point_paths, std::string,
            "List of absolute string paths to the contact points in the model "
            "to query for contact.");

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
    /** Default constructor */
    BodyWrenchSmoothContactActuator();

    /** Constructor with object name and contact body name */
    BodyWrenchSmoothContactActuator(
            const std::string& name,
            const std::string& contactBodyName);

    /** Constructor with object name, contact body name, smoothing and distance
    offset values.*/
    BodyWrenchSmoothContactActuator(
            const std::string& name, 
            const std::string& contactBodyName,
            const double& smoothing,
            const SimTK::Vec3& distanceOffset);

    //=========================================================================
    // OUTPUTS
    //=========================================================================

    OpenSim_DECLARE_OUTPUT(contact_indicator, double, getContactIndicator,
            SimTK::Stage::Position);

    OpenSim_DECLARE_OUTPUT(applied_point, SimTK::Vec3, getAppliedPointInGround,
            SimTK::Stage::Position);

    OpenSim_DECLARE_OUTPUT(contact_centroid, SimTK::Vec3,
            getContactCentroidInGround, SimTK::Stage::Position);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Set & get contact name methods. */
    void setContactBodyName(const std::string& bodyName) {
        set_contact_body_name(bodyName);
    };
    const std::string& getContactBodyName() const {
        return get_contact_body_name();
    };

    /** Set & get contact smoothing. */
    void setContactSmoothing(const double& smoothing) {
        set_contact_smoothing(smoothing);
    };
    double getContactSmoothing() const { 
        return get_contact_smoothing(); };

    /** Set & get max point offset. */
    void setMaxPointOffset(const SimTK::Vec3& offset) {
        set_max_point_offset(offset);
    };
    SimTK::Vec3 getMaxPointOffset() const { 
        return get_max_point_offset(); };

    /** Append a contact point path. */
    void addContactPoint(const std::string& contactPointPath) {
        append_contact_point_paths(contactPointPath);
    };

    /** Get the resolved PhysicalFrame this actuator applies its wrench to. */
    const PhysicalFrame& getBodyFrame() const {
        OPENSIM_THROW_IF_FRMOBJ(!_bodyFrame, Exception,
                "Body frame has not been resolved yet -- has the model been "
                "connected?");
        return *_bodyFrame;
    }

protected:

    //=========================================================================
    // MODEL COMPONENT INTERFACE
    //=========================================================================

    /** Connect to the OpenSim Model. */
    void extendConnectToModel(Model& model) override;

private:

    //=========================================================================
    // PRIVATE MEMBERS
    //=========================================================================

    // Property construction
    void constructProperties();

    // Implements the `ForceProducer` API and applies forces
    void implProduceForces(const SimTK::State&, ForceConsumer&) const override;

    // Implement the actuator interface
    // Number of controls is force x 3, COP x 3, torque x 3
    int numControls() const override { return 9; }
	
    // Get actuator power
    double getPower(const SimTK::State& s) const override;

    // Shared calculation of the smooth/gated force/torque and bounded
    // application point, used by both implProduceForces and getPower
    void calcAppliedWrench(const SimTK::State& s, SimTK::Vec3& force,
            SimTK::Vec3& torque, SimTK::Vec3& pointInBody,
            double& contactIndicator) const;

    // Output evaluation methods
    SimTK::Vec3 getContactCentroidInGround(const SimTK::State& s) const;
    double getContactIndicator(const SimTK::State& s) const;
    SimTK::Vec3 getAppliedPointInGround(const SimTK::State& s) const;
    
    // Cache the resolved physical body frame pointer
    mutable SimTK::ResetOnCopy<SimTK::ReferencePtr<const PhysicalFrame>>
            _bodyFrame{nullptr};

    // Cached pointers resolved from contact_point_paths in extendConnectToModel
    mutable SimTK::ResetOnCopy<std::vector<const ContactPoint*>> _contactPoints;

};

} // namespace OpenSim

#endif // OPENSIM_BODY_WRENCH_SMOOTH_CONTACT_ACTUATOR_H