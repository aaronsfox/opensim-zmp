#ifndef OPENSIM_BODY_WRENCH_ACTUATOR_H
#define OPENSIM_BODY_WRENCH_ACTUATOR_H

/* -------------------------------------------------------------------------- *
 *                    OpenSim: BodyWrenchActuator.h                           *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim {

class Model;

//============================================================================//
//                          BodyWrenchActuator                                //
//============================================================================//
/**
 * The body wrench actuator is a force subclass very similar to the 
 * BodyActuator class, with the exception that it takes three additional 
 * control signals that represent the point of force/torque application in 
 * the ground frame. The actuator applies the force and torque to the 
 * designated body based on the control signals given by force, torque and 
 * the point of application.
 * 
 * @author Aaron Fox
 */
 
class OSIMACTUATORS_API BodyWrenchActuator : public Actuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(BodyWrenchActuator, Actuator);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(contact_body_name, std::string,
        "Name of the body this force applies to (e.g. 'calcn_r'). ");

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
    /** Default constructor */
    BodyWrenchActuator();

    /** Constructor with object name and and contact body name */
    BodyWrenchActuator(const std::string& name,
            const std::string& contactBodyName);

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Set & get contact name methods. */
    void setContactBodyName(const std::string& bodyName);
    const std::string& getContactBodyName() const;

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

    // Implements the `ForceProducer` API by applying body wrench forces
    void implProduceForces(const SimTK::State&, ForceConsumer&) const override;

    // Implement the actuator interface
    // Number of controls is force x 3, COP x 3, torque x 3
    int numControls() const override { return 9; }
	
    // Get actuator power
    double getPower(const SimTK::State& s) const override;

    // Cache the resolved physical body frame pointer
    mutable SimTK::ResetOnCopy<SimTK::ReferencePtr<const PhysicalFrame>>
            _bodyFrame{nullptr};

};

} // namespace OpenSim

#endif // OPENSIM_BODY_WRENCH_ACTUATOR_H