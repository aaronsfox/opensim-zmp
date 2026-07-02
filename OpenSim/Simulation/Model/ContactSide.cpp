/* -------------------------------------------------------------------------- *
 *                         OpenSim: ContactSide.cpp                           *
 * -------------------------------------------------------------------------- */

#include "ContactSide.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//============================================================================//
// CONSTRUCTION
//============================================================================//

/** Default constructor */
ContactSide::ContactSide() {
    constructProperties();
}

/**
 * Convenience constructor specifying the name and contact body.
 * @param contactBodyName     Name of the body to which the GRF will be applied
 */
ContactSide::ContactSide(const std::string&   name,
                         const std::string& contactBodyName)
{
    constructProperties();
    setName(name);
    set_contact_body_name(contactBodyName);

}

/** Construct default properties */
void ContactSide::constructProperties() {
    constructProperty_contact_points();
	setName("contact_side");
    constructProperty_contact_body_name("");
}

//============================================================================//
// CONTACT POINT ACCESS
//============================================================================//

/** Add a contact point for reviewing ground contact with default
distance and velocity parameters */
int ContactSide::addContactPoint(ContactPoint* point) {
    OPENSIM_THROW_IF_FRMOBJ(point == nullptr, Exception,
            "ContactPoint pointer must not be null.");
    // Append contact point
    const int idx = append_contact_points(*point);
    return idx;
}

/** Returns the number of contact points in the set. */
int ContactSide::getNumContactPoints() const {
    return getProperty_contact_points().size();
}

/** Returns a const reference to the i-th ContactPoint. */
const ContactPoint& ContactSide::getContactPoint(int i) const {
    return get_contact_points(i);
}

/** Returns a const reference to the ContactPoint with the given name. */
const ContactPoint& ContactSide::getContactPoint(
        const std::string& pointName) const {
    for (int i = 0; i < getNumContactPoints(); ++i) {
        if (get_contact_points(i).getName() == pointName) return get_contact_points(i);
    }
    OPENSIM_THROW_FRMOBJ(Exception, "No ContactPoint with name '" + pointName +
                                            "' found in ContactSide '" +
                                            getName() + "'.");
}

//============================================================================//
// MODELCOMPONENT INTERFACE
//============================================================================//

void ContactSide::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    OPENSIM_THROW_IF_FRMOBJ(getNumContactPoints() == 0, Exception,
        "ContactSide '" + getName() + "' has no ContactPoints. "
        "At least one ContactPoint is required.");

}

void ContactSide::extendConnectToModel(Model& model) {
    
    Super::extendConnectToModel(model);

    // Check if the contact body is actually listed in the model
    OPENSIM_THROW_IF_FRMOBJ(
            !model.getBodySet().contains(get_contact_body_name()), Exception,
            "ContactSide '" + getName() + "': contact_body_name '" +
                    get_contact_body_name() +
                    "' was not found in the model's BodySet. "
                    "Check that the body name is spelled correctly and exists "
                    "in the model.");    

}

//============================================================================//
// OUTPUT EVALUATION
//============================================================================//

// Checks if any of the contact points are in contact
// Early exit of the loop will occur by returning true if any are in contact
bool ContactSide::isInContact(const SimTK::State& s) const {
    for (int i = 0; i < getNumContactPoints(); ++i)
        if (get_contact_points(i).isInContact(s))
            return true;
    return false;
}