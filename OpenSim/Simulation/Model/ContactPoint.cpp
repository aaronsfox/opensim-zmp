/* -------------------------------------------------------------------------- *
 *                         OpenSim: ContactPoint.cpp                          *
 * -------------------------------------------------------------------------- */

#include "ContactPoint.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

/**
 * A contact point is used to specify points on a body that can be queried for
 * contact against another frame dependent on the use case. The contact points
 * sit within a ContactPointSet which is a ModelComponentSet.
 *
 * @authors Aaron Fox
 * @version 1.0
 */

//============================================================================//
// CONSTRUCTION
//============================================================================//

/** Default constructor */
ContactPoint::ContactPoint() : Station() {
    constructProperties();
}

/**
 * Convenience constructor specifying name, body name and location.
 */
ContactPoint::ContactPoint(const std::string& pointName,
	const PhysicalFrame& parentFrame, const SimTK::Vec3& location)
        : Station() {
    constructProperties();
    setName(pointName);
    set_location(location);
	// Use the absolute path string directly instead of connectSocket
    // which requires the component to already be in the tree
    updSocket<PhysicalFrame>("parent_frame")
            .setConnecteePath(parentFrame.getAbsolutePathString());


}

/** Construct default properties */
void ContactPoint::constructProperties() {
    setName("contact_point");
    constructProperty_distance_threshold(0.08);
    constructProperty_velocity_threshold(1.5);
    constructProperty_contact_checking_method("distance");
}

//============================================================================//
// STATION INTERFACE
//============================================================================//

void ContactPoint::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

	// Check for positive distance threshold
    OPENSIM_THROW_IF_FRMOBJ(
        get_distance_threshold() < 0.0, Exception,
        "distance_threshold must be non-negative (got " +
        std::to_string(get_distance_threshold()) + ").");
	
	// Check for positive velocity threshold
	OPENSIM_THROW_IF_FRMOBJ(
        get_velocity_threshold() < 0.0, Exception,
        "velocity_threshold must be non-negative (got " +
        std::to_string(get_velocity_threshold()) + ").");
		
	// Check for appropriate contact checking method
    OPENSIM_THROW_IF_FRMOBJ(
		get_contact_checking_method() != "distance" &&
		get_contact_checking_method() != "velocity", Exception,
		"contact_checking_method must be 'distance' or 'velocity' (got '" +
		get_contact_checking_method() + "').");

}

void ContactPoint::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    
}

//============================================================================//
// OUTPUT EVALUATION METHODS
//============================================================================//

/**
 * This function takes the state and checks if the contact points allocated
 * to a body are in contact with the ground based on a distance and velocity
 * threshold. If any points meet the criteria then the body is said to be in
 * contact with the ground. However, if useVelocity is set to FALSE then only
 * the distance threshold is considered (i.e. velocity ignored). 
 * 
 * This approach reflects that outlined in Karcnik (2003):
 * https://doi.org/10.1007/BF02345310
 */
bool ContactPoint::isInContact(const SimTK::State& s) const {

	// Define in contact variable in function scope
	// Defaults to false
    bool inContact = false;
	
	// Check if point is in contact based on proposed checking method
    if (getContactCheckingMethod() == "velocity") {
		
		// Calculate if point is in contact using the distance and velocity method
		// Start by getting the distance and velocity threshold
		double distanceThreshold = get_distance_threshold();
        double velocityThreshold = get_velocity_threshold();
		
		std::cout << "TODO: distance and velocity contact checking method not done yet...sorry..."
		          << std::endl;
				  
	} else {
		
		// Calculate if point is in contact using the distance method
		// Start by getting the distance threshold
		double distanceThreshold = get_distance_threshold();
		
		// Check if the point specified on the body is below the distance threshold
		// relative to the ground plane, which is assumed at y = 0
        if (getLocationInGround(s)[1] < distanceThreshold) {
				
				// Specify that the point is in contact with the ground
				inContact = true;
				
			} else {
				
				// Specify that the point is not in contact
				inContact = false;
				
			}
		
	}
	
	// Return the in contact output
	return inContact;
	
}

//=============================================================================
//  METHODS
//=============================================================================

/** Set the distance threshold. */
void ContactPoint::setDistanceThreshold(
        const double& distanceThreshold) {
    set_distance_threshold(distanceThreshold);
}

/** Set the velocity threshold. */
void ContactPoint::setVelocityThreshold(
        const double& velocityThreshold) {
    set_velocity_threshold(velocityThreshold);
}

/** Set the contact checking method. */
void ContactPoint::setContactCheckingMethod(
    const std::string& contactCheckingMethod) {
    set_contact_checking_method(contactCheckingMethod);
}

/** Convenience method to get the 'parent_frame' Socket's connectee_name */
const std::string& ContactPoint::getParentFrameName() const { 
    return getSocket<PhysicalFrame>("parent_frame").getConnecteePath(); }