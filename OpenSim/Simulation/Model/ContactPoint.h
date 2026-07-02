#ifndef OPENSIM_CONTACT_POINT_H
#define OPENSIM_CONTACT_POINT_H
/* -------------------------------------------------------------------------- *
 *                          OpenSim: ContactPoint.h                           *
 * -------------------------------------------------------------------------- *
 * ContactPoint is a ModelComponent representing a single named point fixed   *
 * to a PhysicalFrame (e.g. a body segment) that is used to detect contact    *
 * with the ground plane. It exposes its Ground-frame position and a boolean  *
 * is_in_contact Output evaluated by comparing the point's vertical height    *
 * against a user-defined threshold.                                          *
 *                                                                            *
 * This component is intended to be owned as a subcomponent of ContactSide    *
 * and sit within this in the ContactPoints list property.                    *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//============================================================================//
//                             ContactPoint                                   //
//============================================================================//
/**
 * A named point fixed to a body that detects ground contact.
 *
 * The point position is specified in the parent body's local frame via the
 * `location` property, which are both inherited from the base Station class.
 * Note that the distance method simply refers to height in the y-axis for
 * contact checking. 
 *
 * ### Properties
 * - `distance_threshold`      : double — maximum distance (m) at which contact
 *                               is considered to be active (default 0.08 m) when using
 *                               distance as a method
 * - `velocity_threshold`      : double — velocity (m/s) threshold for determining contact
 *                               (default 1.5 m/s) when using velocity as a method
 * - `contact_checking_method` : string — contact checking method to use, being 
 *                               `distance` (only distance to frame is checked) or 
 *                               `velocity` (distance and velocity are checked)
 *
 *
 * ### Outputs
 * - `is_in_contact`      : bool   indicator if contact point is deemed in ground contact.
 *
 * @author  Aaron Fox
 */
class OSIMSIMULATION_API ContactPoint : public Station {
    OpenSim_DECLARE_CONCRETE_OBJECT(ContactPoint, Station);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(distance_threshold, double,
        "Distance (m) from which the point is considered in contact "
        "with another frame. Default is 0.08 m.");
	
	OpenSim_DECLARE_PROPERTY(velocity_threshold, double,
        "Vertical height (m) below which the point is considered in contact "
        "with the ground plane. Default is 1.5 m/s.");
		
	OpenSim_DECLARE_PROPERTY(contact_checking_method, std::string,
            "The ground contact checking method applied to this contact point. "
            "Options here are strings of distance (only distance to ground is "
            "checked) and velocity (distance and velocity are checked) (defaults "
            "to distance).");

    //=========================================================================
    // OUTPUTS
    //=========================================================================

    OpenSim_DECLARE_OUTPUT(is_in_contact, bool,
        isInContact, SimTK::Stage::Acceleration);

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
	/** Default constructor */
    ContactPoint();

    /**
     * Convenience constructor specifying name, parent frame and location.
     *
     * @param name            Point name (e.g. "heel_r")
     * @param parentFrame     The parent PhysicalFrame the point is attached to
     * @param location        Position in the body frame (m)
     */
    ContactPoint(const std::string& pointName,
                 const PhysicalFrame& parentFrame,
                 const SimTK::Vec3& location);

    //=========================================================================
    // OUTPUT EVALUATION METHODS
    //=========================================================================

    /** Returns true the point meets the contact specifications. */
    bool isInContact(const SimTK::State& s) const;
	
	//=========================================================================
    // METHODS
    //=========================================================================

    /** Get and set the distance threshold. */
    const double& getDistanceThreshold() const { 
        return get_distance_threshold(); }
    void setDistanceThreshold(const double& distanceThreshold);

    /** Get and set the velocity threshold. */
    const double& getVelocityThreshold() const { 
        return get_velocity_threshold(); }
    void setVelocityThreshold(const double& velocityThreshold);

    /** Get and set the contact checking method. */
    const std::string& getContactCheckingMethod() const { 
        return get_contact_checking_method(); }
    void setContactCheckingMethod(const std::string& contactCheckingMethod);

    /** Convenience method to get the 'parent_frame' Socket's connectee_name */
    const std::string& getParentFrameName() const;	
	
protected:
    //=========================================================================
    // MODELCOMPONENT INTERFACE
    //=========================================================================
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_POINT_H
