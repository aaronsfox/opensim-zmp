#ifndef OPENSIM_CONTACT_SIDE_H
#define OPENSIM_CONTACT_SIDE_H
/* -------------------------------------------------------------------------- *
 *                          OpenSim: ContactSide.h                            *
 * -------------------------------------------------------------------------- *
 * ContactSide groups one or more ContactPoints belonging to a single limb    *
 * side (e.g. "right" or "left"). ContactPoints are owned in a                *
 * ContactPointSet property, mirroring the PathPoint/PathPointSet pattern.    *
 *                                                                            *
 * A side is considered IN CONTACT when ANY of its ContactPoints reports      *
 * contact.                                                                   *
 * -------------------------------------------------------------------------- */

//#include <OpenSim/Simulation/Model/ContactPointSet.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//============================================================================//
//                              ContactSide                                   //
//============================================================================//
/**
 * A named limb side that owns a ContactPointSet for ground contact detection
 * and identifies the body to which the ZMP-derived GRF is applied.
 *
 * ### Properties
 * - `contact_point_set`: ContactPointSet
 *     The set of contact detection points owned by this side.
 *
 * ### Sockets
 * - `contact_body` : PhysicalFrame
 *     Body to which the ZMP-derived GRF for this side is applied
 *     (e.g. /bodyset/calcn_r).
 *
 * ### Outputs
 * - `is_in_contact`       : bool   
 *     True if ANY ContactPoint in the set is in contact.
 *
 * @author  Aaron Fox
 */
class OSIMSIMULATION_API ContactSide : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ContactSide, ModelComponent);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_PROPERTY(contact_body_name, std::string,
            "The name of the contact body that the contact side relates to "
            " in the model (e.g. calcn_r, calcn_l) (defaults to blank "
            " which will throw an error given it won't exist in model).")

    OpenSim_DECLARE_LIST_PROPERTY(contact_points, ContactPoint,
        "Set of contact detection points belonging to this side.");

    //=========================================================================
    // SOCKETS
    //=========================================================================
	
    //OpenSim_DECLARE_SOCKET(contact_body, PhysicalFrame,
    //    "PhysicalFrame to which the ZMP-derived ground reaction force for "
    //    "this side is applied (e.g. /bodyset/calcn_r).");

    //=========================================================================
    // OUTPUTS
    //=========================================================================
    OpenSim_DECLARE_OUTPUT(is_in_contact, bool,
        isInContact, SimTK::Stage::Acceleration);

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
	/** Default constructor */
	ContactSide();

    /**
     * Convenience constructor specifying the name and contact body.
     * @param contactBodyName     Name of the body to which the GRF will be applied
     */
    ContactSide(const std::string&   name,
                const std::string& contactBodyName);

    virtual ~ContactSide() = default;

    //=========================================================================
    // CONTACT POINT ACCESS
    //=========================================================================

    ///** Returns the ContactPointSet owned by this side. */
    //const ContactPointSet& getContactPointSet() const;

    ///** Returns a writable reference to the ContactPointSet. */
    //ContactPointSet& updContactPointSet();

    /** Add a contact point for reviewing ground contact */
    int addContactPoint(ContactPoint* point);

    /** Returns the number of contact points body. */
    int getNumContactPoints() const;

    /** Returns a const reference to the i-th ContactPoint. */
    const ContactPoint& getContactPoint(int i) const;

    /** Returns a const reference to the ContactPoint with the given name. */
    const ContactPoint& getContactPoint(const std::string& pointName) const;

    //=========================================================================
    // OUTPUT EVALUATION
    //=========================================================================

    /** Returns true if at least one ContactPoint is in contact. */
    bool isInContact(const SimTK::State& s) const;

protected:
    //=========================================================================
    // MODELCOMPONENT INTERFACE
    //=========================================================================
    void extendFinalizeFromProperties()  override;
    void extendConnectToModel(Model& model) override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_SIDE_H
