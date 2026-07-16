/* -------------------------------------------------------------------------- *
 *                OpenSim: MocoContactPointConstraint.cpp                     *
 * -------------------------------------------------------------------------- */

// TODO: this is just a basic construction...not sure if this actually functions...

#include "MocoContactPointConstraint.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//============================================================================//
// CONSTRUCTION
//============================================================================//

MocoContactPointConstraint::MocoContactPointConstraint() {
    constructProperties();
}

void MocoContactPointConstraint::constructProperties() {
    constructProperty_contact_point_paths();
    constructProperty_ground_plane_height(0.0);
}

//============================================================================//
// INITIALIZATION
//============================================================================//

void MocoContactPointConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problem) const {

     // Validate properties
    OPENSIM_THROW_IF_FRMOBJ(getProperty_contact_point_paths().size() == 0,
            Exception, "contact_point_paths must contain at least one path.");

    // Resolve each ContactPoint path
    _resolvedPoints.clear();
    const int nPaths = getProperty_contact_point_paths().size();

    for (int i = 0; i < nPaths; ++i) {
        const std::string& path = get_contact_point_paths(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<ContactPoint>(path),
                Exception, "No ContactPoint found at path '" + path + "'.");

        ResolvedPoint rp;
        rp.absolutePath = path;
        rp.displayName = path.substr(path.find_last_of('/') + 1);
        _resolvedPoints.push_back(rp);
    }

    const int nPoints = static_cast<int>(_resolvedPoints.size());

    // This is the correct API for setting bounds on a MocoPathConstraint.
    // setNumEquations tells Moco how many constraint equations this
    // constraint contributes. The bounds on each equation are then set
    // via updConstraintInfo().
    setNumEquations(nPoints);

    // Set bounds for each equation: height - ground_plane_height >= 0
    // i.e. lower = 0, upper = +infinity
    std::vector<MocoBounds> bounds(nPoints, MocoBounds(0.0, SimTK::Infinity));
    const_cast<MocoContactPointConstraint*>(this)
            ->updConstraintInfo()
            .setBounds(bounds);

}

//============================================================================//
// CONSTRAINT EVALUATION
//============================================================================//

void MocoContactPointConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state,
        SimTK::Vector&      errors) const {

    // Height is purely kinematic — Position stage is sufficient
    getModel().getMultibodySystem().realize(state, SimTK::Stage::Position);

    const double planeHeight = get_ground_plane_height();
    const int    nPoints     = static_cast<int>(_resolvedPoints.size());

    for (int i = 0; i < nPoints; ++i) {
        const ContactPoint& cp =
            getModel().getComponent<ContactPoint>(
                _resolvedPoints[i].absolutePath);

        // Error = vertical coord - plane height
        // Must be >= 0 (lower bound = 0, upper bound = +inf)
        // getLocationInGround() is inherited from Station via ContactPoint
        errors[i] =
            cp.getLocationInGround(state)[1] - planeHeight;
    }
}
