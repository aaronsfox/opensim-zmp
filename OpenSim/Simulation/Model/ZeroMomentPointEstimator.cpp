/* -------------------------------------------------------------------------- *
 *               OpenSim: ZeroMomentPointEstimator.cpp                        *
 * -------------------------------------------------------------------------- */

#include "ZeroMomentPointEstimator.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

#include <cmath>
#include <sstream>

using namespace OpenSim;

//============================================================================//
// CONSTRUCTION
//============================================================================//

/** Default constructor */
ZeroMomentPointEstimator::ZeroMomentPointEstimator() {
    constructProperties();
}

/**
 * Convenience constructor specifying free joint with other defaults.
 * @param freeJointName     Free joint used in ID residual extraction
 */
ZeroMomentPointEstimator::ZeroMomentPointEstimator(
        const std::string& freeJointName) {
    constructProperties();
    set_free_joint_name(freeJointName);
}

/**
 * Convenience constructor specifying free joint and force threshold.
 * @param contactForceThreshold     Vertical force threshold to consider results
 * valid
 */
ZeroMomentPointEstimator::ZeroMomentPointEstimator(
        const std::string& freeJointName,
        const double& contactForceThreshold) {
    constructProperties();
    set_free_joint_name(freeJointName);
    set_contact_force_threshold(contactForceThreshold);
}

/** Construct default properties */
void ZeroMomentPointEstimator::constructProperties() {
    constructProperty_contact_sides();
    constructProperty_free_joint_name("ground_pelvis");
    constructProperty_contact_force_threshold(20.0);
}

//============================================================================//
// CONTACT SIDE MANAGEMENT
//============================================================================//

/** Appends a ContactSide (max 2) and adopts it as a subcomponent.
 *  Throws if a third side is added. Returns new side index. */
int ZeroMomentPointEstimator::addContactSide(ContactSide* side) {
    OPENSIM_THROW_IF_FRMOBJ(side == nullptr, Exception,
        "ContactSide pointer must not be null.");
    OPENSIM_THROW_IF_FRMOBJ(getProperty_contact_sides().size() >= 2, Exception,
        "ZeroMomentPointEstimator supports a maximum of 2 ContactSides. "
        "'" + getName() + "' already has 2.");
    const int idx = append_contact_sides(*side);
    return idx;
}

/** Gets current number of contact sides */
int ZeroMomentPointEstimator::getNumContactSides() const {
    return getProperty_contact_sides().size();
}

/** Gets contact side by index or name */
const ContactSide& ZeroMomentPointEstimator::getContactSide(int i) const {
    return get_contact_sides(i);
}
const ContactSide& ZeroMomentPointEstimator::getContactSide(
        const std::string& sideName) const {
    for (int i = 0; i < getNumContactSides(); ++i)
        if (get_contact_sides(i).getName() == sideName)
            return get_contact_sides(i);
    OPENSIM_THROW_FRMOBJ(Exception,
        "No ContactSide with side_name '" + sideName + "' in '" +
        getName() + "'.");
}

//=========================================================================
// METHODS
//=========================================================================

/** Set the free joint name in the component. */
void ZeroMomentPointEstimator::setFreeJointName(
        const std::string& freeJointName) {
    set_free_joint_name(freeJointName);
};

/** Set the force threshold for considering when ground contact
has occurred. */
void ZeroMomentPointEstimator::setContactForceThreshold(
        const double& contactForceThreshold) {
    set_contact_force_threshold(contactForceThreshold);
}

/** Set the generalised accelerations (udot) to use in the ID solve.
 *  Call this before evaluating outputs when using kinematically-derived
 *  accelerations rather than the state's forward-dynamics udot. */
void ZeroMomentPointEstimator::setUDot(const SimTK::Vector& udot) {
    _externalUDot.reset(new SimTK::Vector(udot));
}

//============================================================================//
// MODELCOMPONENT INTERFACE
//============================================================================//

// Check and register properties for component
void ZeroMomentPointEstimator::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    // Get the number of contact sides
    const int n = getProperty_contact_sides().size();
    
    // Throw an exception is object has no contact sides
    OPENSIM_THROW_IF_FRMOBJ(n == 0, Exception,
        "ZeroMomentPointEstimator '" + getName() + "' has no ContactSides.");
    
    // Throw an exception is object has more than 2 contact sides
    OPENSIM_THROW_IF_FRMOBJ(n > 2, Exception,
        "ZeroMomentPointEstimator '" + getName() + "' has " +
        std::to_string(n) + " ContactSides; maximum is 2.");

    // Throw an exception if the contact force threshold is negative
    OPENSIM_THROW_IF_FRMOBJ(get_contact_force_threshold() < 0.0, Exception,
        "contact_force_threshold must be non-negative.");

    // Register one list-output channel per side, keyed by side_name.
    for (int i = 0; i < getNumContactSides(); ++i) {
        const std::string& ch = get_contact_sides(i).getName();
        updOutput("GRF").addChannel(ch);
        updOutput("CoP").addChannel(ch);
        updOutput("free_moment").addChannel(ch);
        updOutput("is_in_contact").addChannel(ch);
        updOutput("is_valid").addChannel(ch);
    }

}

// Connect to model
void ZeroMomentPointEstimator::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    _idSolver.reset(new InverseDynamicsSolver(model));
}

// Connect to Simbody multibody system
void ZeroMomentPointEstimator::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    
    // Base class first
    Super::extendAddToSystem(system);

    // Register the typed ZMPResult cache variable
    // Computing ZMP requires inverse dynamics residual extraction
    // (accelerations), so we place the invalidation stage at
    // SimTK::Stage::Acceleration.
    //ZMPResult empty;
    //empty.sides.resize(getNumContactSides(), SideResult{});
    // Stage::Acceleration: the ID solve needs udot.
    _zmpResultCV = addCacheVariable(
        "zmp_result", ZMPResult(), SimTK::Stage::Acceleration);
}

//============================================================================//
// COMPUTATION
//============================================================================//

/** Returns the side index for an output channel name. Throws if absent. */
int ZeroMomentPointEstimator::getSideIndex(
        const std::string& channelName) const {
    for (int i = 0; i < getNumContactSides(); ++i)
        if (get_contact_sides(i).getName() == channelName)
            return i;
    OPENSIM_THROW_FRMOBJ(Exception,
        "Output channel '" + channelName + "' not found in '" + getName() + "'.");
}

/** Returns the ContactMode enum for direct programmatic use. */
ContactMode ZeroMomentPointEstimator::getContactMode(
        const SimTK::State& s) const {
    // ContactMode is determined at Stage::Acceleration — no ID solve needed.
    // We read directly from the ContactSides rather than from the cache.
    const int n = getNumContactSides();
    int activeCount = 0;
    for (int i = 0; i < n; ++i)
        if (get_contact_sides(i).isInContact(s)) ++activeCount;

    if (activeCount == 0)
        return ContactMode::None;
    else if (activeCount == 1)
        return ContactMode::Unilateral;
    else
        return ContactMode::Bilateral;
}

/** Returns the ContactMode as int for the scalar output. */
int ZeroMomentPointEstimator::getContactModeAsInt(const SimTK::State& s) const {
    return static_cast<int>(getContactMode(s));
}

bool ZeroMomentPointEstimator::getIsInContactForChannel(
        const SimTK::State& s, const std::string& ch) const {
    // is_in_contact is a Stage::Position output — evaluate directly from the
    // ContactSide without requiring the Acceleration-stage solve.
    return get_contact_sides(getSideIndex(ch)).isInContact(s);
}

bool ZeroMomentPointEstimator::getIsValidForChannel(
        const SimTK::State& s, const std::string& ch) const {
    const ZMPResult& res = getZMPResult(s);
    return res.sides[getSideIndex(ch)].isValid;
}

//============================================================================//
// CACHING GATEWAY
//============================================================================//

// Function to get and update cache variable result
const ZMPResult& ZeroMomentPointEstimator::getZMPResult(
        const SimTK::State& s) const {

    // If cache is already valid for state, return it instantly
    if (isCacheVariableValid(s, _zmpResultCV)) {
        return getCacheVariableValue<ZMPResult>(s, _zmpResultCV);
    }

    // If the cache is invalid prepare a fresh result
    ZMPResult freshResult;

    // Initialize array layouts based on contact sides
    int numSides = getNumContactSides();
    freshResult.sides.resize(numSides);

    // Evaluate foot contact conditions
    bool anySideInContact = false;
    for (int i = 0; i < numSides; ++i) {
        if (getContactSide(i).isInContact(s)) { anySideInContact = true; }
    }

    // Check whether to run computation pipeline
    if (anySideInContact) {
        // Run full computation pipeline
        freshResult = computeZMPResult(s);
    } else {
        // No contact detected: Populate cache with defaults
        for (int i = 0; i < numSides; ++i) {
            freshResult.sides[i].grf = SimTK::Vec3(0.0);
            freshResult.sides[i].cop = SimTK::Vec3(0.0);
            freshResult.sides[i].freeMoment = SimTK::Vec3(0.0);
        }
    }

    // Commit the new result back to the state cache system and mark as valid
    setCacheVariableValue(s, _zmpResultCV, freshResult);
    markCacheVariableValid(s, _zmpResultCV); // ← add this line

    // Return the safely cached reference
    return getCacheVariableValue<ZMPResult>(s, _zmpResultCV);

}

//============================================================================//
// CHANNEL ACCESS (ROUTED VIA CACHE GATEWAY)
//============================================================================//

SimTK::Vec3 ZeroMomentPointEstimator::getGRFForChannel(
        const SimTK::State& s, const std::string& ch) const {
    const ZMPResult& res = getZMPResult(s);
    return res.sides[getSideIndex(ch)].grf;
}

SimTK::Vec3 ZeroMomentPointEstimator::getCoPForChannel(
        const SimTK::State& s, const std::string& ch) const {
    const ZMPResult& res = getZMPResult(s);
    return res.sides[getSideIndex(ch)].cop;
}

SimTK::Vec3 ZeroMomentPointEstimator::getFreeMomentForChannel(
        const SimTK::State& s, const std::string& ch) const {
    const ZMPResult& res = getZMPResult(s);
    return res.sides[getSideIndex(ch)].freeMoment;
}

//============================================================================//
// COMPUTATION PIPELINE
//============================================================================//

//void ZeroMomentPointEstimator::ensureResultsValid(
//        const SimTK::State& s) const {
//    if (!isCacheVariableValid(s, _zmpResultCV)) {
//        ZMPResult result = computeZMPResult(s);
//        setCacheVariableValue(s, _zmpResultCV, result);
//        markCacheVariableValid(s, _zmpResultCV);
//    }
//}

/** ZMP formula for a flat ground plane assuming y is the vertical axis
 *  Returns CoP in Ground frame. Returns Vec3(0) if |Fvert| < force
 * threshold. */
ZMPResult ZeroMomentPointEstimator::computeZMPResult(const SimTK::State& s) const {

    // Get parameters from component for calculation
    const int    n     = getNumContactSides();
    const double fThr  = get_contact_force_threshold();

    // Set result variables
    ZMPResult result;
    result.sides.resize(n, SideResult{});

    // Get model and realize states
    const auto& model = getModel();
    model.realizeAcceleration(s);

    // ------------------------------------------------------------------
    // Contact detection
    // Determine mode and populate is_in_contact for each side.
    // ------------------------------------------------------------------
    
    // Set starting variables for count of contact sides
    int  activeCount   = 0;
    int  activeSideIdx = -1;   // used only in the unilateral case

    // Check which sides are in contact
    for (int i = 0; i < n; ++i) {
        const bool inContact = get_contact_sides(i).isInContact(s);
        result.sides[i].isInContact = inContact;
        if (inContact) {
            ++activeCount;
            activeSideIdx = i;
        }
    }

    // If no contact, return results all outputs zeroed / invalid
    if (activeCount == 0) {
        result.mode = ContactMode::None;
        return result;   // all outputs zeroed / invalid
    }

    // Set unilateral mode if one active side, otherwise set bilateral mode
    result.mode = (activeCount == 1) ? ContactMode::Unilateral
                                     : ContactMode::Bilateral;

    // ------------------------------------------------------------------
    // Inverse dynamics at Stage::Acceleration.
    // Obtain net external load (force + moment) in Ground frame.
    // ------------------------------------------------------------------

    // Get the free body name and associated joint
    const Joint& freeJoint = model.getJointSet().get(get_free_joint_name());

    // Create ID solver
    if (!_idSolver)
        _idSolver.reset(new InverseDynamicsSolver(model));

    // Check-out the values for udot. This looks firstly for the externally
    // provided udot from a table, otherwise using the forward dynamics 
    // driven approach from the states
    const SimTK::Vector udot = _externalUDot ? *_externalUDot : s.getUDot();

    // Extract the generalised coordinate forces
    const SimTK::Vector      tau     = _idSolver->solve(s, udot);

    // Calculate the equivalent body force at the free joint in the model
    // Uses the generalised force trajectory passed from ID solver
    SimTK::SpatialVec netLoad =
            freeJoint.calcEquivalentSpatialForce(s, tau);
    
    // Extract the net force and moments from the SpatialVec
    const SimTK::Vec3 netForce = netLoad[1];
    const SimTK::Vec3 netMoment = netLoad[0];

    // Check if vertical force exceeds contact force threshold
    // Assumes vertical force is in the y-axis
    const double Fvert = netForce[1];
    if (Fvert < fThr)
        return result;   // all outputs zeroed / invalid

    // ------------------------------------------------------------------
    // Compute the ZMP to get the CoP on the ground plane
    // ------------------------------------------------------------------

    // Get the position of the free body in the ground frame
    // This is based on getting the child frame of the free joint
    // and hence assumes that the ground should always be the
    // parent.
    // Getting the frames position in the ground should represent
    // it's translational coordinates, as it appears robust to frame
    // translation in the model. The frames origin (i.e. 0,0,0)
    // can be used as the station. An adaptation to this could be to use
    // the centre of mass of the body frame to estimate of the ground
    // reactions.
    SimTK::Vec3 rp =
    freeJoint.getChildFrame().findStationLocationInGround(
        s, SimTK::Vec3(0, 0, 0));  

    // Calculate moment at origin
    // Formulas used here come from Xiang et al. 2009:
    // https://doi.org/10.1002/nme.2575
    SimTK::Vec3 rp_fbf = SimTK::cross(rp, netForce);
    SimTK::Vec3 groundM = 
        SimTK::Vec3(netMoment.get(0) + rp_fbf.get(0),
            netMoment.get(1) + rp_fbf.get(1),
            netMoment.get(2) + rp_fbf.get(2));

    // Calculate X & Z cZMP, noting that yZMP is set as 0
    // Given this (and some other calculations) the standard
    // OpenSim coordinate system must be used.
    // Formula used here come from Xiang et al. 2009:
    // https://doi.org/10.1002/nme.2575
    SimTK::Vec3 zmpCOP =
        SimTK::Vec3(groundM.get(2) / netForce.get(1), 0,
            -groundM.get(0) / netForce.get(1));

    // Calculate the resultant active moment at ZMP along the y-axis
    // TODO: This formula still doesn't seem quite right?
    double myZMP = groundM.get(1) + 
        (netForce.get(0) * zmpCOP.get(2)) - 
        (netForce.get(2) * zmpCOP.get(1));

    // ------------------------------------------------------------------
    // Dispatch results to unilateral or bilateral mode calculations
    // ------------------------------------------------------------------
    
    // Unilateral mode
    // There is no need for an additional function as it is simple to 
    // just allocate the data to the active side
    if (result.mode == ContactMode::Unilateral) {
        
        // Set the active side to allocate forces to
        SideResult& sr = result.sides[activeSideIdx];

        // The total GRF and ZMP belong entirely to the active side
        // The inactive side (if present) remains zeroed / invalid.
        sr.grf = netForce;
        sr.cop = zmpCOP;
        sr.freeMoment = SimTK::Vec3(0.0, myZMP, 0.0);
        sr.isValid = true;

    } else {

        // A bilateral contact function (and perhaps more...) needs to be created
        // Likely need to adopt some motion specific (e.g. walking) methods for this
        std::cout << "Bilateral contact not yet supported...results will be all zeroes...Sorry..."
            << std::endl;

        return result; // all outputs zeroed / invalid

    }

    return result;
}

//============================================================================//
// BILATERAL
//============================================================================//

// TODO: below is commented out, as this is a theoretical approach to the problem...
//void ZeroMomentPointEstimator::computeBilateral(
//        const Vec3& netForce,
//        const Vec3& netMoment,
//        const Vec3& zmpTotal,
//        const Vec3& centroid0,
//        const Vec3& centroid1,
//        ZMPResult&  result) const {
//
//    const int    vAxis = get_vertical_axis();
//    const double hGnd  = get_ground_plane_height();
//
//    // ------------------------------------------------------------------
//    // Solve for alpha ∈ [0,1] such that:
//    //
//    //   zmpTotal = alpha * centroid0 + (1-alpha) * centroid1
//    //   =>  zmpTotal - centroid1 = alpha * (centroid0 - centroid1)
//    //
//    // We solve by projecting onto the inter-centroid axis (horizontal
//    // plane only, to avoid the vertical component contaminating alpha).
//    //
//    // If the two centroids are coincident (degenerate double support)
//    // we fall back to alpha = 0.5 (equal split).
//    // ------------------------------------------------------------------
//    int h0 = (vAxis + 1) % 3;
//    int h1 = (vAxis + 2) % 3;
//
//    const double dx =  centroid0[h0] - centroid1[h0];
//    const double dz =  centroid0[h1] - centroid1[h1];
//    const double lenSq = dx*dx + dz*dz;
//
//    double alpha = 0.5;   // fallback for coincident centroids
//    if (lenSq > 1e-8) {
//        const double rx = zmpTotal[h0] - centroid1[h0];
//        const double rz = zmpTotal[h1] - centroid1[h1];
//        alpha = (rx*dx + rz*dz) / lenSq;
//        // Clamp to [0,1] so GRFs remain non-negative under each foot.
//        alpha = std::max(0.0, std::min(1.0, alpha));
//    }
//
//    const double oneMinusAlpha = 1.0 - alpha;
//
//    // ------------------------------------------------------------------
//    // Distribute force by alpha.
//    // All three components (vertical and horizontal) are split by the
//    // same scalar — this preserves the direction of the total GRF vector
//    // under each foot, which is the simplest physically consistent choice
//    // available without additional sensor data.
//    // ------------------------------------------------------------------
//    SideResult& sr0 = result.sides[0];
//    SideResult& sr1 = result.sides[1];
//
//    sr0.grf = alpha          * netForce;
//    sr1.grf = oneMinusAlpha  * netForce;
//
//    // Per-side CoP is fixed at each side's active contact centroid,
//    // projected onto the ground plane.
//    sr0.cop        = centroid0;
//    sr0.cop[vAxis] = hGnd;
//    sr1.cop        = centroid1;
//    sr1.cop[vAxis] = hGnd;
//
//    // Residual free moment (vertical component only) split by alpha.
//    const Vec3 fm0Full = computeFreeMoment(netMoment, netForce, sr0.cop, vAxis);
//    const Vec3 fm1Full = computeFreeMoment(netMoment, netForce, sr1.cop, vAxis);
//    sr0.freeMoment        = Vec3(0.0);
//    sr0.freeMoment[vAxis] = alpha         * fm0Full[vAxis];
//    sr1.freeMoment        = Vec3(0.0);
//    sr1.freeMoment[vAxis] = oneMinusAlpha * fm1Full[vAxis];
//
//    sr0.isValid = true;
//    sr1.isValid = true;
//}