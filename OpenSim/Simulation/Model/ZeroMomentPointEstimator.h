#ifndef OPENSIM_ZERO_MOMENT_POINT_ESTIMATOR_H
#define OPENSIM_ZERO_MOMENT_POINT_ESTIMATOR_H
/* -------------------------------------------------------------------------- *
 *                 OpenSim: ZeroMomentPointEstimator.h                        *
 * -------------------------------------------------------------------------- *
 * Estimates ground reaction forces (GRFs) and centres of pressure (CoPs)     *
 * from model kinematics using the Zero Moment Point (ZMP) method.            *
 *                                                                            *
 * Supports a maximum of TWO contact sides (e.g. left and right foot).        *
 * The component detects whether unilateral or bilateral contact is           *
 * occurring at each state and selects the appropriate ZMP formulation:       *
 *                                                                            *
 *  UNILATERAL  — one side in contact.                                        *
 *    The total net GRF and ZMP are assigned entirely to that side.           *
 *    CoP = ZMP (whole-body, ground-plane projected).                         *
 *                                                                            *
 *  BILATERAL   — both sides in contact (double support).                     *
 *    The problem is statically indeterminate.                                *
 *                                                                            *
 *                                                                            *
 *    NOTE: BILATERAL contact doesn't work yet!                               *
 *    TODO: add in an option for different bilateral models that use          *
 *    different methods to split the GRFs (e.g. 'walking' mode)               *
 *                                                                            *
 * Contact detection uses ContactSide / ContactPoint subcomponents. A side    *
 * is considered IN CONTACT when ANY of its ContactPoints reports contact     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Common/GCVSplineSet.h>

#include <memory>

namespace OpenSim {

/** Contact mode determined at each state evaluation. */
enum class ContactMode {
    None,        ///< No side in contact — all outputs zeroed / invalid
    Unilateral,  ///< Exactly one side in contact
    Bilateral    ///< Both sides in contact (double support)
};

/** Individual evaluation results for a single contact limb side. */
struct SideResult {
    SimTK::Vec3 grf{0.0};
    SimTK::Vec3 cop{0.0};
    SimTK::Vec3 freeMoment{0.0};
    bool isInContact{false};
    bool isValid{false};
};

/** Top-level aggregated ZMP evaluation result stored in the state cache. */
struct ZMPResult {
    SimTK::Array_<SideResult> sides; ///< parallel to contact_sides list
    ContactMode mode{ContactMode::None};
};

//============================================================================//
//                       ZeroMomentPointEstimator                             //
//============================================================================//
/**
 * Estimates per-side ground reaction forces and centres of pressure from
 * model kinematics via the Zero Moment Point method.
 * 
 * The Zero Moment Point method is outlined in Xiang et al. (2009) and an implementation
 * to predicting GRFs during gait explored by Dijkstra & Gutierrez-Farewik (2015).
 * The method for checking ground contact with points on a body is similar to the
 * method implemented in Karcnik (2003).
 *  
 * Xiang et al. (2009): https://doi.org/10.1002/nme.2575
 * Dijkstra & Gutierrez-Farewik (2015): https://doi.org/10.1016/j.jbiomech.2015.08.027
 * Karcnik  (2003): https://doi.org/10.1007/BF02345310).
 *
 * Supports a maximum of two ContactSide subcomponents.
 *
 * ### Properties
 * - `contact_sides`           : list<ContactSide>  (max 2)
 * - `free_joint_name`         : string (default ground_pelvis)
 *     Joint that connects model to ground (i.e. root) for ID residual extraction
 * - `contact_force_threshold` : double  (default 20.0 N)
 *     Minimum estimated total vertical GRF to treat results as valid.
 *
 * ### List Outputs  (one channel per ContactSide, keyed by side_name)
 * - `GRF`          : Vec3  @ Acceleration — ground reaction force (N)
 * - `CoP`          : Vec3  @ Acceleration — centre of pressure / ZMP (m)
 * - `free_moment`  : Vec3  @ Acceleration — residual free moment at CoP (Nm)
 * - `is_in_contact`: bool  @ Position     — any contact point active
 * - `is_valid`     : bool  @ Acceleration — in contact AND Fvert > threshold
 *
 * ### Scalar Outputs
 * - `contact_mode` : int @ Position
 *     0 = None, 1 = Unilateral, 2 = Bilateral
 *
 * @author  Aaron Fox
 */
class OSIMSIMULATION_API ZeroMomentPointEstimator : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ZeroMomentPointEstimator, ModelComponent);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================
    OpenSim_DECLARE_LIST_PROPERTY(contact_sides, ContactSide,
        "Limb sides for contact detection and GRF application (currently limiting to 2).");

    OpenSim_DECLARE_PROPERTY(free_joint_name, std::string,
        "The name of the free joint (i.e. the joint that connects the "
        "model to the ground) in the model (defaults to ground_pelvis).");

    OpenSim_DECLARE_PROPERTY(contact_force_threshold, double,
        "Minimum total vertical GRF (N) for outputs to be marked valid. "
        "Default 20.0 N.");

    //=========================================================================
    // LIST OUTPUTS — one channel per ContactSide
    //=========================================================================
    OpenSim_DECLARE_LIST_OUTPUT(GRF, SimTK::Vec3,
        getGRFForChannel, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(CoP, SimTK::Vec3,
        getCoPForChannel, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(free_moment, SimTK::Vec3,
        getFreeMomentForChannel, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(is_in_contact, bool,
        getIsInContactForChannel, SimTK::Stage::Acceleration);

    OpenSim_DECLARE_LIST_OUTPUT(is_valid, bool,
        getIsValidForChannel, SimTK::Stage::Acceleration);

    //=========================================================================
    // SCALAR OUTPUTS
    //=========================================================================
    OpenSim_DECLARE_OUTPUT(contact_mode, int,
        getContactModeAsInt, SimTK::Stage::Acceleration);

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================
    
	/** Default constructor */
	ZeroMomentPointEstimator();
    	
	/**
     * Convenience constructor specifying free joint with other defaults.
     * @param freeJointName     Free joint used in ID residual extraction
     */
    ZeroMomentPointEstimator(const std::string& freeJointName);
	
	/**
     * Convenience constructor specifying free joint and force threshold.
     * @param contactForceThreshold     Vertical force threshold to consider results valid
     */
    ZeroMomentPointEstimator(const std::string& freeJointName,
        const double& contactForceThreshold);

    virtual ~ZeroMomentPointEstimator() = default;

    //=========================================================================
    // CONTACT SIDE MANAGEMENT
    //=========================================================================

    /** Appends a ContactSide (max 2) and adopts it as a subcomponent.
     *  Throws if a third side is added. Returns new side index. */
    int addContactSide(ContactSide* side);

    /** Gets current number of contact sides */
    int getNumContactSides() const;
    
    /** Gets contact side by index or name */
    const ContactSide& getContactSide(int i) const;
    const ContactSide& getContactSide(const std::string& sideName) const;

    //=========================================================================
    // METHODS
    //=========================================================================

    /** Get and set the free joint name in the component. */
    const std::string& getFreeJointName() const {
        return get_free_joint_name();
    }
    void setFreeJointName(const std::string& freeJointName);

    /** Get and set the force threshold for considering when ground contact
    has occurred. */
    const double& getContactForceThreshold() const {
        return get_contact_force_threshold(); 
    }
    void setContactForceThreshold(const double& contactForceThreshold);

    /** Set the generalised accelerations (udot) to use in the ID solve.
     *  Call this before evaluating outputs when using kinematically-derived
     *  accelerations rather than the state's forward-dynamics udot. */
    void setUDot(const SimTK::Vector& udot);

    //=========================================================================
    // OUTPUT EVALUATION METHODS
    //=========================================================================
    SimTK::Vec3 getGRFForChannel(const SimTK::State& s,
                                 const std::string& channel) const;

    SimTK::Vec3 getCoPForChannel(const SimTK::State& s,
                                 const std::string& channel) const;

    SimTK::Vec3 getFreeMomentForChannel(const SimTK::State& s,
                                        const std::string& channel) const;

    bool getIsInContactForChannel(const SimTK::State& s,
                                  const std::string& channel) const;

    bool getIsValidForChannel(const SimTK::State& s,
                              const std::string& channel) const;

    /** Returns the ContactMode as int for the scalar output. */
    int getContactModeAsInt(const SimTK::State& s) const;

    /** Returns the ContactMode enum for direct programmatic use. */
    ContactMode getContactMode(const SimTK::State& s) const;

protected:
    //=========================================================================
    // MODELCOMPONENT INTERFACE
    //=========================================================================
    void extendFinalizeFromProperties()              override;
    void extendConnectToModel(Model& model)          override;
    void extendAddToSystem(SimTK::MultibodySystem&)  const override;

private:
    ////=========================================================================
    //// INTERNAL RESULT STRUCT
    ////=========================================================================
    //struct SideResult {
    //    SimTK::Vec3 grf{0.0};
    //    SimTK::Vec3 cop{0.0};
    //    SimTK::Vec3 freeMoment{0.0};
    //    bool        isInContact{false};
    //    bool        isValid{false};
    //};

    ///** Top-level result stored in the cache variable. */
    //struct ZMPResult {
    //    SimTK::Array_<SideResult> sides;  ///< parallel to contact_sides list
    //    ContactMode               mode{ContactMode::None};
    //};

    //=========================================================================
    // COMPUTATION
    //=========================================================================

    /** Returns the side index for an output channel name. Throws if absent. */
    int getSideIndex(const std::string& channelName) const;

    // Gateway caching method for getting ZMP result
    const ZMPResult& getZMPResult(const SimTK::State& s) const;

    ///** Ensures the cache is valid for the current state. */
    //void ensureResultsValid(const SimTK::State& s) const;

    /** Full ZMP pipeline — called only when cache is stale. */
    ZMPResult computeZMPResult(const SimTK::State& s) const;

    /**
     * Bilateral case: both sides in contact.
     *
     * Solves for alpha ∈ [0,1] such that:
     *   ZMP_total = alpha * centroid_0 + (1-alpha) * centroid_1
     *
     * alpha is the projection of (ZMP - centroid_1) onto the unit vector
     * from centroid_1 to centroid_0, clamped to [0,1].
     *
     * Vertical load per side:
     *   Fv_0 = alpha     * Fv_total
     *   Fv_1 = (1-alpha) * Fv_total
     *
     * Horizontal forces split by the same alpha. Per-side CoP is fixed at
     * each side's active contact centroid. Free moment (vertical component
     * only) is split by alpha.
     */
    //void computeBilateral(const SimTK::Vec3&  netForce,
    //                      const SimTK::Vec3&  netMoment,
    //                      const SimTK::Vec3&  zmpTotal,
    //                      const SimTK::Vec3&  centroid0,
    //                      const SimTK::Vec3&  centroid1,
    //                      ZMPResult&          result) const;

    void constructProperties();

    //=========================================================================
    // PRIVATE MEMBERS
    //=========================================================================
    
    // ResetOnCopy so copies don't share solver state.
    mutable SimTK::ResetOnCopy<std::unique_ptr<InverseDynamicsSolver>>
        _idSolver;

    // Typed cache variable — avoids per-output string lookup.
    mutable CacheVariable<ZMPResult> _zmpResultCV;

    // Externally provided udot — used instead of s.getUDot() when set
    mutable SimTK::ResetOnCopy<std::unique_ptr<SimTK::Vector>> _externalUDot;


};

} // namespace OpenSim

#endif // OPENSIM_ZERO_MOMENT_POINT_ESTIMATOR_H
