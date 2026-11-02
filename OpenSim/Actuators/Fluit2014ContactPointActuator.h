#ifndef OPENSIM_FLUIT2014_CONTACT_POINT_ACTUATOR_H_
#define OPENSIM_FLUIT2014_CONTACT_POINT_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                OpenSim:  Fluit2014ContactPointActuator.h                   *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim {

class Body;
class Model;

//=============================================================================
//                         CONTACT POINT ACTUATOR
//=============================================================================
/**
 * A five-control `Actuator` that reproduces a single ground-contact point of
 * the GRF&M-prediction method of:
 *
 *   Fluit R, Andersen MS, Kolk S, Verdonschot N, Koopman HFJM. Prediction of
 *   ground reaction forces and moments during various activities of daily
 *   living. J Biomech. 2014;47(10):2321-2329.
 *
 * and:
 *
 *   Skals S, Jung MK, Damsgaard M, Andersen MS. Prediction of ground
 *   reaction forces and moments during sports-related movements. Multibody
 *   Syst Dyn. 2017;39:175-195.
 *
 * Both papers place, at each contact point beneath the foot, FIVE unilateral
 * ("push-only") muscle-like actuators, each independently controlled by its
 * own activation level (`q` in `[0, 1]`) as part of the muscle-recruitment or
 * optimal-control problem:
 *   - 1 actuator aligned with the ground-normal (+Y) direction,
 *   - 2 actuators approximating +/- antero-posterior shear. Skals et al. tilted 
 *     this into the ground plane at angle atan(mu), so that activating one
 *     produces a normal-direction AND a shear-direction force
 *     simultaneously, in a fixed ratio, whereas Fluit et al. did not implement 
 *     this and hence a mu = 0 produces the purely horizontal shear force with
 *     no explicit friction bound).
 *   - 2 actuators approximating +/- medio-lateral shear, analogous to
 *     the above.
 *
 * This class models one component per contact point with FIVE independent
 * controls (`numControls() == 5`), Control indices are, in order
 * (see `ControlIndex`):
 *
 *   0: normal (+Y)
 *   1: shear  (+X)
 *   2: shear  (-X)
 *   3: shear  (+Z)
 *   4: shear  (-Z)
 *
 * Each control independently scales its own actuator's fixed unit-force
 * direction (`getActuatorDirection()`). All five share the same maximum
 * strength (`max_force`; Fmax/Nmax in the source papers) and the same
 * instantaneous, state-dependent strength-scaling factor (see below). None
 * of the five actuators' forces is computed as a function of another's
 * control value (i.e. all five are free optimization variables).
 *
 * ## Oblique shear directions (friction coupling)
 *
 * Following Skals et al., each shear actuator's unit direction vector is
 * tilted off the ground-normal by `atan(friction_coefficient)`:
 * 
 * \f[
 *   \hat{d}_{shear,\pm X} = \frac{(\pm\mu,\,1,\,0)}{\sqrt{1+\mu^2}}, \qquad
 *   \hat{d}_{shear,\pm Z} = \frac{(0,\,1,\,\pm\mu)}{\sqrt{1+\mu^2}}
 * \f]
 * 
 * so that activating one produces a normal-direction force component and a
 * shear-direction component in a fixed `mu : 1` ratio, by construction (a
 * four-facet, "pyramidal" approximation of a Coulomb friction cone).
 * Setting `friction_coefficient` to 0 recovers untilted, unconstrained 
 * shear actuators as in Fluit et al.
 *
 * ## Strength scaling
 *
 * As in Fluit et al. (Eq. 1) and Skals et al. (Eq. 2), each actuator's
 * maximum available force is scaled by a factor in (0, 1) that depends on
 * the instantaneous state of the contact point (not on any actuator's
 * control), reflecting that a point can only push against the ground when
 * it is at/near ground level. This strength scaling function can be set to
 * use the contact points distance AND velocity to the ground by setting the
 * use_velocity_in_scaling parameter to True, otherwise only the contact point
 * distance to the ground is used. Using the distance only mode is likely most
 * useful in a context like treadmill running where the contact point velocity
 * is unlikely to go under the threshold as it keeps moving.
 *
 * Both modes use a smooth logistic approximation to the papers' original
 * piecewise on/off strength function, with steepness set by `smoothing`;
 * see `calcStrengthScaleFactor()`.
 *
 * ## Control bounds
 *
 * Per the source papers, all five controls represent unilateral activation
 * levels and should be bounded to `[0, 1]` (`min_control`/`max_control`,
 * applied uniformly to all five controls; defaults 0 and 1). If solving an 
 * optimal control problem, also set the per-control bounds explicitly there 
 * (e.g. via `MocoProblem::setControlInfo()`, addressing this actuator's path 
 * and control index 0-4) rather than relying on this class's internal clamp
 * alone.
 *
 * @author Aaron Fox
 */

class OSIMACTUATORS_API Fluit2014ContactPointActuator : public Actuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(Fluit2014ContactPointActuator, Actuator);

public:

    //=========================================================================
    // CONTROL INDICES
    //=========================================================================

    /** Indices into this actuator's 5-element control vector. **/
    enum ControlIndex {
        Control_Normal = 0,
        Control_ShearPosX = 1,
        Control_ShearNegX = 2,
        Control_ShearPosZ = 3,
        Control_ShearNegZ = 4
    };
    static const int NumContactControls = 5;

    //=========================================================================
    // PROPERTIES
    //=========================================================================

    OpenSim_DECLARE_OPTIONAL_PROPERTY(body_name, std::string,
        "Name of the body to which this actuator's contact point is attached "
        "and applies force to.");

    OpenSim_DECLARE_PROPERTY(point, SimTK::Vec3,
        "Location of the contact point; interpreted in the body frame "
        "unless point_is_global is true.");

    /** The default is point_is_global=false. **/
    OpenSim_DECLARE_PROPERTY(point_is_global, bool,
        "Interpret point in Ground if true; otherwise, in the body frame. "
        "Default: false.");

    OpenSim_DECLARE_PROPERTY(max_force, double,
        "Maximum (fully-active) force magnitude that EACH of the five "
        "unilateral actuators at this contact point can produce along its "
        "own fixed direction; Fmax/Nmax in Fluit et al. (2014) and Skals "
        "et al. (2017). Units of force (e.g., N). Default: 1.0.");

    OpenSim_DECLARE_PROPERTY(ground_height, double,
        "Height (Y-coordinate, expressed in Ground) of the floor/ground "
        "plane that this actuator acts against. Default: 0.0.");

    OpenSim_DECLARE_PROPERTY(contact_height_threshold, double,
        "Height of the contact point above the ground plane (z_crit in "
        "Fluit et al., 2014) below which the actuator approaches full "
        "strength. Units of length (e.g., m). Default: 0.03.");

    OpenSim_DECLARE_PROPERTY(contact_velocity_threshold, double,
        "Speed of the contact point (v_crit in Fluit et al., 2014) below "
        "which the actuator approaches full strength. Units of velocity "
        "(e.g., m/s). Default: 0.8.");

    OpenSim_DECLARE_PROPERTY(smoothing, double,
        "Dimensionless steepness parameter (k) of the smooth logistic "
        "approximation to the piecewise strength function of Fluit et al. "
        "(2014) / Skals et al. (2017). Larger values approach the "
        "original sharp on/off strength function; smaller values give a "
        "more gradual, more strongly-differentiable transition, which is "
        "usually preferable in gradient-based optimal control. "
        "Default: 8.");

    OpenSim_DECLARE_PROPERTY(use_velocity_in_scaling, bool,
        "Determines which point variables gate the strength-scaling factor. "
        "Setting this to True uses both the point (z_crit) and velocity (v_crit) "
        "thresholds, while setting to False only uses the distance threshold. "
        "Default: True.");

    OpenSim_DECLARE_PROPERTY(friction_coefficient, double,
        "Coefficient of friction (mu) determining the tilt of the four "
        "shear actuators lines of action off the ground-normal direction "
        "(atan(mu)). Following Skals et al. (2017), activating a shear "
        "actuator produces a normal-direction force component and a "
        "shear-direction component in a fixed mu:1 ratio, by construction. "
        "Set to 0 to recover un-tilted approach used in Fluit et al."
        "Default: 0.5.");

    OpenSim_DECLARE_PROPERTY(min_control, double,
        "Minimum allowed value for each of the five controls. Per the "
        "source papers, all controls are unilateral activation levels; "
        "enforced internally as a clamp on each raw control value when "
        "computing force. Not automatically read by external tools (e.g., "
        "Moco) -- set per-control bounds explicitly there too. "
        "Default: 0.0.");

    OpenSim_DECLARE_PROPERTY(max_control, double,
            "Maximum allowed value for each of the five controls. See "
            "min_control. Default: 1.0.");

    //=========================================================================
    // OUTPUTS
    //=========================================================================
    
    OpenSim_DECLARE_OUTPUT(normal_force, double, getNormalForce,
            SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(strength_scale_factor, double,
            getStrengthScaleFactor, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(contact_point_height, double,
            getContactPointHeight, SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(contact_point_speed, double, 
            getSpeed, SimTK::Stage::Velocity);

    //=========================================================================
    // CONSTRUCTION
    //=========================================================================

    /** Default constructor */
    Fluit2014ContactPointActuator();

    /** Construct with the name and body name of the Body this actuator's contact 
    point is attached to.**/
    explicit Fluit2014ContactPointActuator(const std::string& name,
        const std::string& bodyName);

    //=========================================================================
    // GET & SET METHODS
    //=========================================================================
    
    /** Set and get the 'body_name' property. **/
    void setBodyName(const std::string& bodyName);
    const std::string& getBodyName() const;

    /** Set and get the 'max_force' property. **/
    void setMaxForce(const double& maxForce);
    const double& getMaxForce() const;

    /** Set and get the 'ground_height' property. **/
    void setGroundHeight(const double& groundHeight);
    const double& getGroundHeight() const;

    /** Set and get the 'contact_height_threshold' property. **/
    void setContactHeightThreshold(const double& height);
    const double& getContactHeightThreshold() const;

    /** Set and get the 'contact_velocity_threshold' property. **/
    void setContactVelocityThreshold(const double& velocity);
    const double& getContactVelocityThreshold() const;

    /** Set and get the 'smoothing' property. **/
    void setSmoothingCoefficient(const double& smoothing);
    const double& getSmoothingCoefficient() const;

    /** Set and get the 'friction_coefficient' property. **/
    void setFrictionCoefficient(const double& friction);
    const double& getFrictionCoefficient() const;

    /** Set and get the strength scaling mode property. **/
    void setUseVelocityInScaling(const bool& mode);
    const bool& getUseVelocityInScaling() const;

    /** Set and get the min/max controls. **/
    void setMinControl(const double& minControl);
    const double& getMinControl() const;
    void setMaxControl(const double& maxControl);
    const double& getMaxControl() const;

    //=========================================================================
    // COMPUTATION
    //=========================================================================

    /** Height of the contact point above the ground plane (i.e., its
    ground +Y coordinate minus ground_height), in meters. **/
    double getContactPointHeight(const SimTK::State& s) const;

    /** Gets the contact point velocity, in meters per second. **/
    SimTK::Vec3 getContactPointVelocity(const SimTK::State& s) const;

    /** Gets the contact point location expressed in ground. **/
    SimTK::Vec3 getContactPointLocationInGround(const SimTK::State& s) const;

    /** Value (in the open interval (0, 1)) of the smooth strength-scaling
    factor evaluated from the current position (and, depending on
    strength_scaling_mode, velocity) of the contact point. This factor is
    shared by all five of this actuator's controls. **/
    double getStrengthScaleFactor(const SimTK::State& s) const;

    /** Fixed unit direction (in Ground) of the given control index's line
    of action; depends only on friction_coefficient (see class
    documentation). **/
    SimTK::Vec3 getActuatorDirection(int controlIndex) const;

    /** Force currently produced by a single one of the five actuators
    (its own clamped control, times max_force, times the shared
    strength-scale factor, along its fixed direction), expressed in
    Ground. **/
    SimTK::Vec3 getActuatorForce(const SimTK::State& s, int controlIndex) const;

    /** Total (ground +Y, vertical) component of the summed force from all
    five actuators. Note this includes the normal-direction contribution
    of the (tilted) shear actuators too -- matching how "vertical GRF" is
    computed in the source papers, this is NOT just the dedicated normal
    actuator's own force. **/
    double getNormalForce(const SimTK::State& s) const;

    /** Total (medio-lateral / antero-posterior, ground X-Z) component of
    the summed force from all five actuators. **/
    SimTK::Vec3 getShearForce(const SimTK::State& s) const;

    /** Total (normal + shear) force from all five actuators, expressed in
    the Ground frame. **/
    SimTK::Vec3 getContactForce(const SimTK::State& s) const;

    // Model building / Actuator interface
    int numControls() const override { return NumContactControls; }
    double getPower(const SimTK::State& s) const override;

protected:
    
    //=========================================================================
    // MODEL COMPONENT INTERFACE
    //=========================================================================

    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendConnectToModel(Model& model) override;
    void extendFinalizeFromProperties() override;

private:

    void setNull();
    void constructProperties();

    ///** Set/get the body to which this actuator applies; setting this
    //pointer also sets the corresponding body name property.**/
    //void setBody(Body* aBody);
    //Body* getBody() const;

    //=========================================================================
    // COMPUTATION
    //=========================================================================

    /** Contact point location, in the body frame (resolving point_is_global
    if necessary). **/
    SimTK::Vec3 getPointInBodyFrame(const SimTK::State& s) const;

    /** Location/velocity of the contact point, expressed in Ground. **/
    SimTK::Vec3 calcPointLocationInGround(const SimTK::State& s) const;
    SimTK::Vec3 calcPointVelocityInGround(const SimTK::State& s) const;

    /** Smooth strength-scaling factor, in (0, 1); see class
    documentation. **/
    double calcStrengthScaleFactor(const SimTK::State& s) const;

    /** Contact force calculation for caching. **/
    SimTK::Vec3 calcContactForce(const SimTK::State& s) const;

    /** Speed (norm of the 3D velocity, in Ground) and height of the 
    contact point. **/
    double getSpeed(const SimTK::State& s) const;
    double calcSpeed(const SimTK::State& s) const;
    double getHeight(const SimTK::State& s) const;

    /**
     * Implements the `ForceProducer` interface by applying the combined
     * force of all five actuators to the specified body, at the specified
     * point.
     */
    void implProduceForces(const SimTK::State&, ForceConsumer&) const override;

    //=========================================================================
    // REPORTING
    //=========================================================================

    OpenSim::Array<std::string> getRecordLabels() const override;
    OpenSim::Array<double> getRecordValues(
            const SimTK::State& s) const override;

    // Cache the resolved physical body frame pointer
    mutable SimTK::ResetOnCopy<SimTK::ReferencePtr<const PhysicalFrame>>
            _bodyFrame{nullptr};

    // Cache for storing computed speed of the contact point.
    mutable CacheVariable<double> _speedCV;
    mutable CacheVariable<double> _heightCV;

    // Cache for storing total contact force
    mutable CacheVariable<SimTK::Vec3> _forceCV;


};  // END of class Fluit2014ContactPointActuator

} // namespace OpenSim

#endif // OPENSIM_FLUIT2014_CONTACT_POINT_ACTUATOR_H_