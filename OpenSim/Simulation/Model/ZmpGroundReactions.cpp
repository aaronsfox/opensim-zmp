/* -------------------------------------------------------------------------- *
 * OpenSim: ZmpGroundReactions.cpp                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Aaron Fox                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "ZmpGroundReactions.h"
#include "ContactPointSet.h"
#include "ContactPoint.h"
#include "Model.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

#include <unordered_set>

using namespace OpenSim;
//=============================================================================
//=============================================================================
/**
 * The ZmpGroundReactions is a model component used to calculate estimates of 
 * ground reactions (i.e. forces, moments and point of application) from a model
 * at a particular state or during a particular motion. This allows a prediction
 * of the ground reactions from a motion where experimental measures were unavailable,
 * alongside use in other tools. The component requires the points on bodies that are 
 * expected to come into contact with the ground for specifying when ground contact 
 * has occurred. 
 *  
 * TODO: ZmpForce
 * 
 * The component can also be connected to the ZmpForce class so that the estimated
 * ground reactions can be applied as external forces to the model during dynamic
 * simulations.
 * 
 * Ground reactions are estimated via the Zero Moment Point method when ground contact
 * is identified. Ground contact is identified by first checking if the contact
 * points meet the criteria for ground contact - either based on distance from the 
 * ground plane (which is assumed to be the XZ plane) or by both distance to the 
 * ground plane and meeting a velocity threshold. A force threshold is also specified
 * for if ground contact has occurred, whereby if the vertical force (assumed to be 
 * in the y-direction) is greater than the force threshold then ground contact is
 * occurring. Note that this process is related to estimating ground reactions from
 * experimental motion.
 * 
 * TODO: details around dynamic simulations...
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
 * @authors Aaron Fox
 * @version 1.0
 */

//=============================================================================
//  CONSTRUCTORS
//=============================================================================

/** Default constructor */
ZmpGroundReactions::ZmpGroundReactions() {
    
    // Construct with default properties
    constructProperties();

}

/** Constructor specifying free joint name with other defaults. */
ZmpGroundReactions::ZmpGroundReactions(
        const std::string& freeJointName) {

    // Construct with default properties
    constructProperties();

    // Set the free joint name
    set_free_joint_name(freeJointName);

}

/** Constructor specifying free joint name and force threshold. */
ZmpGroundReactions::ZmpGroundReactions(
    const std::string& freeJointName, const double& forceThreshold) {

    // Construct with default properties
    constructProperties();

    // Set the free joint name
    set_free_joint_name(freeJointName);

    // Set the force threshold
    set_force_threshold(forceThreshold);

}

/** Construct default properties */
void ZmpGroundReactions::constructProperties() {

    // Contact body set
    ContactPointSet contactPointSet;
    contactPointSet.setName(
            IO::Lowercase(contactPointSet.getConcreteClassName()));
    constructProperty_ContactPointSet(contactPointSet);

    // Standard properties
    constructProperty_free_joint_name("ground_pelvis");
    constructProperty_force_threshold(20.0);
}

/** Finalize properties */
void ZmpGroundReactions::extendFinalizeFromProperties() {}

/** Connection to model */
void ZmpGroundReactions::extendConnectToModel(Model& model) {
    
    // Base class first
    Super::extendConnectToModel(model);

}

/** Topology and creating contact body map */
void ZmpGroundReactions::extendRealizeTopology(
        SimTK::State& state) const {
    
    // Base class first
    Super::extendRealizeTopology(state);

    //// Clear contact body map variable
    //m_contactBodyIndices.clear();

    //// Assign indices in map
    //for (int ii = 0; ii < get_ZeroMomentPointContactBodySet().getSize(); ++ii) {
    //    const ZeroMomentPointContactBody& contactBody = get_ZeroMomentPointContactBodySet().get(ii);
    //    m_contactBodyIndices[contactBody.getName()] = ii;        
    //}

}

/** Add component to system */
void ZmpGroundReactions::extendAddToSystem(
        SimTK::MultibodySystem& system) const {

    // Base class first
    Super::extendAddToSystem(system);

    // Set cache variables
    /*this->_groundReactionsCV = addCacheVariable("ground_reactions", 
        SimTK::Vector((int)m_contactBodyIndices.size() * 9, 0.0),
        SimTK::Stage::Dynamics);*/

}

//=============================================================================
//  METHODS
//=============================================================================

/** Add a contact point for reviewing ground contact with default
distance and velocity parameters */
void ZmpGroundReactions::addContactPoint(
    const std::string& pointName,
    const PhysicalFrame& body,
    const SimTK::Vec3& pointLocation) {

    // Create the contact point with the specified properties
    ContactPoint* cp = new ContactPoint(pointName, body, pointLocation);

    // Append to the contact point set
    updContactPointSet().adoptAndAppend(cp);

}

/** Add a contact point for reviewing ground contact with specified
distance and default velocity parameter */
void ZmpGroundReactions::addContactPoint(
    const std::string& pointName,
    const PhysicalFrame& body,
    const SimTK::Vec3& pointLocation,
    const double& distanceThreshold) {

    // Create the contact point with the specified properties
    ContactPoint* cp = new ContactPoint(pointName, body, pointLocation);

    // Set the distance threshold
    cp->setDistanceThreshold(distanceThreshold);

    // Append to the contact point set
    updContactPointSet().adoptAndAppend(cp);

}

/** Add a contact point for reviewing ground contact with specified
distance and velocity parameters */
void ZmpGroundReactions::addContactPoint(
    const std::string& pointName,
    const PhysicalFrame& body,
    const SimTK::Vec3& pointLocation,
    const double& distanceThreshold,
    const double& velocityThreshold) {

    // Create the contact point with the specified properties
    ContactPoint* cp = new ContactPoint(
        pointName,
        body,
        pointLocation,
        distanceThreshold,
        velocityThreshold);

    // Append to the contact point set
    updContactPointSet().adoptAndAppend(cp);

}

/** Add a contact point for reviewing ground contact with specified
distance and velocity parameters, and contact checking method */
void ZmpGroundReactions::addContactPoint(
    const std::string& pointName,
    const PhysicalFrame& body, 
    const SimTK::Vec3& pointLocation,
    const double& distanceThreshold, 
    const double& velocityThreshold,
    const std::string& contactCheckingMethod) {

    // Create the contact point with the specified properties
    ContactPoint* cp = new ContactPoint(
        pointName, 
        body, 
        pointLocation,
        distanceThreshold, 
        velocityThreshold,
        contactCheckingMethod);

    // Append to the contact point set
    updContactPointSet().adoptAndAppend(cp);

}


/** Set the free joint name in the component. */
void ZmpGroundReactions::setFreeJointName(
        const std::string& free_joint_name) {
    set_free_joint_name(free_joint_name);
};

/** Get and set the force threshold for considering when ground contact
has occurred. */
void ZmpGroundReactions::setForceThreshold(
        const double& force_threshold) {
    set_force_threshold(force_threshold);
}

/** Get the set of contact bodies associated with contact points in the
component. */
std::vector<std::string> ZmpGroundReactions::getContactBodyNames() const {

    // Get the contact point set
    ContactPointSet cps = get_ContactPointSet();

    // Get the number of contact points in the set
    const int nCP = cps.getSize();

    // Set-up arrays for bodies and uniquely identified bodies
    std::vector<std::string> bodies;
    std::unordered_set<std::string> uniqueBodies;

    // Loop through contact points to get body frame names
    for (int iCP = 0; iCP < (nCP); iCP++) {

        // Get body name
        const std::string bodyName = cps.get(iCP).getParentFrameName();

        // The parent frame is given as an absolute path, so this needs
        // to be split by the separate components
        size_t pos = bodyName.find_last_of("/");
        std::string body =
            (pos != std::string::npos) ? bodyName.substr(pos + 1) : bodyName;

        // Append to bodies array if the body name can be successfully
        // added to unique bodies set. insert() returns a pair and 
        // .second is true if insertion succeeded
        if (uniqueBodies.insert(body).second) {
            bodies.push_back(body);
        }

    }

    return bodies;

}

/** Create a StatesTrajectory from a TimeSeriesTable. Used as a convenience
function when calculating ZMP estimates from a motion specified in a
TimeSeriesTable. This is an adaptation of the createFromStatesTable
function associated with StatesTrajectory. */
StatesTrajectory ZmpGroundReactions::convertMotionToStatesTrajectory(
        const TimeSeriesTable& table) const {

    // Assemble the required objects.
    // ==============================

    // Create an empty states trajectory
    StatesTrajectory states;

    // Make a copy of the model to generate states
    Model localModel = getModel();

    // Generate state to edit as looping through time
    SimTK::State state = localModel.initSystem();

    // Get the labels of the columns in the table
    const auto& tableLabels = table.getColumnLabels();
    int numDependentColumns = (int)table.getNumColumns();

    // Error checking.
    // ===============
    // TODO: Avoiding this as need better coverage of table inputs...?
    //// Angular quantities must be expressed in radians.
    //OPENSIM_THROW_IF(TableUtilities::isInDegrees(table), DataIsInDegrees);

    // If column labels aren't unique, it's unclear which column the user
    // wanted to use for the related state variable.
    TableUtilities::checkNonUniqueLabels(tableLabels);

    // Check if states are missing from the table
    const auto& modelStateNames = localModel.getStateVariableNames();
    std::vector<std::string> missingColumnNames;
    // Also, assemble the indices of the states that we will actually set in the
    // trajectory.
    std::map<int, int> statesToFillUp;
    for (int is = 0; is < modelStateNames.getSize(); ++is) {
        // getStateIndex() will check for pre-4.0 column names.
        const int stateIndex = TableUtilities::findStateLabelIndex(
                tableLabels, modelStateNames[is]);
        if (stateIndex == -1) {
            missingColumnNames.push_back(modelStateNames[is]);
        } else {
            statesToFillUp[stateIndex] = is;
        }
    }

    // Check if the Table has columns that are not states in the Model.
    if ((unsigned)numDependentColumns > statesToFillUp.size()) {
        std::vector<std::string> extraColumnNames;
        // We want the actual column names, not the state names; the two
        // might be different b/c the state names changed in v4.0.
        for (int ic = 1; ic < (int)tableLabels.size(); ++ic) {
            // Has this label been marked as a model state?
            if (statesToFillUp.count(ic) == 0) {
                extraColumnNames.push_back(tableLabels[ic]);
            }
        }
    }

    // Fill up trajectory.
    // ===================

    //// Reserve the memory we'll need to fit all the states.
    //states.m_states.reserve(table.getNumRows());

    // Working memory for state. Initialize so that missing columns end up as
    // NaN.
    SimTK::Vector statesValues(modelStateNames.getSize(), SimTK::NaN);

    // Initialize so that missing columns end up as NaN.
    state.updY().setToNaN();

    // Loop through all rows of the Storage.
    for (int itime = 0; itime < (int)table.getNumRows(); ++itime) {

        const auto& row = table.getRowAtIndex(itime);

        // Set the correct time in the state.
        state.setTime(table.getIndependentColumn()[itime]);

        // Fill up current State with the data for the current time.
        for (const auto& kv : statesToFillUp) {
            // 'first': index for Table; 'second': index for Model.
            statesValues[kv.second] = row[kv.first];
        }
        localModel.setStateVariableValues(state, statesValues);

        // Make a copy of the edited state and put it in the trajectory.
        states.append(state);
    }

    return states;

}

//=============================================================================
//  CALCULATIONS
//=============================================================================

/** Get ZMP estimates of ground reactions from a motion. This method
requires model with the ZmpGroundReactions component and the motion in the
form of a time-series table specifying coordinate values, speeds and/or
accelerations. No contact bodies are provided so all potential contact bodies
specified in the ContactPointSet are queried. At a minimum it requires all
coordinate values to be specified. If not all speeds or accelerations are 
provided then they are derived from the coordinate values. No time bounds are 
provided therefore the entire motion is queried. */
Storage ZmpGroundReactions::calcFromQ(
        const TimeSeriesTable& table) const {

    // Check the table
    // ===============

    // Get the labels of the columns in the table
    const auto& tableLabels = table.getColumnLabels();

    // If column labels aren't unique, it's unclear which column the user
    // wanted to use for the related state variable.
    TableUtilities::checkNonUniqueLabels(tableLabels);

    // Make a copy of the model to generate state variable names
    Model localModel = getModel();
    SimTK::State localState = localModel.initSystem();

    // Get model values from states
    const auto& modelStateNames = localModel.getStateVariableNames();
    std::vector<std::string> modelValueNames;
    for (int is = 0; is < modelStateNames.getSize(); ++is) {
        const auto& name = modelStateNames[is];
        auto leafpos = name.find("/value");
        if (leafpos != std::string::npos) { 
            modelValueNames.push_back(name);
        }
    }

    // Get column labels from table
    std::vector<std::string> col_names = table.getColumnLabels();

    // Get value indices and names from table
    // Only take these if they are in model value names
    std::vector<int> valueIndices;
    std::vector<std::string> valueNames;
    for (int i = 0; i < (int)col_names.size(); ++i) {
        const auto& name = col_names[i];
        auto leafpos = name.find("/value");
        auto inModel = std::find(
            modelValueNames.begin(), modelValueNames.end(), name);
        if ((leafpos != std::string::npos && inModel != modelValueNames.end())) {
            valueIndices.push_back(i);
            valueNames.push_back(name);
        }
    }

    // Check that table includes all model value names to set
    bool allValuesPresent = std::all_of(modelValueNames.begin(),
        modelValueNames.end(), [&](const std::string& s) {
            return std::find(valueNames.begin(), valueNames.end(), s) !=
                valueNames.end();
        });
    OPENSIM_THROW_IF((!allValuesPresent), Exception,
        "Not all model coordinate values are included in "
        "the provided table.");

    // Assemble the required objects
    // ==============================

    // Get the model
    const Model& model = getModel();

    // Get time from the table
    const auto& time = table.getIndependentColumn();
    SimTK::Vector m_time = SimTK::Vector((int)time.size(), time.data());
    const int numTimes = m_time.size();

    // Get the number of coordinates in model
    // TODO: needed?
    int numCoords = model.getNumCoordinates();

    // Get the number of values, speeds and accelerations in time series table
    // This will help determine whether accelerations are calculated from values or speeds
    int numValues = valueIndices.size();

    // Use the values to create speeds and accelerations
    // ==============================

    // TODO: up to here with edits but most of this code below should be good...

    // Create matrix of values from table
    SimTK::Matrix values(table.getNumRows(), numValues);
    for (int i = 0; i < numValues; ++i) {
        values.updCol(i) = table.getMatrix().col(valueIndices[i]);
    }

    // Convert to time series table
    TimeSeriesTable values_table = TimeSeriesTable(
        std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
        values, valueNames);

    // Spline the values trajectory
    GCVSplineSet splines(values_table, {}, std::min(numTimes - 1, 5));

    // Set names for accelerations and speeds
    std::vector<std::string> speedNames;
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        std::string name(valueNames[ivalue]);
        auto leafpos = name.find("/value");
        name.replace(leafpos, name.size(), "/speed");
        accelerationNames.push_back(name);
    }
    std::vector<std::string> accelerationNames;
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        std::string name(valueNames[ivalue]);
        auto leafpos = name.find("/value");
        name.replace(leafpos, name.size(), "/accel");
        accelerationNames.push_back(name);
    }

    // Compute accelerations from values
    SimTK::Matrix accelerations(table.getNumRows(), numValues);
    SimTK::Vector currTime(1, SimTK::NaN);
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        const auto& name = valueNames[ivalue];
        // Compute the double derivative from the splined value and assign to this
        // speed memory.
        for (int itime = 0; itime < m_time.size(); ++itime) {
            currTime[0] = m_time[itime];
            accelerations(itime, ivalue) =
                    splines.get(name).calcDerivative({0, 0}, currTime);
        }
    }

    // Convert to time series table
    TimeSeriesTable udot_table = TimeSeriesTable(
            std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            accelerations, accelerationNames);

    // UP TO HERE: above code needs to be cleaned up but it is providing
    // the necessary udot table for calculations. It could probably be cleaned
    // up better to create relevant tables where possible so that they can be
    // inserted appropriately into the states trajectory (i.e. ensure all values,
    // speeds and accelerations are computed to be provided in the state). From
    // this, the state and udot at each time point can be used to get the ground
    // reactions


    // TODO: states trajectory may eventually be necessary, but...
    // better option here is to take in a time series table and check
    // or keep the values/speeds if present. If only values are present,
    // then use a similar method outlined in MocoTrajectory
    // generateAccelerationsFrom values, or if speeds are present then 
    // similarly just use generateAccelerationsFromSpeeds (i.e. get the 
    // number of values and speed columns and check that against model
    // coordinates). Once all this is done a udot table can be created
    // and similarly from all of this a states trajectory can also be
    // created to be used in an earlier function that gets ground reactions
    // from a motion when states and acceleration tables are provided...

    // TODO: essentially need to try and manually replicate the udot table
    // that can be provided by a MocoTrajectory generate function...

    // Create a states trajectory ...
    // TODO: get from convenience function

    // Return the contact body names from the ZMP component
    const std::vector<std::string>& contactBodyNames = getContactBodyNames();

    // Identify number of contact bodies to be included and queried
    const int nCB = contactBodyNames.size();

    // Create the storage for the ZMP results
    Storage zmpResults;

    // Set the name in the ZMP storage
    zmpResults.setName("ZMP Estimated Ground Reactions");

    // Create the columns for storing ground reactions based on body names
    // Ordering is Fx, Fy, Fz, Px, Py, Pz, Mx, My, Mz
    Array<std::string> zmpLabels("time", 9 * nCB);
    for (int iCB = 0; iCB < nCB; iCB++) {
        zmpLabels.set(iCB * 9 + 1, contactBodyNames.at(iCB) + "_force_vx");
        zmpLabels.set(iCB * 9 + 2, contactBodyNames.at(iCB) + "_force_vy");
        zmpLabels.set(iCB * 9 + 3, contactBodyNames.at(iCB) + "_force_vz");
        zmpLabels.set(iCB * 9 + 4, contactBodyNames.at(iCB) + "_force_px");
        zmpLabels.set(iCB * 9 + 5, contactBodyNames.at(iCB) + "_force_py");
        zmpLabels.set(iCB * 9 + 6, contactBodyNames.at(iCB) + "_force_pz");
        zmpLabels.set(iCB * 9 + 7, contactBodyNames.at(iCB) + "_torque_x");
        zmpLabels.set(iCB * 9 + 8, contactBodyNames.at(iCB) + "_torque_y");
        zmpLabels.set(iCB * 9 + 9, contactBodyNames.at(iCB) + "_torque_z");
    }

    // Set the column labels in table
    zmpResults.setColumnLabels(zmpLabels);

    return zmpResults;
}

//Storage ZmpGroundReactions::calcFromU(const TimeSeriesTable& table) const {
//
//    // Get the model
//    const Model& model = getModel();
//
//    // Get time from the table
//    const auto& time = table.getIndependentColumn();
//    SimTK::Vector m_time = SimTK::Vector((int)time.size(), time.data());
//    const int numTimes = m_time.size();
//
//    // Get the number of coordinates in model
//    int numCoords = model.getNumCoordinates();
//
//    // Get column labels from table
//    std::vector<std::string> col_names = table.getColumnLabels();
//
//    // Get value indices and names from table
//    std::vector<int> valueIndices;
//    std::vector<std::string> valueNames;
//    for (int i = 0; i < (int)col_names.size(); ++i) {
//        const auto& name = col_names[i];
//        auto leafpos = name.find("/value");
//        if (leafpos != std::string::npos) {
//            valueIndices.push_back(i);
//            valueNames.push_back(name);
//        }
//    }
//
//    //// Get speed indices and names from table
//    // std::vector<int> speedIndices;
//    // std::vector<std::string> speedNames;
//    // for (int i = 0; i < (int)col_names.size(); ++i) {
//    //     const auto& name = col_names[i];
//    //     auto leafpos = name.find("/speed");
//    //     if (leafpos != std::string::npos) {
//    //         speedIndices.push_back(i);
//    //         speedNames.push_back(name);
//    //     }
//    // }
//
//    //// Get acceleration indices and names from table
//    // std::vector<int> accelIndices;
//    // std::vector<std::string> accelNames;
//    // for (int i = 0; i < (int)col_names.size(); ++i) {
//    //     const auto& name = col_names[i];
//    //     auto leafpos = name.find("/accel");
//    //     if (leafpos != std::string::npos) {
//    //         speedIndices.push_back(i);
//    //         speedNames.push_back(name);
//    //     }
//    // }
//
//    // Get the number of values, speeds and accelerations in time series table
//    // This will help determine whether accelerations are calculated from values
//    // or speeds
//    int numValues = valueIndices.size();
//    /*int numSpeeds = speedIndices.size();
//    int numAccels = accelIndices.size();*/
//
//    // Check that the value names provided in table match coordinates in model
//
//    // Check for enough values or speeds
//    OPENSIM_THROW_IF((numValues < numCoords && numSpeeds < numCoords),
//            Exception,
//            "Not enough values or speeds exist in trajectory to "
//            "match model coordinates.");
//
//    // If enough speeds are available generate accelerations from speeds
//    if (numSpeeds >= numCoords) {
//
//        // Create matrix of speeds from table
//        SimTK::Matrix speeds(table.getNumRows(), numSpeeds);
//        for (int i = 0; i < numSpeeds; ++i) {
//            speeds.updCol(i) = table.getMatrix().col(speedIndices[i]);
//        }
//
//        // Convert to time series table
//        TimeSeriesTable speeds_table = TimeSeriesTable(
//                std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
//                speeds, speedNames);
//
//        // Spline the values trajectory
//        GCVSplineSet splines(speeds_table, {}, std::min(numTimes - 1, 5));
//
//        // Set names for accelerations
//        std::vector<std::string> accelerationNames;
//        for (int ivalue = 0; ivalue < numSpeeds; ++ivalue) {
//            std::string name(speedNames[ivalue]);
//            auto leafpos = name.find("/speed");
//            name.replace(leafpos, name.size(), "/accel");
//            accelerationNames.push_back(name);
//        }
//
//        // Compute accelerations from speeds
//        SimTK::Matrix accelerations(table.getNumRows(), numSpeeds);
//        SimTK::Vector currTime(1, SimTK::NaN);
//        for (int ivalue = 0; ivalue < numSpeeds; ++ivalue) {
//            const auto& name = speedNames[ivalue];
//            // Compute the derivative from the splined value and assign to this
//            // speed memory.
//            for (int itime = 0; itime < m_time.size(); ++itime) {
//                currTime[0] = m_time[itime];
//                accelerations(itime, ivalue) =
//                        splines.get(name).calcDerivative({0}, currTime);
//            }
//        }
//
//        // Convert to time series table
//        TimeSeriesTable udot_table = TimeSeriesTable(
//                std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
//                accelerations, accelerationNames);
//
//        // Otherwise use the values to create speeds and accelerations
//    } else {
//
//        // Create matrix of values from table
//        SimTK::Matrix values(table.getNumRows(), numValues);
//        for (int i = 0; i < numValues; ++i) {
//            values.updCol(i) = table.getMatrix().col(valueIndices[i]);
//        }
//
//        // Convert to time series table
//        TimeSeriesTable values_table = TimeSeriesTable(
//                std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
//                values, valueNames);
//
//        // Spline the values trajectory
//        GCVSplineSet splines(values_table, {}, std::min(numTimes - 1, 5));
//
//        // Set names for accelerations
//        std::vector<std::string> accelerationNames;
//        for (int ivalue = 0; ivalue < numValues; ++ivalue) {
//            std::string name(valueNames[ivalue]);
//            auto leafpos = name.find("/value");
//            name.replace(leafpos, name.size(), "/accel");
//            accelerationNames.push_back(name);
//        }
//
//        // Compute accelerations from values
//        SimTK::Matrix accelerations(table.getNumRows(), numValues);
//        SimTK::Vector currTime(1, SimTK::NaN);
//        for (int ivalue = 0; ivalue < numValues; ++ivalue) {
//            const auto& name = valueNames[ivalue];
//            // Compute the double derivative from the splined value and assign
//            // to this speed memory.
//            for (int itime = 0; itime < m_time.size(); ++itime) {
//                currTime[0] = m_time[itime];
//                accelerations(itime, ivalue) =
//                        splines.get(name).calcDerivative({0, 0}, currTime);
//            }
//        }
//
//        // Convert to time series table
//        TimeSeriesTable udot_table = TimeSeriesTable(
//                std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
//                accelerations, accelerationNames);
//    }
//
//    // UP TO HERE: above code needs to be cleaned up but it is providing
//    // the necessary udot table for calculations. It could probably be cleaned
//    // up better to create relevant tables where possible so that they can be
//    // inserted appropriately into the states trajectory (i.e. ensure all
//    // values, speeds and accelerations are computed to be provided in the
//    // state). From this, the state and udot at each time point can be used to
//    // get the ground reactions
//
//    // TODO: states trajectory may eventually be necessary, but...
//    // better option here is to take in a time series table and check
//    // or keep the values/speeds if present. If only values are present,
//    // then use a similar method outlined in MocoTrajectory
//    // generateAccelerationsFrom values, or if speeds are present then
//    // similarly just use generateAccelerationsFromSpeeds (i.e. get the
//    // number of values and speed columns and check that against model
//    // coordinates). Once all this is done a udot table can be created
//    // and similarly from all of this a states trajectory can also be
//    // created to be used in an earlier function that gets ground reactions
//    // from a motion when states and acceleration tables are provided...
//
//    // TODO: essentially need to try and manually replicate the udot table
//    // that can be provided by a MocoTrajectory generate function...
//
//    // Create a states trajectory ...
//    // TODO: get from convenience function
//
//    // Return the contact body names from the ZMP component
//    const std::vector<std::string>& contactBodyNames = getContactBodyNames();
//
//    // Identify number of contact bodies to be included and queried
//    const int nCB = contactBodyNames.size();
//
//    // Create the storage for the ZMP results
//    Storage zmpResults;
//
//    // Set the name in the ZMP storage
//    zmpResults.setName("ZMP Estimated Ground Reactions");
//
//    // Create the columns for storing ground reactions based on body names
//    // Ordering is Fx, Fy, Fz, Px, Py, Pz, Mx, My, Mz
//    Array<std::string> zmpLabels("time", 9 * nCB);
//    for (int iCB = 0; iCB < nCB; iCB++) {
//        zmpLabels.set(iCB * 9 + 1, contactBodyNames.at(iCB) + "_force_vx");
//        zmpLabels.set(iCB * 9 + 2, contactBodyNames.at(iCB) + "_force_vy");
//        zmpLabels.set(iCB * 9 + 3, contactBodyNames.at(iCB) + "_force_vz");
//        zmpLabels.set(iCB * 9 + 4, contactBodyNames.at(iCB) + "_force_px");
//        zmpLabels.set(iCB * 9 + 5, contactBodyNames.at(iCB) + "_force_py");
//        zmpLabels.set(iCB * 9 + 6, contactBodyNames.at(iCB) + "_force_pz");
//        zmpLabels.set(iCB * 9 + 7, contactBodyNames.at(iCB) + "_torque_x");
//        zmpLabels.set(iCB * 9 + 8, contactBodyNames.at(iCB) + "_torque_y");
//        zmpLabels.set(iCB * 9 + 9, contactBodyNames.at(iCB) + "_torque_z");
//    }
//
//    // Set the column labels in table
//    zmpResults.setColumnLabels(zmpLabels);
//
//    return zmpResults;
//}

///** Check ground contact of each contact body specified. Returns a vector
//of 1 or 0 for each contact body specifying if contact has or has not
//occurred. */
//SimTK::Vector ZeroMomentPointGroundReactions::checkGroundContact(
//        const SimTK::State& s) const {
//
//    /* This function takes the state and checks if the contact points allocated
//    to a body are in contact with the ground based on a distance and velocity
//    threshold. If any points meet the criteria then the body is said to be in
//    contact with the ground. However, if useVelocity is set to FALSE then only
//    the distance threshold is considered (i.e. velocity ignored). 
//    
//    This approach reflects that outlined in Karcnik (2003):
//    https://doi.org/10.1007/BF02345310 */
//
//    // Get the model
//    const Model& model = getModel();
//
//    // Temp return while function isn't being used
//    SimTK::Vector bodyInContact = SimTK::Vector(1);
//    bodyInContact.set(0, FALSE);
//
//    //// Realize to acceleration stage
//    //model.getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
//
//    //// Get the number of contact bodies
//    //const int nCB = get_ZeroMomentPointContactBodySet().getSize();
//
//    //// Create a vector to store whether the array is vs. isn't in ground contact
//    //SimTK::Vector bodyInContact = SimTK::Vector(nCB);
//
//    //// Loop through contact bodies and check for contact
//    //for (int iCB = 0; iCB < (nCB); iCB++) {
//
//    //    // Get the contact checking method for the current body
//    //    std::string checkingMethod = get_ZeroMomentPointContactBodySet()
//    //                                         .get(iCB)
//    //                                         .get_zmp_contact_checking_method();
//
//    //    // Get the number of contact body points linked to this contact body
//    //    int nCBP = get_ZeroMomentPointContactBodySet()
//    //                       .get(iCB).get_ContactPointSet().getSize();
//
//    //    // Create a vector to store if points are in contact
//    //    SimTK::Vector pointInContact = SimTK::Vector(nCBP);
//
//    //    // Loop through contact body points for the current contact body
//    //    for (int iCBP = 0; iCBP < (nCBP); iCBP++) {
//
//    //        // Get the body name, location and thresholds for the
//    //        // current contact point
//    //        ContactPoint& contactPoint =
//    //                get_ZeroMomentPointContactBodySet()
//    //                        .get(iCB)
//    //                        .get_ContactPointSet()
//    //                        .get(iCBP);
//    //        std::string bodyName = contactPoint.getParentFrame().getName();
//    //        SimTK::Vec3 pointLoc = contactPoint.get_location();
//    //        double distanceThreshold = contactPoint.get_distance_threshold();
//    //        double velocityThreshold = contactPoint.get_velocity_threshold();
//
//    //        // Check if point is in contact based on proposed checking method
//    //        if (checkingMethod == "velocity") {
//
//    //            std::cout << "TODO: distance and velocity contact checking"
//    //                      << std::endl;
//
//    //        } else {
//
//    //            // Calculate the position of the current contact point in the
//    //            // ground and it's vertical level (y-axis). Check if this
//    //            // vertical level is within the the specified distance
//    //            // threshold.
//    //            if (model.getBodySet()
//    //                            .get(bodyName)
//    //                            .findStationLocationInGround(s, pointLoc)
//    //                            .get(1) < distanceThreshold) {
//
//    //                // Specify that the point is in contact
//    //                pointInContact.set(iCBP, 1);
//
//    //            } else {
//
//    //                // Specify that the point is not in contact
//    //                pointInContact.set(iCBP, 0);
//
//    //            }
//    //        }
//    //    }
//
//    //    // If any of the points are in contact then the body can be set as
//    //    // in contact
//    //    if (pointInContact.sum() > 0) {
//
//    //        // At least one of the points is in contact with the ground
//    //        bodyInContact.set(iCB, TRUE);
//
//    //    } else {
//
//    //        // The contact body point is not in contact with the ground
//    //        bodyInContact.set(iCB, FALSE);
//
//    //    }
//
//    //}
//
//    // Return the vector of if the bodies are in contact
//    return bodyInContact;
//
//}

/** The below functions calculate the Zero Moment Point of the model based
on the Model state and any other provided inputs. It identifies the ground
reaction forces, moments and centre of pressure for each contact body
listed in the component. The output is returned as a Vector which size is
based on the number of contact bodies. Each component of the Vector contains
a spatial vector that contains the separate force, moment and point
components.

    i.e.

        FX, FY, FZ, PX, PY, PZ, MX, MY, MZ

which is repeated for the number of contact bodies in the component.*/

// TODO: consider using Vector of SpatialVec for output from ground reactions...

///** Calculate ground reactions from state. This is a simpler function that
//uses the state to get udot and then feeds back to the other function. */
//SimTK::Vector ZeroMomentPointGroundReactions::getGroundReactions(
//        const SimTK::State& s) const {
//
//    // Get the model
//    const auto& model = getModel();
//
//    // Realize to appropriate stage
//    model.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
//
//    // Get accelerations from state
//    // TODO: is this the right way to do it? Comes out as zeros sometimes?
//    SimTK::Vector udot = s.getUDot();
//
//    // Feed state and udot into detailed function to return ground reactions
//    SimTK::Vector groundReactions = getGroundReactions(s, udot);
//
//    return groundReactions;
//
//}

///** Calculate ground reactions with state and udot accelerations vector.*/
//SimTK::Vector ZeroMomentPointGroundReactions::getGroundReactions(
//        const SimTK::State& s, const SimTK::Vector& udot) const {
//
//    ///* TODO: mapping if q != u in index (i.e.tree vs.model)... */
//    ///     - Not sure it matters as calcEquivalentSpatial force seems to work...
//    ///* TODO: getting the MY calculation correct - currently staying as zero...
//    ///*/
//
//    // Get the model
//    const auto& model = getModel();
//
//    // Get the free body name and associated joint
//    const std::string freeJointName = get_free_joint_name();
//    const Joint& freeJoint = model.getJointSet().get(freeJointName);
//
//    // Get the number of contact bodies
//    const int nCB = get_ZeroMomentPointContactBodySet().getSize();
//
//    // Get the desired force threshold property
//    const double forceThreshold = get_force_threshold();
//
//    // Create the vector to fill with the calculated ground reactions
//    // Size is based on the number of contact bodies * 3x3 (F, M and P)
//    SimTK::Vector groundReactionsVec = SimTK::Vector(9 * nCB);
//
//    // Set all zeros as default in ground reactions
//    for (int iCol = 0; iCol < (9 * nCB); iCol++) {
//        groundReactionsVec.set(iCol, 0.0);
//    }
//
//    // Initialise an inverse dynamics solver with model
//    InverseDynamicsSolver ivdSolver(model);
//
//    // Realize to appropriate stage
//    model.getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
//
//    // Check the contact bodies for ground contact
//    SimTK::Vector inContact(nCB);
//    inContact = checkGroundContact(s);
//
//    // Determine whether contact has occurred
//    bool contactOccurring = FALSE;
//    if (inContact.sum() > 0) { contactOccurring = TRUE; }
//
//    // Proceed through subsequent calculations only if contact occurring
//    if (contactOccurring) {
//
//        // Solve inverse dynamics given current states and udot
//        // The output vector contains the generalised coordinate forces
//        // to generate the accelerations based on the current state.
//        // Note that these aren't necessarily in the order of the
//        // coordinate set, but rather the multibody tree order.
//        SimTK::Vector genForceTraj = ivdSolver.solve(s, udot);
//
//        // Calculate the equivalent body force at the free joint in the model
//        SimTK::SpatialVec equivalentBodyForceAtJoint =
//                freeJoint.calcEquivalentSpatialForce(s, genForceTraj);
//
//        // Extract the body and torque components in ground frame
//        SimTK::Vec3 freeBodyTorque = equivalentBodyForceAtJoint.get(0);
//        SimTK::Vec3 freeBodyForce = equivalentBodyForceAtJoint.get(1);
//
//        // Run a secondary check to see if free body vertical force (assumes
//        // y-axis) if greater than the desired force threshold
//        if (freeBodyForce.get(1) > forceThreshold) {
//
//            // Get the position of the free body in the ground frame
//            // This is based on getting the child frame of the free joint
//            // and hence assumes that the ground should always be the parent.
//            // Getting the frames position in the ground should represent it's
//            // translational coordinates, as it appears robust to frame
//            // translation in the model. The frames origin (i.e. 0,0,0) can be
//            // used as the station. An adaptation to this could be to use the
//            // centre of mass of the body frame to estimate of the ground
//            // reactions.
//            SimTK::Vec3 rp =
//                    freeJoint.getChildFrame().findStationLocationInGround(
//                            s, SimTK::Vec3(0, 0, 0));             
//
//            // Calculate moment at origin
//            // Formulas used here come from Xiang et al. 2009:
//            // https://doi.org/10.1002/nme.2575
//            SimTK::Vec3 rp_fbf = SimTK::cross(rp, freeBodyForce);
//            SimTK::Vec3 groundM =
//                    SimTK::Vec3(freeBodyTorque.get(0) + rp_fbf.get(0),
//                            freeBodyTorque.get(1) + rp_fbf.get(1),
//                            freeBodyTorque.get(2) + rp_fbf.get(2));
//
//            // Calculate X & Z cZMP, noting that yZMP is set as 0
//            // Given this (and some other calculations) the standard OpenSim
//            // coordinate system must be used.
//            // Formula used here come from Xiang et al. 2009:
//            // https://doi.org/10.1002/nme.2575
//            SimTK::Vec3 zmpCOP =
//                    SimTK::Vec3(groundM.get(2) / freeBodyForce.get(1), 0,
//                        -groundM.get(0) / freeBodyForce.get(1));
//
//            // Calculate the resultant active moment at ZMP along the y-axis
//            // TODO: This formula still doesn't seem quite right?
//            double myZMP = groundM.get(1) +
//                           (freeBodyForce.get(0) * zmpCOP.get(2)) -
//                           (freeBodyForce.get(2) * zmpCOP.get(1));
//
//            // Check for unilateral contact
//            if (inContact.sum() == 1) {
//
//                // If unilateral contact is flagged, then the force is simply
//                // allocated to the body that is deemed in contact with the
//                // ground.
//
//                // Figure out which body index is contacting the ground
//                int contactInd;
//                for (int bb = 0; bb < nCB; bb++) {
//                    if (inContact.get(bb) == 1) { contactInd = bb; }
//                }
//
//                // Set the values at the appropriate indices in the ground
//                // reactions vector Note the MX (index = 6*nCB) and MZ (index =
//                // 8 * nCB) remain set as zero
//                groundReactionsVec.set(
//                        contactInd * 9 + 0, freeBodyForce.get(0));
//                groundReactionsVec.set(
//                        contactInd * 9 + 1, freeBodyForce.get(1));
//                groundReactionsVec.set(
//                        contactInd * 9 + 2, freeBodyForce.get(2));
//                groundReactionsVec.set(contactInd * 9 + 3, zmpCOP.get(0));
//                groundReactionsVec.set(contactInd * 9 + 4, zmpCOP.get(1));
//                groundReactionsVec.set(contactInd * 9 + 5, zmpCOP.get(2));
//                /*groundReactionsVec.set(contactInd * 9 + 7, myZMP);*/
//
//            } else {
//
//                // TODO: dealing with bilateral contact
//                // There can probably only be a max of 2 contact bodies given
//                // method used
//                std::cout << "TODO: calculations with bilateral contact"
//                          << std::endl;
//            }
//        }
//    }
//
//    return groundReactionsVec;
//}

///** These functions calculate the Zero Moment Point of the model based
//on a series of states from a predefined motion. It identifies the ground
//reaction forces, moments and centre of pressure for each contact body
//listed in the component across the states provided. The output is returned
//as a Storage with the number of columns based on the number of contact
//bodies and the separate force, moment and point components.
//
//i.e.
//
//    FXn, FYn, FZn, PXn, PYn, PZn, MXn, MYn, MZn
//
//where n is repeated for the number of contact bodies specified.*/
//
///** Calculate ground reactions from a provided states trajectory and
//accelerations table. */
//Storage ZeroMomentPointGroundReactions::getGroundReactionsFromMotion(
//        const StatesTrajectory& states, const TimeSeriesTable& udot) const {
//
//    // Get the model
//    const Model& model = getModel();
//
//    // Create the table to store ZMP results in
//    Storage zmpResults;
//
//    // Set the name in the ZMP storage
//    zmpResults.setName("ZMP Estimated Ground Reactions");
//
//    // Define number of times from states trajectory
//    int nt = states.getSize();
//
//    // Get number of contact bodies in the component
//    const int nCB = get_ZeroMomentPointContactBodySet().getSize();
//
//    // Set the contact body names for labelling
//    Array<std::string> contactBodyNames;
//    for (int iCB = 0; iCB < nCB; iCB++) {
//        std::string labelBody = get_ZeroMomentPointContactBodySet()
//            .get(iCB)
//            .get_body_name();
//        contactBodyNames.append(labelBody);
//    }
//
//    // Create the columns for storing ground reactions based on body names
//    // Ordering is Fx, Fy, Fz, Px, Py, Pz, Mx, My, Mz
//    Array<std::string> zmpLabels("time", 9 * nCB);
//    for (int iCB = 0; iCB < nCB; iCB++) {
//        zmpLabels.set(iCB * 9 + 1, contactBodyNames.get(iCB) + "_force_vx");
//        zmpLabels.set(iCB * 9 + 2, contactBodyNames.get(iCB) + "_force_vy");
//        zmpLabels.set(iCB * 9 + 3, contactBodyNames.get(iCB) + "_force_vz");
//        zmpLabels.set(iCB * 9 + 4, contactBodyNames.get(iCB) + "_force_px");
//        zmpLabels.set(iCB * 9 + 5, contactBodyNames.get(iCB) + "_force_py");
//        zmpLabels.set(iCB * 9 + 6, contactBodyNames.get(iCB) + "_force_pz");
//        zmpLabels.set(iCB * 9 + 7, contactBodyNames.get(iCB) + "_torque_x");
//        zmpLabels.set(iCB * 9 + 8, contactBodyNames.get(iCB) + "_torque_y");
//        zmpLabels.set(iCB * 9 + 9, contactBodyNames.get(iCB) + "_torque_z");
//    }
//
//    // Set the column labels in table
//    zmpResults.setColumnLabels(zmpLabels);
//
//    // Loop through times to calculate ground reactions at each state
//    for (int i = 0; i < nt; i++) {
//
//        // Get the current state
//        SimTK::State s = states[i];
//
//        // Get number of coordinates from state
//        int nq = s.getNQ();
//
//        // Get accelerations from the Moco trajectory at the current state
//        SimTK::Vector s_udot(nq);
//        for (int k = 0; k < nq; k++) {
//            s_udot.set(k, udot.getRowAtIndex(i).getAnyElt(0, k));
//        }
//
//        // Calculate ground reactions from state and udot via convenience function
//        SimTK::Vector groundReactions = getGroundReactions(s, s_udot);
//
//        // Create a state vector with the time and ZMP values
//        StateVector zmpStateVec = StateVector(s.getTime(), groundReactions);
//
//        // Append the state vector to the storage object
//        zmpResults.append(zmpStateVec);
//    
//    }
//
//    return zmpResults;
//
//}

//=============================================================================
//  OUTPUTS
//=============================================================================

/** The below function allows getting the ground reactions from the component as
a model output. The output takes the form of SimTK::Vectors that specifies the
forces, points and moments in the same way as the calculation functions:

i.e.

    FXn, FYn, FZn, PXn, PYn, PZn, MXn, MYn, MZn

where n is repeated for the number of contact bodies specified.*/

///** Get the forces estimated for the specified contact body. */
//SimTK::Vec3 ZeroMomentPointGroundReactions::getContactBodyForces(
//    const SimTK::State& s, const std::string& bodyName) const 
//{
//
//    /*if (isCacheVariableValid(s, _groundReactionsCV)) {
//        return getCacheVariableValue(s, _groundReactionsCV);
//    }*/
//
//    
//
//    //// Run the calculation function with the input state
//    //SimTK::Vector groundReactionsOut = getGroundReactions(s);
//
//    // Get the contact body index for the specified body object
//    const int& bodyInd = m_contactBodyIndices.at(bodyName);
//
//    // TODO: just default values at the moment
//    SimTK::Vec3 forces = SimTK::Vec3(0);
//
//    return forces;
//
//}
//
///** Get the moments estimated for the specified contact body. */
//SimTK::Vec3 ZeroMomentPointGroundReactions::getContactBodyMoments(
//        const SimTK::State& s, const std::string& bodyName) const {
//
//    //// Run the calculation function with the input state
//    // SimTK::Vector groundReactionsOut = getGroundReactions(s);
//
//    // TODO: just default values at the moment
//    SimTK::Vec3 moments = SimTK::Vec3(0);
//
//    return moments;
//}
//
///** Get the point of application estimated for the specified contact body. */
//SimTK::Vec3 ZeroMomentPointGroundReactions::getContactBodyPoint(
//        const SimTK::State& s, const std::string& bodyName) const {
//
//    //// Run the calculation function with the input state
//    // SimTK::Vector groundReactionsOut = getGroundReactions(s);
//
//    // TODO: just default values at the moment
//    SimTK::Vec3 point = SimTK::Vec3(0);
//
//    return point;
//}