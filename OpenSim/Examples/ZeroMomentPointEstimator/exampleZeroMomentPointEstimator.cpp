/* -------------------------------------------------------------------------- *
 *               OpenSim:  exampleZeroMomentPointEstimator.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2026 Stanford University and the Authors                     *
 * Author(s): Aaron Fox                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// This example demonstrates how to estimate ground reaction forces (GRFs),
// centers of pressure (CoPs) and free moments from model kinematics with the
// Zero Moment Point (ZMP) method, using the ZeroMomentPointEstimator,
// ContactSide and ContactPoint components and the ZMP utility functions.
//
// The example:
//   1. loads a running model;
//   2. creates a ZeroMomentPointEstimator;
//   3. builds a ContactSide for each foot, with ContactPoints at the foot
//      markers, using contact hysteresis and a minimum contact duration;
//   4. adds the estimator to the model and prints the model;
//   5. queries the estimator's properties, contact sides and outputs;
//   6. estimates the GRFs from a Moco tracking solution, using the solution's
//      states (values and speeds) and, for comparison, its coordinate values
//      only;
//   7. creates ExternalLoads that apply the estimated GRFs to the feet; and
//   8. compares the estimated GRFs with the measured GRFs.
//
// See README.md for details about the method, the data and the output files.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <OpenSim/OpenSim.h>

using namespace OpenSim;

namespace {

void printSection(const std::string& title) {
    std::cout << "\n"
              << std::string(72, '=') << "\n"
              << "  " << title << "\n"
              << std::string(72, '=') << std::endl;
}

// Linear interpolation of a column of `table` at `time`.
double interpolate(
        const TimeSeriesTable& table, const std::string& label, double time) {
    const auto& times = table.getIndependentColumn();
    const auto column = table.getDependentColumn(label);
    const auto upper = std::lower_bound(times.begin(), times.end(), time);
    const int i = std::clamp(static_cast<int>(upper - times.begin()), 1,
            static_cast<int>(times.size()) - 1);
    const double weight = (time - times[i - 1]) / (times[i] - times[i - 1]);
    return column[i - 1] + weight * (column[i] - column[i - 1]);
}

} // namespace

int main() {

    // Input files (copied next to the executable).
    const std::string modelFile = "subject01_run3.osim";
    const std::string solutionFile = "subject01_run3_trackingSolution.sto";
    const std::string measuredGRFFile = "Run_3_grf.mot";

    //==========================================================================
    // Step 1. Load the model.
    //==========================================================================
    printSection("Step 1. Load the model");

    Model model(modelFile);
    // Connect the model so that the markers' frames can be used below.
    model.finalizeConnections();
    std::cout << "Loaded model '" << model.getName()
              << "': " << model.getNumBodies() << " bodies, "
              << model.getNumJoints() << " joints, "
              << model.getNumCoordinates() << " coordinates, "
              << model.getMarkerSet().getSize() << " markers." << std::endl;

    //==========================================================================
    // Step 2. Create the ZeroMomentPointEstimator.
    //==========================================================================
    printSection("Step 2. Create the ZeroMomentPointEstimator");

    // The estimator extracts the inverse dynamics residual at the joint that
    // connects the model to Ground (here, "ground_pelvis"). The defaults are
    // kept for the other properties: outputs are valid only when the vertical
    // force is at least 20 N (contact_force_threshold), and the ground is the
    // horizontal plane y = 0 (ground_height), as in the measured data.
    auto* zmp = new ZeroMomentPointEstimator("ground_pelvis");
    zmp->setName("zmp_estimator");

    std::cout << "Created ZeroMomentPointEstimator '" << zmp->getName()
              << "':\n"
              << "  free joint:              " << zmp->getFreeJointName()
              << "\n"
              << "  contact force threshold: "
              << zmp->getContactForceThreshold() << " N\n"
              << "  ground height:           " << zmp->getGroundHeight() << " m"
              << std::endl;

    //==========================================================================
    // Step 3. Build a ContactSide for each foot.
    //==========================================================================
    printSection("Step 3. Build the contact sides");

    // Each ContactSide groups the ContactPoints used to detect ground contact
    // for one foot, and names the body that its GRF is applied to (calcn_r or
    // calcn_l). ContactPoints are placed at the model's foot markers that sit
    // on the sole of the shoe (the "_ground" markers): heel (CAL), toe tip
    // (TOE), 5th metatarsal head (MT5), mid toe (TOE_mid), mid metatarsal
    // (MT5_mid) and distal end of the foot (_end).
    //
    // Contact detection settings for this trial:
    // - The 'distance' method is used. These data were collected on a
    //   treadmill, so the stance foot moves backward with the belt and the
    //   'velocity' method (Karcnik 2003) would not detect contact.
    // - A point comes into contact when it is within 2 cm of the ground
    //   (distance_threshold) and, with hysteresis, stays in contact until it
    //   is more than 4 cm above the ground (distance_off_threshold). The
    //   default threshold (0.8 mm, for calibrated markers) is too small for
    //   these marker-based points, which do not all reach the floor.
    // - Contact and no-contact periods shorter than 20 ms are removed
    //   (minimum_contact_duration). None occur in this trial, but this guards
    //   against brief, spurious contacts in noisier data.
    // Hysteresis and the minimum duration are applied by
    // createZMPLoadsFromStates() (Step 6), which processes frames in order.
    const std::vector<std::string> markerNames{
            "CAL", "TOE", "MT5", "TOE_mid", "MT5_mid", "_end"};
    for (const auto& [sideName, prefix, bodyName] :
            std::vector<std::tuple<std::string, std::string, std::string>>{
                    {"right", "R", "calcn_r"}, {"left", "L", "calcn_l"}}) {
        const auto& body = model.getBodySet().get(bodyName);
        auto* side = new ContactSide(sideName, body);
        side->setMinimumContactDuration(0.02);

        for (const auto& markerName : markerNames) {
            const Marker& marker =
                    model.getMarkerSet().get(prefix + markerName + "_ground");
            auto* point = new ContactPoint(prefix + markerName,
                    marker.getParentFrame(), marker.get_location());
            point->setContactCheckingMethod("distance");
            point->setDistanceThreshold(0.02);
            point->setDistanceOffThreshold(0.04);
            // The side takes ownership of the point.
            side->adoptContactPoint(point);
        }

        std::cout << "ContactSide '" << side->getName()
                  << "': " << side->getNumContactPoints()
                  << " contact points, GRF applied to '" << bodyName << "'."
                  << std::endl;

        // The estimator takes ownership of the side.
        zmp->adoptContactSide(side);
    }

    //==========================================================================
    // Step 4. Add the estimator to the model and print the model.
    //==========================================================================
    printSection("Step 4. Add the estimator to the model");

    model.addComponent(zmp);
    model.finalizeConnections();

    // The estimator, sides and points are saved with the model, so this file
    // can be loaded later without repeating Steps 2 and 3.
    const std::string outputModelFile = "subject01_run3_with_zmp.osim";
    model.print(outputModelFile);
    std::cout << "Model with the estimator printed to '" << outputModelFile
              << "'." << std::endl;

    //==========================================================================
    // Step 5. Query the estimator.
    //==========================================================================
    printSection("Step 5. Query the estimator");

    model.initSystem();
    const auto& estimator =
            model.getComponent<ZeroMomentPointEstimator>("/zmp_estimator");

    std::cout << "Number of contact sides: " << estimator.getNumContactSides()
              << std::endl;
    for (int i = 0; i < estimator.getNumContactSides(); ++i) {
        const ContactSide& side = estimator.getContactSide(i);
        std::cout << "\nSide '" << side.getName() << "' (contact body "
                  << side.getContactBody().getName()
                  << ", minimum contact duration "
                  << side.getMinimumContactDuration() << " s):" << std::endl;
        for (int j = 0; j < side.getNumContactPoints(); ++j) {
            const ContactPoint& point = side.getContactPoint(j);
            std::cout << "  " << point.getName() << ": location "
                      << point.get_location() << " in "
                      << point.getParentFrame().getName() << ", method "
                      << point.getContactCheckingMethodName() << ", on/off "
                      << point.getDistanceThreshold() << "/"
                      << point.getDistanceOffThreshold() << " m"
                      << (point.usesHysteresis() ? " (hysteresis)" : "")
                      << std::endl;
        }
    }

    // Sides can also be accessed by name.
    const ContactSide& right = estimator.getContactSide("right");
    std::cout << "\nFound side 'right' by name: " << right.getNumContactPoints()
              << " contact points." << std::endl;

    // List outputs have one channel per side, named after the side.
    std::cout << "\nEstimator outputs:" << std::endl;
    for (const auto& name : estimator.getOutputNames()) {
        const AbstractOutput& output = estimator.getOutput(name);
        std::cout << "  " << name << " (" << output.getTypeName() << ")"
                  << (output.isListOutput() ? ", one channel per side" : "")
                  << std::endl;
    }

    //==========================================================================
    // Step 6. Estimate the GRFs from the tracking solution.
    //==========================================================================
    printSection("Step 6. Estimate the GRFs");

    // createZMPLoadsFromStates() chooses how to obtain the accelerations from
    // the columns in the table (see README.md):
    // - The tracking solution contains coordinate values and speeds, so the
    //   values and speeds are used as given, and the accelerations are the
    //   derivatives of splines of the speeds. Its control columns are
    //   ignored.
    // - With coordinate values only (e.g., Inverse Kinematics results), the
    //   values are fit with splines and prescribed with a PositionMotion, so
    //   the speeds and accelerations are their first and second derivatives.
    // The solution is smooth, so no filtering is applied. For measured
    // kinematics, pass a low-pass cutoff frequency (Hz) as the last argument,
    // e.g., createZMPLoadsFromStates(model, states, "", 6.0).
    TimeSeriesTable states(solutionFile);
    std::cout << "\nFrom the states (values and speeds):" << std::endl;
    const TimeSeriesTable grfFromStates =
            createZMPLoadsFromStates(model, states);

    TimeSeriesTable values = states;
    for (const auto& label : states.getColumnLabels()) {
        if (label.find("/value") == std::string::npos) {
            values.removeColumn(label);
        }
    }
    std::cout << "\nFrom the coordinate values only:" << std::endl;
    const TimeSeriesTable grfFromValues =
            createZMPLoadsFromStates(model, values);

    std::cout << "\nEstimated GRF table: " << grfFromStates.getNumRows()
              << " rows, columns:" << std::endl;
    for (const auto& label : grfFromStates.getColumnLabels()) {
        std::cout << "  " << label << std::endl;
    }

    const std::string grfFromStatesFile = "subject01_run3_zmp_grf.mot";
    const std::string grfFromValuesFile =
            "subject01_run3_zmp_grf_fromValues.mot";
    STOFileAdapter::write(grfFromStates, grfFromStatesFile);
    STOFileAdapter::write(grfFromValues, grfFromValuesFile);
    std::cout << "\nEstimated GRFs written to '" << grfFromStatesFile
              << "' and '" << grfFromValuesFile << "'." << std::endl;

    //==========================================================================
    // Step 7. Create ExternalLoads that apply the estimated GRFs.
    //==========================================================================
    printSection("Step 7. Create ExternalLoads");

    // One ExternalForce per side, applied to the side's contact body, reading
    // the columns of the GRF file written above. The ExternalLoads file can be
    // used like a force plate ExternalLoads file (e.g., with the Inverse
    // Dynamics tool, or ModOpAddExternalLoads in Moco).
    const ExternalLoads externalLoads =
            createZMPExternalLoads(model, grfFromStatesFile);
    const std::string externalLoadsFile = "subject01_run3_zmp_grf.xml";
    externalLoads.print(externalLoadsFile);
    for (int i = 0; i < externalLoads.getSize(); ++i) {
        const ExternalForce& force = externalLoads.get(i);
        std::cout << "ExternalForce '" << force.getName() << "': applied to '"
                  << force.get_applied_to_body() << "', force '"
                  << force.get_force_identifier() << "', point '"
                  << force.get_point_identifier() << "', torque '"
                  << force.get_torque_identifier() << "'." << std::endl;
    }
    std::cout << "ExternalLoads printed to '" << externalLoadsFile << "'."
              << std::endl;

    //==========================================================================
    // Step 8. Compare with the measured GRFs.
    //==========================================================================
    printSection("Step 8. Compare with the measured GRFs");

    // Root-mean-square differences over the frames where both the estimate
    // and the force plate (vertical force > 20 N) report a GRF.
    const TimeSeriesTable measured(measuredGRFFile);
    const auto& times = grfFromStates.getIndependentColumn();
    for (const auto& [side, plate] :
            std::vector<std::pair<std::string, std::string>>{
                    {"right", "R_ground"}, {"left", "L_ground"}}) {
        for (const auto& [name, estimate] :
                std::vector<std::pair<std::string, const TimeSeriesTable*>>{
                        {"states", &grfFromStates},
                        {"values only", &grfFromValues}}) {
            double sumSquaredVertical = 0;
            double sumSquaredFore = 0;
            double peakVertical = 0;
            int frames = 0;
            int agreement = 0;
            for (int i = 0; i < static_cast<int>(times.size()); ++i) {
                const double estimatedY =
                        estimate->getDependentColumn(side + "_force_vy")[i];
                const double estimatedX =
                        estimate->getDependentColumn(side + "_force_vx")[i];
                const double measuredY =
                        interpolate(measured, plate + "_force_vy", times[i]);
                const double measuredX =
                        interpolate(measured, plate + "_force_vx", times[i]);
                const bool estimatedOn = estimatedY > 0;
                const bool measuredOn = measuredY > 20;
                if (estimatedOn == measuredOn) ++agreement;
                if (estimatedOn && measuredOn) {
                    sumSquaredVertical += std::pow(estimatedY - measuredY, 2);
                    sumSquaredFore += std::pow(estimatedX - measuredX, 2);
                    peakVertical = std::max(peakVertical, measuredY);
                    ++frames;
                }
            }
            std::cout << side << " (" << name
                      << "): contact agrees with the force plate in "
                      << agreement << " of " << times.size() << " frames;";
            if (frames > 0) {
                std::cout << " RMS difference vertical "
                          << std::sqrt(sumSquaredVertical / frames)
                          << " N (peak measured " << peakVertical
                          << " N), fore-aft "
                          << std::sqrt(sumSquaredFore / frames) << " N.";
            }
            std::cout << std::endl;
        }
    }

    //==========================================================================
    // Done.
    //==========================================================================
    printSection("Example complete");
    std::cout << "Output files:\n"
              << "  " << outputModelFile << "\n"
              << "  " << grfFromStatesFile << "\n"
              << "  " << grfFromValuesFile << "\n"
              << "  " << externalLoadsFile << "\n"
              << "Compare the estimated GRFs with the measured GRFs in '"
              << measuredGRFFile << "'." << std::endl;

    return EXIT_SUCCESS;
}
