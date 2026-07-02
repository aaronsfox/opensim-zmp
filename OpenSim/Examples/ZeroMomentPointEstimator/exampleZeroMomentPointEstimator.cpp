/* -------------------------------------------------------------------------- *
 *            OpenSim: exampleZeroMomentPointEstimator.cpp                    *
 * -------------------------------------------------------------------------- *
 * Demonstrates the use of the ZeroMomentPointEstimator component to          *
 * estimate ground reaction forces (GRFs) and centres of pressure (CoPs)      *
 * from model kinematics using the Zero Moment Point (ZMP) method.            *
 *                                                                            *
 * Please review the associated README.MD file for example specific details.  *
 *                                                                            *
 * @author  Aaron Fox                                                         *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/ContactPoint.h>
#include <OpenSim/Simulation/Model/ContactSide.h>
#include <OpenSim/Simulation/Model/ZeroMomentPointEstimator.h>
#include <OpenSim/Simulation/ZeroMomentPointUtilities.h>
#include <OpenSim/Moco/MocoTrajectory.h>

#include <iostream>
#include <string>

using namespace OpenSim;
using namespace SimTK;

// Helper: print a section header to console
static void printSection(const std::string& title) {
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "============================================================\n";
}

int main() {

    // ======================================================================
    // STEP 1: Load the model
    // ======================================================================
    printSection("Step 1. Loading model");

    const std::string modelFile    = "subject01_run3.osim";
    const std::string grfFile      = "Run_3_grf.mot";
    const std::string solutionFile = "subject01_run3_trackingSolution.sto";

    Model model(modelFile);
    std::cout << "Loaded model: " << model.getName() << "\n";
    std::cout << "  Bodies:      " << model.getBodySet().getSize()  << "\n";
    std::cout << "  Joints:      " << model.getJointSet().getSize() << "\n";
    std::cout << "  Markers:     " << model.getMarkerSet().getSize()<< "\n";

    // ======================================================================
    // STEP 2: Create the ZeroMomentPointEstimator
    // ======================================================================
    printSection("Step 2. Creating ZeroMomentPointEstimator");

    // The estimator needs the name of the free joint (root joint) that
    // connects the model to ground. In the Hamner model this is named
    // as "ground_pelvis". The contact force threshold is set to 20 N —
    // below this threshold any GRFs will be marked as zero.

    // Create the component with default properties
    auto* zmp = new ZeroMomentPointEstimator();

    // Set the overall name for the component
    zmp->setName("zmp_grf_estimator");

    // Set the free joint name
    zmp->setFreeJointName("ground_pelvis");

    // Set the contact force threshold
    zmp->setContactForceThreshold(50.0);

    // Print some outputs from the tool
    std::cout << "Created ZeroMomentPointEstimator:\n";
    std::cout << "  Name:                    " << zmp->getName()              << "\n";
    std::cout << "  Free joint:              " << zmp->getFreeJointName()     << "\n";
    std::cout << "  Contact force threshold: " << zmp->getContactForceThreshold() << " N\n";

    // ======================================================================
    // STEP 3: Build the contact sides
    // ======================================================================
    // Contact points are placed at the local frame positions
    // corresponding to the marker locations on the feet:
    //   RCAL      — calcaneus (heel)
    //   RTOE      — toe tip
    //   RMT5      — 5th metatarsal head
    //   RTOE_mid  — midpoint toe
    //   RMT5_mid  — midpoint metatarsal
    //   R_end     — distal end of foot
    //
    // All contact points use the distance checking method with a 0.08 m
    // threshold (point height <= 0.08 m → in contact).
    // The GRF for the right side will be applied to calcn_r.
    // ======================================================================
    printSection("Step 3. Building contact sides");

    // Get the bodies for which the contact sides will be set to
    const PhysicalFrame& calcn_r = model.getBodySet().get("calcn_r");
    const PhysicalFrame& calcn_l = model.getBodySet().get("calcn_l");

    // Create the contact sides
    auto* rightContactSide = new ContactSide("right_side", "calcn_r");
    auto* leftContactSide = new ContactSide("left_side", "calcn_l");

    // To get the contact point locations we look for the marker position
    // locations in the model. Here a list is set for the markers to look for.
    // Right side
    Array<std::string> contactPointsRight;
    contactPointsRight.append("RCAL");
    contactPointsRight.append("RTOE");
    contactPointsRight.append("RMT5");
    contactPointsRight.append("RTOE_mid");
    contactPointsRight.append("RMT5_mid");
    contactPointsRight.append("R_end");
    // Left side
    Array<std::string> contactPointsLeft;
    contactPointsLeft.append("LCAL");
    contactPointsLeft.append("LTOE");
    contactPointsLeft.append("LMT5");
    contactPointsLeft.append("LTOE_mid");
    contactPointsLeft.append("LMT5_mid");
    contactPointsLeft.append("L_end");

    // Loop through the right side contact points, get the marker locations
    // and then append to the relevant contact side. Note that all markers
    // are attached to the model via the calcaneus bodies, even though that
    // they are more applicable to the toes bodies
    // Right side
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker =
                model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point
        ContactPoint* cp = new ContactPoint(
            "contact_point_" + contactName, calcn_r, pointLocation);
        cp->set_distance_threshold(0.04);
        cp->set_contact_checking_method("distance");
        // Add the contact point to the contact side
        rightContactSide->addContactPoint(cp);
    }
    // Left side
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker = model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point
        ContactPoint* cp = new ContactPoint(
                "contact_point_" + contactName, calcn_l, pointLocation);
        cp->set_distance_threshold(0.04);
        cp->set_contact_checking_method("distance");
        // Add the contact point to the contact side
        leftContactSide->addContactPoint(cp);
    }

    // Print some outputs
    std::cout << "Right contact side created:\n";
    std::cout << "  Side name:        " << rightContactSide->getName()   << "\n";
    std::cout << "  Contact points:   " << rightContactSide->getNumContactPoints() << "\n";
    std::cout << "Left contact side created:\n";
    std::cout << "  Side name:        " << leftContactSide->getName() << "\n";
    std::cout << "  Contact points:   " << leftContactSide->getNumContactPoints() << "\n";

    // ======================================================================
    // STEP 4: Add sides to estimator, add estimator to model
    // ======================================================================
    printSection("Step 4. Connecting estimator to model");

    // Add contact sides to estimator
    zmp->addContactSide(rightContactSide);
    zmp->addContactSide(leftContactSide);

    // Add estimator component to model
    model.addComponent(zmp);

    // Print the updated model to file so the component configuration is
    // serialised and can be reloaded without repeating this setup.
    const std::string outputModelFile = "subject01_run3_with_zmp.osim";
    model.finalizeConnections();
    model.print(outputModelFile);
    std::cout << "Model with ZMP estimator printed to: " << outputModelFile << "\n";

    // Initialise the system so outputs can be evaluated.
    SimTK::State state = model.initSystem();
    std::cout << "Model initialised successfully.\n";

    // ======================================================================
    // STEP 5: Query component properties
    // ======================================================================
    printSection("Step 5. Querying component properties");

    // Retrieve the estimator from the model by component path
    const auto& estimator =
        model.getComponent<ZeroMomentPointEstimator>("/zmp_grf_estimator");

    // Print some outputs to demonstrate retrieval of properties
    std::cout << "ZeroMomentPointEstimator properties:\n";

    // Number of contact sides
    std::cout << "  Number of contact sides: "
              << estimator.getNumContactSides() << "\n";

    // Contact point parameters
    for (int i = 0; i < estimator.getNumContactSides(); ++i) {
        const ContactSide& side = estimator.getContactSide(i);
        std::cout << "\n  Side " << i+1 << ": '" << side.getName() << "'\n";
        std::cout << "    Component name:   " << side.getName() << "\n";
        std::cout << "    Contact points:   "
                  << side.getNumContactPoints() << "\n";
        for (int j = 0; j < side.getNumContactPoints(); ++j) {
            const ContactPoint& cp = side.getContactPoint(j);
            std::cout << "      [" << j+1 << "] " << cp.getName()
                      << "  location: " << cp.get_location()
                      << "  threshold: " << cp.getDistanceThreshold()
                      << " m  method: " << cp.getContactCheckingMethod()
                      << "\n";
        }
    }

    // Demonstrate getContactSide by name
    std::cout << "\n  Accessing right side by name:\n";
    const ContactSide& rightByName = estimator.getContactSide("right_side");
    std::cout << "    Found side: '" << rightByName.getName() << "' ("
              << rightByName.getNumContactPoints() << " contact points)\n";

    // Show available output channels
    std::cout << "\n  ZMP estimator output channels:\n";
    const std::vector<std::string>& outputs = estimator.getOutputNames();
    for (const std::string& outputName : outputs) {
        std::cout << "    " << outputName << "\n";
    }

    // ======================================================================
    // STEP 6: Generate ZMP GRF table from Moco tracking solution
    // ======================================================================
    printSection("Step 6. Generating ZMP GRF table from tracking solution");

    // Get the Moco tracking solution as a TimeSeriesTable of states and bvalues.
    // The solution .sto file contains state variable trajectories that
    // can be passed directly to createZMPLoadsFromStates.
    std::cout << "Loading tracking solution: " << solutionFile << "\n";
    MocoTrajectory mocoTraj =
            MocoTrajectory("subject01_run3_trackingSolution.sto");
    TimeSeriesTable statesTable = mocoTraj.exportToStatesTable();
    TimeSeriesTable valuesTable = mocoTraj.exportToValuesTable();

    // Generate the ZMP-estimated GRF table.
    // This loops over every time point in the tables, realises the
    // model state, queries the estimator outputs, and assembles a flat
    // TimeSeriesTable in the standard OpenSim GRF .mot format.
    // We can create the ZMP-estimated GRFs from both values tables and
    // states tables (values and speeds). The process is the same, with
    // the difference being that the values only approach differentiates
    // first to get the speeds (i.e. full states) and then passes these
    // to the createZMPLoadsFromStates function. Given that the values
    // are twice differentiated to get accelerations, there could be some
    // slight differences between the generated GRFs - likely dependent
    // on how smooth vs. noisy the values signals are.
    TimeSeriesTable zmpGRFTablefromValues =
            createZMPLoadsFromStates(model, statesTable, estimator);
    TimeSeriesTable zmpGRFTable_fromStates =
            createZMPLoadsFromStates(model, statesTable, estimator);

    // Print some outputs from one of the generated tables
    std::cout << "ZMP GRF table generated from states:\n";
    std::cout << "  Rows:    " << zmpGRFTable_fromStates.getNumRows()
        << "\n";
    std::cout << "  Columns: " << zmpGRFTable_fromStates.getNumColumns()
        << "\n";
    std::cout << "  Column labels:\n";
    for (const auto& label : zmpGRFTable_fromStates.getColumnLabels())
        std::cout << "    " << label << "\n";

    // Write the ZMP GRF tables to a .mot file that can be loaded in the
    // OpenSim GUI or used as input to further analyses.
    const std::string zmpGRFFile_fromStates = 
        "subject01_run3_zmp_grf_fromStates.mot";
    STOFileAdapter::write(zmpGRFTable_fromStates, zmpGRFFile_fromStates);
    std::cout << "\nZMP GRF table from states written to: " 
        << zmpGRFFile_fromStates << "\n";
    const std::string zmpGRFFile_fromValues =
            "subject01_run3_zmp_grf_fromValues.mot";
    STOFileAdapter::write(zmpGRFTable_fromStates, zmpGRFFile_fromValues);
    std::cout << "ZMP GRF table from states written to: "
              << zmpGRFFile_fromValues << "\n";

    // ======================================================================
    // Wrapping up
    // ======================================================================
    printSection("Example complete");
    std::cout << "Output files produced:\n";
    std::cout << "  " << outputModelFile << "\n";
    std::cout << "  " << zmpGRFFile_fromStates      << "\n";
    std::cout << "  " << zmpGRFFile_fromValues << "\n";
    std::cout << "\nThe ZMP-estimated GRFs can be compared against the\n";
    std::cout << "measured GRF data in: " << grfFile << "\n";

    return 0;
}
