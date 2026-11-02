/* -------------------------------------------------------------------------- *
 *         OpenSim: exampleFluit2014ContactPointActuator.cpp                  *
 * -------------------------------------------------------------------------- *
 * Demonstrates how to add the Fluit2014ContactPointActuator class to an      *
 * OpenSim model.                                                             *
 *                                                                            *
 * Please review the associated README.MD file for example specific details.  *
 *                                                                            *
 * @author  Aaron Fox                                                         *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/Fluit2014ContactPointActuator.h>

#include <iostream>
#include <string>

using namespace OpenSim;

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

    Model model(modelFile);
    std::cout << "Loaded model: " << model.getName() << "\n";
    std::cout << "  Bodies:      " << model.getBodySet().getSize()  << "\n";
    std::cout << "  Joints:      " << model.getJointSet().getSize() << "\n";
    std::cout << "  Markers:     " << model.getMarkerSet().getSize()<< "\n";

    // ======================================================================
    // STEP 2: Add the contact point actuators
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
    // Contact actuator parameters are set in building this example, with most
    // of these reflecting defaults. The parameters are simply set here to
    // provide an example of using the set methods.
    // ======================================================================
    printSection("Step 2. Adding contact actuators.");

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
    // and then create the contact actuators for these points. Note that all 
    // markers are attached to the model via the calcaneus bodies, even though 
    // that they are more applicable to the toes bodies
    // Right side
    for (int ii = 0; ii < contactPointsRight.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsRight.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker =
                model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point actuator
        Fluit2014ContactPointActuator* conAct =
                new Fluit2014ContactPointActuator(
                        "contact_actuator_" + contactName, "calcn_r");
        // Set the parameters
        conAct->set_point(pointLocation);
        conAct->set_max_force(1000.0);
        conAct->set_ground_height(0.0);
        conAct->set_contact_height_threshold(0.03);
        conAct->set_contact_velocity_threshold(0.8);
        conAct->set_use_velocity_in_scaling(false);
        conAct->set_smoothing(8.0);
        conAct->set_friction_coefficient(0.5);
        conAct->set_min_control(0.0);
        conAct->set_max_control(1.0);
        // Add the contact point to the model
        model.addForce(conAct);
        // Print confirmation
        std::cout << "Contact actuator created: " << conAct->getName() << "\n";
    }
    // Left side
    for (int ii = 0; ii < contactPointsLeft.size(); ii++) {
        // Get the marker name
        std::string contactName = contactPointsLeft.get(ii);
        // Get the marker associated to the current point name
        Marker pointMarker = model.getMarkerSet().get(contactName + "_ground");
        // Get the location of the marker
        SimTK::Vec3 pointLocation = pointMarker.get_location();
        // Build the contact point actuator
        Fluit2014ContactPointActuator* conAct =
                new Fluit2014ContactPointActuator(
                        "contact_actuator_" + contactName, "calcn_l");
        // Set the parameters
        conAct->set_point(pointLocation);
        conAct->set_max_force(1000.0);
        conAct->set_ground_height(0.0);
        conAct->set_contact_height_threshold(0.03);
        conAct->set_contact_velocity_threshold(0.8);
        conAct->set_use_velocity_in_scaling(false);
        conAct->set_smoothing(8.0);
        conAct->set_friction_coefficient(0.5);
        conAct->set_min_control(0.0);
        conAct->set_max_control(1.0);
        // Add the contact point to the model
        model.addForce(conAct);
        // Print confirmation
        std::cout << "Contact actuator created: " << conAct->getName() << "\n";
    }

    // ======================================================================
    // STEP 1: Save model to file
    // ======================================================================
    printSection("Step 3. Saving model to file.");

    // Print the model to file
    model.finalizeConnections();
    std::string outputModelFile =
            "subject01_run3_withContactPointActuators.osim";
    model.print(outputModelFile);

    // ======================================================================
    // Wrapping up
    // ======================================================================
    printSection("Example complete");
    std::cout << "Output files produced:\n";
    std::cout << "  " << outputModelFile << "\n";

    return 0;
}
