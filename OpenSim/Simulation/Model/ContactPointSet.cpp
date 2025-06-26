/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ContactPointSet.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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

#include "ContactPointSet.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Get names of contact points in the set
 */
void ContactPointSet::getContactPointNames(Array<string>& contactPointNamesArray) const {
    contactPointNamesArray.setSize(0);
    for (int i = 0; i < getSize(); i++)
    {
        ContactPoint& nextContactPoint = get(i);
        contactPointNamesArray.append(nextContactPoint.getName());
    }
}

void ContactPointSet::addNamePrefix(const string& prefix)
{
    int i;

    // Cycle through set and add prefix
    for (i = 0; i < getSize(); i++)
        get(i).setName(prefix + get(i).getName());
}

