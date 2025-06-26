/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimZmp.cpp                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2025 Stanford University and the Authors                     *
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

#include "RegisterTypes_osimZmp.h"

#include "ContactPoint.h"
#include "ContactPointSet.h"
#include "ZmpGroundReactions.h"

#include <string>
#include <iostream>
#include <exception>

using namespace OpenSim;

static osimZmpInstantiator instantiator;

OSIMZMP_API void RegisterTypes_osimZmp() 
{
    try {
		
		Object::registerType(ContactPoint());
		Object::registerType(ContactPointSet());
		Object::registerType(ZmpGroundReactions());

    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimZmp Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimZmpInstantiator::osimZmpInstantiator() { registerDllClasses(); }

void osimZmpInstantiator::registerDllClasses() { RegisterTypes_osimZmp(); }
