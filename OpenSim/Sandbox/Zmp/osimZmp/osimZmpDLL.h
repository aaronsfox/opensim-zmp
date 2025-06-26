#ifndef OPENSIM_OSIMZMPDLL_H
#define OPENSIM_OSIMZMPDLL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: osimZmpDLL.h                                                      *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


#ifndef _WIN32
    #define OSIMZMP_API
#else
    #ifdef OSIMZMP_EXPORTS
        #define OSIMZMP_API __declspec(dllexport)
    #else
        #define OSIMZMP_API __declspec(dllimport)
    #endif
#endif

#endif // OPENSIM_OSIMZMPDLL_H
