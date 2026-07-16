#ifndef OPENSIM_INVERSE_DYNAMICS_TOOL_H_
#define OPENSIM_INVERSE_DYNAMICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  InverseDynamicsTool.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Property.h>
#include "DynamicsTool.h"

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class Model;
class JointSet;

//=============================================================================
//=============================================================================
/**
 * A Tool that performs an Inverse Dynamics analysis with a given model.
 * Inverse Dynamics is the solution for the generalized-coordinate forces that
 * generate given generalized-coordinate accelerations at a given state.
 * This Tool determines the state from provided coordinate trajectories as
 * functions as that are twice differentiable to estimate velocities and
 * accelerations.
 *
 * As an additional service, the InverseDynamicsTool can provide an equivalent 
 * body force (torque and force) applied to the joint frame. Since generalized
 * forces include scaling (due to units conversion as well as coupling between
 * translations and rotations, for example) they are not necessarily joint torques
 * or forces.  OpenSim employs a pseudo inverse to find the smallest applied  
 * torque and/or force that will generate the equivalent generalized force.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API InverseDynamicsTool: public DynamicsTool {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseDynamicsTool, DynamicsTool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
public:
OpenSim_DECLARE_PROPERTY(coordinates_file, std::string, "The name of the file containing coordinate data. Can be a motion (.mot) or a states (.sto) file.");
OpenSim_DECLARE_PROPERTY(lowpass_cutoff_frequency_for_coordinates, double, "Low-pass cut-off frequency for filtering the coordinates_file data (currently does not apply to states_file or speeds_file). "
                 "A negative value results in no filtering. The default value is -1.0, so no filtering.");
OpenSim_DECLARE_PROPERTY(output_gen_force_file, std::string, "Name of the storage file (.sto) to which the generalized forces "
            "are written. Only a filename should be specified here (not a "
            "full path); the file will appear in the location provided in the "
            "results_directory property.");
OpenSim_DECLARE_LIST_PROPERTY(joints_to_report_body_forces, std::string, "List of joints (keyword All, for all joints)"
        " to report body forces acting at the joint frame expressed in ground.");
OpenSim_DECLARE_PROPERTY(output_body_forces_file, std::string, "Name of the storage file (.sto) to which the body forces at specified joints are written.");

private:
    Storage* _coordinateValues;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~InverseDynamicsTool();
    InverseDynamicsTool();
    InverseDynamicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;

    /* Register types to be used when reading an InverseDynamicsTool object from xml file. */
    static void registerTypes();
    /* Handle reading older formats/Versioning */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

protected:
    /** helper method to get a list of model joints by name */
    void getJointsByName(Model &model, const Array<std::string> &jointNames, JointSet &joints) const;

private:
    void constructProperties();
    /* If CoordinatesFile property is populated, load data into a live _coordinateValues
    storage object. */
    bool loadCoordinateValues();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:

    //--------------------------------------------------------------------------    
    // GET AND SET
    //--------------------------------------------------------------------------
    void setCoordinateValues(const OpenSim::Storage& aStorage);
    /**
     * get/set the name of the file to be used as output from the tool
     */
    std::string getOutputGenForceFileName() const { return get_output_gen_force_file();}
    void setOutputGenForceFileName(const std::string& desiredOutputFileName) {
        set_output_gen_force_file(desiredOutputFileName);
    }
    /**
     * get/set the name of the file containing coordinates
     */
    const std::string& getCoordinatesFileName() const { return get_coordinates_file();};
    /** %Set the name of the coordinatesFile to be used. This call resets 
     _coordinateValues as well. */
    void setCoordinatesFileName(const std::string& aCoordinateFile)  { 
        set_coordinates_file(aCoordinateFile);
        if (_coordinateValues != NULL){
            // there's an old Storage hanging around from potentially different 
            // CoordinatesFile, wipe it out.
            delete _coordinateValues;
            _coordinateValues = NULL;
        }
    }
    double getLowpassCutoffFrequency() const {
        return get_lowpass_cutoff_frequency_for_coordinates();
    };
    void setLowpassCutoffFrequency(double aFrequency) {
        set_lowpass_cutoff_frequency_for_coordinates(aFrequency);
    }
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run() override SWIG_DECLARE_EXCEPTION;


//=============================================================================
};  // END of class InverseDynamicsTool
//=============================================================================
} // namespace

#endif // OPENSIM_INVERSE_DYNAMICS_TOOL_H_
