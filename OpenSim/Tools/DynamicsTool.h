#ifndef __DynamicsTool_h__
#define __DynamicsTool_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  DynamicsTool.h                          *
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

#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include <OpenSim/Common/Property.h>
#include "Tool.h"
#include <string>

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * An abstract Tool for defining tools for performing a dynamics analysis 
 * with a given model. For example, InverseDynamics and ForwardDynamics Tools
 * derive from DynamicsTool, which provides convenient method for performing
 * and dynamics analysis over or to produce a trajectory in time.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API DynamicsTool: public Tool {
OpenSim_DECLARE_ABSTRACT_OBJECT(DynamicsTool, Tool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
public:
OpenSim_DECLARE_PROPERTY(model_file, std::string, "Name of the .osim file used to construct a model.");
OpenSim_DECLARE_LIST_PROPERTY_SIZE(time_range, double, 2, "Time range over which the inverse dynamics problem is solved.");
OpenSim_DECLARE_LIST_PROPERTY(forces_to_exclude, std::string, "List of forces by individual or grouping name "
            "(e.g. All, actuators, muscles, ...)"
            " to be excluded when computing model dynamics. "
            "'All' also excludes external loads added "
            "via 'external_loads_file'."); 
OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "XML file (.xml) containing the external loads applied to the model as a set of ExternalForce(s).")
OpenSim_DECLARE_PROPERTY(ExternalLoads, ExternalLoads, "ExternalLoads member for creating and editing applied external forces"
    "(e.g. GRFs through the GUI) prior to running the Tool")

protected:
    /** Pointer to the model being investigated. */
    Model *_model;
    // Reference to external loads added to the model but not owned by the Tool
    SimTK::ReferencePtr<ExternalLoads> _modelExternalLoads;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~DynamicsTool();
    DynamicsTool();
    DynamicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;

    /** Modify model to exclude specified forces by disabling those identified by name or group */
    void disableModelForces(Model &model, SimTK::State &s, const Array<std::string> &forcesByNameOrGroup);
    
    const ExternalLoads& getExternalLoads() const { return get_ExternalLoads(); }
    ExternalLoads& updExternalLoads() { return upd_ExternalLoads(); }

    // External loads get/set
    const std::string &getExternalLoadsFileName() const { return get_external_loads_file(); }
    void setExternalLoadsFileName(const std::string &aFileName) { 
        set_external_loads_file(aFileName);
    }

    // Model file name
    void setModelFileName(const std::string &aFileName) {
        set_model_file(aFileName);
    }

    std::string getModelFileName() const { return get_model_file(); };

private:
    void constructProperties();

public:
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    void setStartTime(double d) { upd_time_range(0) = d; };
    double getStartTime() const {return  get_time_range(0); };

    void setEndTime(double d) { upd_time_range(1) = d; };
    double getEndTime() const {return  get_time_range(1); };
    void setModel(Model& aModel) { _model = &aModel; };

    void setExcludedForces(const Array<std::string> &aExcluded) {
        set_forces_to_exclude(aExcluded);
    }
    bool createExternalLoads( const std::string &externalLoadsFileName,
                              Model& model);

    bool modelHasExternalLoads() { return !_modelExternalLoads.empty(); }

    void removeExternalLoadsFromModel();

    virtual bool run() override SWIG_DECLARE_EXCEPTION=0;


//=============================================================================
};  // END of class DynamicsTool
//=============================================================================
} // namespace

#endif // __DynamicsTool_h__
