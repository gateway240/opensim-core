#ifndef _MuscleAnalysis_h_
#define _MuscleAnalysis_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  MuscleAnalysis.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Matthew Millard, Katherine R. S. Holzbaur,           *
 *            Frank C. Anderson                                               *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include "osimAnalysesDLL.h"


#ifdef SWIG
    #ifdef OSIMANALYSES_API
        #undef OSIMANALYSES_API
        #define OSIMANALYSES_API
    #endif
#endif



namespace OpenSim { 

class Coordinate;

//=============================================================================
//=============================================================================
/**
 * A class for recording and computing basic quantities (length, shortening
 * velocity, tendon length, ...) for muscles during a simulation.
 *
 * @author Ajay Seth, Matthew Millard, Katherine Holzbaur, Frank C. Anderson 
 * @version 1.0
 */
class OSIMANALYSES_API MuscleAnalysis : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleAnalysis, Analysis);
#ifndef SWIG
//=============================================================================
// DATA
//=============================================================================
public:
    // STRUCT FOR PAIRING MOMENT ARM STORAGE OBJECTS WITH THEIR
    // ASSOCIATE GENERALIZED COORDINATE

    typedef struct {
        Coordinate *q;
        Storage *momentArmStore;
        Storage *momentStore;
    }  
// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond

    StorageCoordinatePair;
#endif
    /// @endcond
private:

    /** List of muscles for which to compute moment arms. */
    PropertyStrArray _muscleListProp;

    /** List of generalized coordinates for which to compute moment arms. */
    PropertyStrArray _coordinateListProp;

    /** Compute moments and moment arms. */
    PropertyBool _computeMomentsProp;

    /** Pennation angle storage. */
    std::unique_ptr<Storage> _pennationAngleStore;
    /** Muscle-tendon length storage. */
    std::unique_ptr<Storage> _lengthStore;
    /** Fiber length storage. */
    std::unique_ptr<Storage> _fiberLengthStore;
    /** Normalized fiber length storage. */
    std::unique_ptr<Storage> _normalizedFiberLengthStore;
    /** Tendon length storage. */
    std::unique_ptr<Storage> _tendonLengthStore;

    /** Lengthening velocity of the muscle fibers. */
    std::unique_ptr<Storage> _fiberVelocityStore;
    /** Normalized lengthening velocity of the muscle fibers. */
    std::unique_ptr<Storage> _normFiberVelocityStore;
    /** Angular velocity of the muscle fibers. */
    std::unique_ptr<Storage> _pennationAngularVelocityStore;

    /** Force applied by the muscle. */
    std::unique_ptr<Storage> _forceStore;
    /** Force in the muscle fibers. */
    std::unique_ptr<Storage> _fiberForceStore;
    /** Active force in the muscle fibers. */
    std::unique_ptr<Storage> _activeFiberForceStore;
    /** Passive force in the muscle fibers. */
    std::unique_ptr<Storage> _passiveFiberForceStore;
    /** Active force in the muscle fibers along tendon. */
    std::unique_ptr<Storage> _activeFiberForceAlongTendonStore;
    /** Passive force in the muscle fibers along tendon. */
    std::unique_ptr<Storage> _passiveFiberForceAlongTendonStore;

    /** Fiber power */
    std::unique_ptr<Storage> _fiberActivePowerStore;
    std::unique_ptr<Storage> _fiberPassivePowerStore;
    /** Tendon power */
    std::unique_ptr<Storage> _tendonPowerStore;
    /** Muscle actuator power */
    std::unique_ptr<Storage> _musclePowerStore;

    // FOR MOMENT ARMS AND MOMENTS----------------
    /** Work array for holding the list of muscles.  This array */
    Array<std::string> _muscleList;

    /** Work array for holding the list of coordinates. */
    Array<std::string> _coordinateList;

#ifndef SWIG
    /** Array of active storage and coordinate pairs. */
    ArrayPtrs<StorageCoordinatePair> _momentArmStorageArray;
#endif
    /** Array of active muscles. */
    ArrayPtrs<Muscle> _muscleArray;

//=============================================================================
// METHODS
//=============================================================================
public:
    MuscleAnalysis(Model *aModel=0);
    MuscleAnalysis(const std::string &aFileName);
    MuscleAnalysis(const MuscleAnalysis &aObject);
    virtual ~MuscleAnalysis();

private:
    void setNull();
    void setupProperties();
    void constructDescription();
    void constructColumnLabels();

public:
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    MuscleAnalysis& operator=(const MuscleAnalysis &aMuscleAnalysis);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setModel(Model& aModel) override;
    [[deprecated("this method no longer does anything")]]
    void setStorageCapacityIncrements(int) {}

    Storage* getPennationAngleStorage() const { 
        return _pennationAngleStore.get(); }
    Storage* getMuscleTendonLengthStorage() const { 
        return _lengthStore.get(); }
    Storage* getFiberLengthStorage() const { 
        return _fiberLengthStore.get(); }
    Storage* getNormalizedFiberLengthStorage() const { 
        return _normalizedFiberLengthStore.get(); }
    Storage* getTendonLengthStorage() const { 
        return _tendonLengthStore.get(); }

    Storage* getFiberVelocityStorage() const { 
        return _fiberVelocityStore.get(); }
    Storage* getNormalizedFiberVelocityStorage() const { 
        return _normFiberVelocityStore.get(); }
    Storage* getPennationAngularVelocityStorage() const { 
        return _pennationAngularVelocityStore.get(); }

    Storage* getForceStorage() const { 
        return _forceStore.get(); }
    Storage* getFiberForceStorage() const { 
        return _fiberForceStore.get(); }
    Storage* getActiveFiberForceStorage() const { 
        return _activeFiberForceStore.get(); }
    Storage* getPassiveFiberForceStorage() const { 
        return _passiveFiberForceStore.get(); }
    Storage* getActiveFiberForceAlongTendonStorage() const { 
        return _activeFiberForceAlongTendonStore.get(); }
    Storage* getPassiveFiberForceAlongTendonStorage() const { 
        return _passiveFiberForceAlongTendonStore.get(); }
    
    Storage* getFiberActivePowerStorage() const { 
        return _fiberActivePowerStore.get(); }
    Storage* getFiberPassivePowerStorage() const { 
        return _fiberPassivePowerStore.get(); }
    Storage* getTendonPowerStorage() const { 
        return _tendonPowerStore.get(); }
    Storage* getMusclePowerStorage() const { 
        return _musclePowerStore.get(); }

    void setMuscles(Array<std::string>& aMuscles);
    void setCoordinates(Array<std::string>& aCoordinates);

    void setComputeMoments(bool aTrueFalse) {
        _computeMomentsProp.setValue(aTrueFalse);
    }
    bool getComputeMoments() const { 
        return _computeMomentsProp.getValueBool();
    }
#ifndef SWIG
    const ArrayPtrs<StorageCoordinatePair>& getMomentArmStorageArray() const { return _momentArmStorageArray; }
#endif
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
    int
        begin( const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end( const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;
    /** 
     * Intended for use only by GUI that holds one MuscleAnalysis and keeps changing attributes to generate various plots
     * For all other use cases, the code handles the allocation/deallocation of resources internally.
     */
    void allocateStorageObjects();
//=============================================================================
};  // END of class MuscleAnalysis

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MuscleAnalysis_h__
