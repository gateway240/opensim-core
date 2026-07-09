
//osimTools
%include <OpenSim/Tools/osimToolsDLL.h>
%include <OpenSim/Tools/IKTask.h>
%template(SetIKTasks) OpenSim::Set<OpenSim::IKTask, OpenSim::Object>;
%include <OpenSim/Tools/IKMarkerTask.h>
%include <OpenSim/Tools/IKCoordinateTask.h>
%include <OpenSim/Tools/IKTaskSet.h>
%include <OpenSim/Tools/MarkerPair.h>
%include <OpenSim/Tools/Measurement.h>
%include <OpenSim/Tools/GenericModelMaker.h>
%include <OpenSim/Tools/ModelScaler.h>
%include <OpenSim/Tools/MarkerPlacer.h>
%include <OpenSim/Tools/ScaleTool.h>

%include <OpenSim/Tools/Tool.h>
%include <OpenSim/Tools/DynamicsTool.h>
%include <OpenSim/Tools/InverseDynamicsTool.h>
%include <OpenSim/Tools/ForwardTool.h>

%include <OpenSim/Tools/TrackingTask.h>
%include <OpenSim/Tools/CMC_Task.h>
%include <OpenSim/Tools/CMC_Joint.h>
%include <OpenSim/Tools/CMC_Point.h>
%template (SetTrackingTasks) OpenSim::Set<OpenSim::TrackingTask, OpenSim::Object>;
%include <OpenSim/Tools/CMC_TaskSet.h>

%include <OpenSim/Tools/CMCTool.h>
%include <OpenSim/Tools/RRATool.h>
%include <OpenSim/Tools/AnalyzeTool.h>
%include <OpenSim/Tools/InverseKinematicsToolBase.h>
%include <OpenSim/Tools/InverseKinematicsTool.h>
%include <OpenSim/Tools/IMUInverseKinematicsTool.h>
