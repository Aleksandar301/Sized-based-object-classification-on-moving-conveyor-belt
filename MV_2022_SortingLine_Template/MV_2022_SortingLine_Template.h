#ifndef __MV_2019_SortingLine_Template_H__
#define __MV_2019_SortingLine_Template_H__


#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// Setting for using Basler GigE cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>
//typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CInstantCamera Camera_t;
//using namespace Basler_GigECameraParams;

// Namespace for using pylon objects.
using namespace Pylon;

int SortingLineSetPosition (std::mutex* comm_lock, unsigned long position);
unsigned long SortingLineGetCurrentPosition(std::mutex* comm_lock);
unsigned long SortingLineGetObjectBeginningPosition (std::mutex* comm_lock);
unsigned long SortingLineGetObjectEndPosition (std::mutex* comm_lock);
int SortingLineGetInPositionStatus(std::mutex* comm_lock);
int SortingLineSetPositionPusher1 (unsigned long mavis_position_obj_push);
int SortingLineSetPositionPusher2 (unsigned long mavis_position_obj_push);
int SortingLineSetPositionPusher3 (unsigned long mavis_position_obj_push);

int AcquireImage(Camera_t *camera);

#endif