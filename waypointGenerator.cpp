//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: waypointGenerator.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 02-May-2019 23:20:30
//

// Include Files
#include "waypointGenerator.h"
#include "waypointGenerator_emxutil.h"

// Function Definitions

//
// Inputs:
//  vertPoints:   number of vertical points required, found by doing width/TurnRadius
//  topLeft:      top left point of the search area, array [x_TL, y_TL]
//  botRight:     bot right point of the search area, array [x_BL, y_BL]
//  botLeftGPs:   array for lat and long of bot left point
// Arguments    : const double topLeft[2]
//                const double botRight[2]
//                double vertPoints
//                const double botLeftGPS[2]
//                emxArray_real_T *SearchPattern
// Return Type  : void
//
void waypointGenerator(const double topLeft[2], const double botRight[2], double
  vertPoints, const double botLeftGPS[2], emxArray_real_T *SearchPattern)
{
  emxArray_real_T *left_waypoints;
  emxArray_real_T *mid_waypoints;
  emxArray_real_T *right_waypoints;
  double botMid_idx_0;
  double botMid_idx_1;
  double x_distance_iteration;
  double y_distance_iteration;
  int i0;
  int i1;
  int i;
  double d0;
  int direction;
  double d1;
  emxInit_real_T(&left_waypoints, 2);
  emxInit_real_T(&mid_waypoints, 2);
  emxInit_real_T(&right_waypoints, 2);

  // Function assumes Cartesian plane, then converts to geodetic coordinates
  // In C++ code, use gpsDistance function to find top left and bot right
  // points using bot left as a reference point for deltaLat and deltaLong
  botMid_idx_0 = botRight[0] / 2.0;
  botMid_idx_1 = botRight[1] / 2.0;
  x_distance_iteration = topLeft[0] / vertPoints;
  y_distance_iteration = topLeft[1] / vertPoints;
  i0 = (int)vertPoints;
  i1 = left_waypoints->size[0] * left_waypoints->size[1];
  left_waypoints->size[0] = i0;
  left_waypoints->size[1] = 2;
  emxEnsureCapacity_real_T(left_waypoints, i1);
  i1 = mid_waypoints->size[0] * mid_waypoints->size[1];
  mid_waypoints->size[0] = i0;
  mid_waypoints->size[1] = 2;
  emxEnsureCapacity_real_T(mid_waypoints, i1);
  i1 = right_waypoints->size[0] * right_waypoints->size[1];
  right_waypoints->size[0] = i0;
  right_waypoints->size[1] = 2;
  emxEnsureCapacity_real_T(right_waypoints, i1);
  for (i = 0; i < i0; i++) {
    d0 = x_distance_iteration * (1.0 + (double)i);
    left_waypoints->data[i] = d0;
    d1 = y_distance_iteration * (1.0 + (double)i);
    left_waypoints->data[i + left_waypoints->size[0]] = d1;
    mid_waypoints->data[i] = botMid_idx_0 + d0;
    mid_waypoints->data[i + mid_waypoints->size[0]] = botMid_idx_1 + d1;
    right_waypoints->data[i] = botRight[0] + d0;
    right_waypoints->data[i + right_waypoints->size[0]] = botRight[1] + d1;
  }

  x_distance_iteration = 3.0 * vertPoints;
  i1 = SearchPattern->size[0] * SearchPattern->size[1];
  direction = (int)x_distance_iteration;
  SearchPattern->size[0] = direction;
  SearchPattern->size[1] = 2;
  emxEnsureCapacity_real_T(SearchPattern, i1);
  direction <<= 1;
  for (i1 = 0; i1 < direction; i1++) {
    SearchPattern->data[i1] = 0.0;
  }

  direction = 1;
  y_distance_iteration = 3.0;
  SearchPattern->data[0] = 0.0;
  SearchPattern->data[SearchPattern->size[0]] = 0.0;
  SearchPattern->data[1] = botMid_idx_0;
  SearchPattern->data[1 + SearchPattern->size[0]] = botMid_idx_1;
  SearchPattern->data[2] = botRight[0];
  SearchPattern->data[2 + SearchPattern->size[0]] = botRight[1];
  for (i = 0; i < i0; i++) {
    if (direction == 0) {
      SearchPattern->data[(int)(y_distance_iteration + 1.0) - 1] =
        left_waypoints->data[i];
      SearchPattern->data[((int)(y_distance_iteration + 1.0) +
                           SearchPattern->size[0]) - 1] = left_waypoints->data[i
        + left_waypoints->size[0]];
      SearchPattern->data[(int)(y_distance_iteration + 2.0) - 1] =
        mid_waypoints->data[i];
      SearchPattern->data[((int)(y_distance_iteration + 2.0) +
                           SearchPattern->size[0]) - 1] = mid_waypoints->data[i
        + mid_waypoints->size[0]];
      SearchPattern->data[(int)(y_distance_iteration + 3.0) - 1] =
        right_waypoints->data[i];
      SearchPattern->data[((int)(y_distance_iteration + 3.0) +
                           SearchPattern->size[0]) - 1] = right_waypoints->
        data[i + right_waypoints->size[0]];
      direction = 1;
      y_distance_iteration += 3.0;
    } else {
      i1 = (int)(y_distance_iteration + 1.0);
      SearchPattern->data[i1 - 1] = right_waypoints->data[i];
      SearchPattern->data[(i1 + SearchPattern->size[0]) - 1] =
        right_waypoints->data[i + right_waypoints->size[0]];
      i1 = (int)(y_distance_iteration + 2.0);
      SearchPattern->data[i1 - 1] = mid_waypoints->data[i];
      SearchPattern->data[(i1 + SearchPattern->size[0]) - 1] =
        mid_waypoints->data[i + mid_waypoints->size[0]];
      i1 = (int)(y_distance_iteration + 3.0);
      SearchPattern->data[i1 - 1] = left_waypoints->data[i];
      SearchPattern->data[(i1 + SearchPattern->size[0]) - 1] =
        left_waypoints->data[i + left_waypoints->size[0]];
      direction = 0;
      y_distance_iteration += 3.0;
    }
  }

  emxFree_real_T(&right_waypoints);
  emxFree_real_T(&mid_waypoints);
  emxFree_real_T(&left_waypoints);
  i0 = (int)(x_distance_iteration + 3.0);
  for (i = 0; i < i0; i++) {
    x_distance_iteration = SearchPattern->data[i];

    //  INPUT:
    //  deltaX: desired x distance (in m). This x up in cartesian coordinates
    //  deltaY: desired y distance (in m). This y is West/East direction
    //  pos:    Current position [lat,lon]
    // This function uses the NED frame with +x = North = dLat
    //                                   and +y = East  = dLon
    // km
    //  To meters
    // coordinate offset in Radians
    SearchPattern->data[i] = botLeftGPS[0] + SearchPattern->data[i +
      SearchPattern->size[0]] * 0.001 / 6378.037 * 57.295779513082323;
    SearchPattern->data[i + SearchPattern->size[0]] = botLeftGPS[1] +
      x_distance_iteration * 0.001 / 6378.037 * 57.295779513082323;
  }

  // [lattrk, longtrk] = track(SearchPattern); %checks waypoints path if so desired 
}

//
// File trailer for waypointGenerator.cpp
//
// [EOF]
//
