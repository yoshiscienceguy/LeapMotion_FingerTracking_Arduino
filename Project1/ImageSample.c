/******************************************************************************\
* Copyright (C) 2012-2017 Ultraleap Ltd. All rights reserved.                  *
* Ultraleap proprietary and confidential. Not for distribution.                *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Ultraleap and you, your company or other organization.               *
\******************************************************************************/

#undef __cplusplus

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "LeapC.h"
#include "ExampleConnection.h"

#include <math.h> 
#define PI 3.14159265
int64_t lastFrameID = 0; //The last frame received

typedef struct {
	double roll;
	double pitch;
	double yaw;

}Euler;


Euler ToEulerAngles(LEAP_QUATERNION q) {
	Euler newEuler;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	newEuler.roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (abs(sinp) >= 1)
		newEuler.pitch = copysign(PI / 2, sinp); // use 90 degrees if out of range
	else
		newEuler.pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	newEuler.yaw = atan2(siny_cosp, cosy_cosp);

	return newEuler;
}
float dot(LEAP_VECTOR a, LEAP_VECTOR b)  //calculates dot product of a and b
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float mag(LEAP_VECTOR a)  //calculates magnitude of a
{
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

int main(int argc, char** argv) {
	OpenConnection();
	while (!IsConnected)
		millisleep(100); //wait a bit to let the connection complete

	printf("Connected.");
	LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
	if (deviceProps)
		printf("Using device %s.\n", deviceProps->serial);

	for (;;) {
		LEAP_TRACKING_EVENT *frame = GetFrame();
		if (frame && (frame->tracking_frame_id > lastFrameID)) {
			lastFrameID = frame->tracking_frame_id;
			//printf("Frame %lli with %i hands.\n", (long long int)frame->tracking_frame_id, frame->nHands);
			for (uint32_t h = 0; h < frame->nHands; h++) {
				LEAP_HAND* hand = &frame->pHands[h];
				//printf("    Hand id %i is a %s hand with position (%f, %f, %f).\n",
				//	hand->id,
				//	(hand->type == eLeapHandType_Left ? "left" : "right"),
				//	hand->palm.position.x,
				//	hand->palm.position.y,
				//	hand->palm.position.z);
				
				LEAP_VECTOR startP = hand->index.proximal.prev_joint;
				LEAP_VECTOR endP = hand->index.distal.next_joint;
				
				float angle = acos(dot(startP, endP) / (mag(startP)*mag(endP)));
				printf("%f\n", angle);
			}
		}
	} //ctrl-c to exit
	return 0;
}
//End-of-Sample
