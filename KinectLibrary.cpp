#include "KinectLibrary.h"
#include "KinectLibraryException.h"
#include "Windows.h"
#include "Kinect.h"
#include "Ole2.h"
#include <stdint.h>
#include <intrin.h>

KinectLibrary::KinectLibrary(uint8_t sensors) {

	HRESULT result = GetDefaultKinectSensor(&sensor);
	int count = 0;
	if (FAILED(result)) {
		throw KinectLibraryException("Failed to get the Kinect sensor");
	}
	else if (!sensor) {
		return;
	}
	
	if (__popcnt16((uint16_t)sensors) > 1) {
		DWORD frameSourceTypes = 0;
		if (sensors & COLOR_SENSOR) {
			frameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Color;
		}
		if (sensors & DEPTH_SENSOR) {
			frameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Depth;
		}
		if (sensors & INFRARED_SENSOR) {
			frameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Infrared;
		}
		if (sensors & BODY_SENSOR) {
			frameSourceTypes |= FrameSourceTypes::FrameSourceTypes_Body;
		}

		result = sensor->OpenMultiSourceFrameReader((DWORD)frameSourceTypes, &msfReader);
		if (FAILED(result)) {
			throw KinectLibraryException("Failed to open the multi-source frame reader");
		}
	}
	else {
		switch (sensors) {
		case COLOR_SENSOR:
			break;
		case DEPTH_SENSOR:
			break;
		case INFRARED_SENSOR:
			break;
		case BODY_SENSOR:
			break;
		default:
			break;
		}
	}

}


KinectLibrary::~KinectLibrary() {
}
