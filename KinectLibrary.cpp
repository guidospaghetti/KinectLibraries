#include "KinectLibrary.h"
#include "KinectLibraryException.h"
#include "Windows.h"

#include "Ole2.h"

#include <intrin.h>

KinectLibrary::KinectLibrary(uint8_t sensors) {

	_sensors = sensors;

	HRESULT result = GetDefaultKinectSensor(&sensor);
	int count = 0;
	if (FAILED(result)) {
		throw KinectLibraryException("Failed to get the Kinect sensor");
	}
	else if (!sensor) {
		return;
	}
	
	result = sensor->Open();
	if (FAILED(result)) {
		throw KinectLibraryException("Failed to open the Kinect sensor");
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

			initColor();

			break;
		case DEPTH_SENSOR:

			initDepth();

			break;
		case INFRARED_SENSOR:

			initInfrared();

			break;
		case BODY_SENSOR:

			initBody();

			break;
		default:
			break;
		}
		
		if (frameDesc) {
			frameDesc->get_Height(&height);
			frameDesc->get_Width(&width);
		}

	}

}

void KinectLibrary::initColor() {

	HRESULT result = sensor->get_ColorFrameSource(&colorSource);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get color source");
	}

	result = colorSource->OpenReader(&colorReader);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get color reader");
	}

	result = colorSource->get_FrameDescription(&frameDesc);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get frame description");
	}
}

void KinectLibrary::initDepth() {
	HRESULT result = sensor->get_DepthFrameSource(&depthSource);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get depth source");
	}

	result = depthSource->OpenReader(&depthReader);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get depth reader");
	}

	result = depthSource->get_FrameDescription(&frameDesc);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get frame description");
	}
}

void KinectLibrary::initInfrared() {
	HRESULT result = sensor->get_InfraredFrameSource(&infraredSource);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get infrared source");
	}

	result = infraredSource->OpenReader(&infraredReader);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get infrared reader");
	}

	result = infraredSource->get_FrameDescription(&frameDesc);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get frame description");
	}

}

void KinectLibrary::initBody() {
	HRESULT result = sensor->get_BodyFrameSource(&bodySource);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get body source");
	}

	result = bodySource->OpenReader(&bodyReader);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get body reader");
	}
}

KinectLibrary::~KinectLibrary() {
}

