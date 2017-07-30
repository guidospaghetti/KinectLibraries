#include "KinectLibrary.h"
#include "KinectLibraryException.h"
#include "Windows.h"
#include <intrin.h>
#include <iostream>

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
		if (sensors & BODY_INDEX_SENSOR) {
			frameSourceTypes |= FrameSourceTypes::FrameSourceTypes_BodyIndex;
		}

		result = sensor->OpenMultiSourceFrameReader((DWORD)frameSourceTypes, &msfReader);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to open the multi-source frame reader");
		}
		
		while (!msf) {
			result = msfReader->AcquireLatestFrame(&msf);
		}

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get multi-source frame");
		}
		
		initMSFSizes();
		msf->Release();
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
		case BODY_INDEX_SENSOR:
			
			initBodyIndex();

			break;
		default:
			break;
		}
		
		if (frameDesc) {
			frameDesc->get_Height(&height);
			frameDesc->get_Width(&width);
			frameDesc->Release();
		}

	}

}

bool KinectLibrary::getColorImage(cv::Mat& output) {

	HRESULT result;

	if (!(_sensors & COLOR_SENSOR)) {
		return false;
		throw KinectLibraryException("getColorImage called incorrectly");
	}
	
	if (!msfReader && !colorReader) {
		return false;
		throw KinectLibraryException("getColorImage called incorrectly, call constructor first");
	}

	if (msfReader) {
		
		result = msfReader->AcquireLatestFrame(&msf);

		delete[] colorBuffer;
		colorBuffer = new UINT8[colorHeight * colorWidth * 4];
		
		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_ColorFrameReference(&colorRef);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get color frame reference");
		}

		result = colorRef->AcquireFrame(&colorFrame);

		if (FAILED(result)) {
			//throw KinectLibraryException("Unable to get color frame");
			return false;
		}

		result = colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, colorBuffer ,ColorImageFormat::ColorImageFormat_Bgra);
		
		output = cv::Mat(colorHeight, colorWidth, CV_8UC4, colorBuffer);

		msf->Release();
		colorRef->Release();
		colorFrame->Release();
		return true;

	}
	else if (colorReader) {

		delete[] colorBuffer;
		colorBuffer = new UINT8[height * width * 4];

		result = colorReader->AcquireLatestFrame(&colorFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get color frame");
		}

		result = colorFrame->CopyConvertedFrameDataToArray(height * width * 4, colorBuffer, ColorImageFormat::ColorImageFormat_Bgra);

		
		output = cv::Mat(height, width, CV_8UC4, colorBuffer);

		colorRef->Release();
		colorFrame->Release();
		return true;
	}

	return false;
}

bool KinectLibrary::getDepthImage(cv::Mat& output) {
	
	HRESULT result;

	if (!(_sensors & DEPTH_SENSOR)) {
		return false;
		throw KinectLibraryException("getDepthImage called incorrectly");
	}

	if (!msfReader && !depthReader) {
		return false;
		throw KinectLibraryException("getDepthImage called incorrectly. Call constructor first");
	}

	if (msfReader) {

		delete[] depthBuffer;
		depthBuffer = new UINT16[depthHeight * depthWidth];

		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_DepthFrameReference(&depthRef);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get depth frame reference");
		}

		result = depthRef->AcquireFrame(&depthFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get depth frame");
		}

		result = depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, depthBuffer);
		
		output = cv::Mat(depthHeight, depthWidth, CV_16UC1, depthBuffer);


		msf->Release();
		depthRef->Release();
		depthFrame->Release();
		return true;
		
	}
	else if (depthReader) {

		delete[] depthBuffer;
		depthBuffer = new UINT16[height * width];

		result = depthReader->AcquireLatestFrame(&depthFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get depth frame");
		}
		
		result = depthFrame->CopyFrameDataToArray(height * width, depthBuffer);

		output = cv::Mat(height, width, CV_16UC1, depthBuffer);
		
		msf->Release();
		depthRef->Release();
		depthFrame->Release();
		return true;
		
	}

	return false;
}

bool KinectLibrary::getInfraredImage(cv::Mat& output) {
	HRESULT result;

	if (!(_sensors & INFRARED_SENSOR)) {
		return false;
		throw KinectLibraryException("getInfraredImage called incorrectly");
	}

	if (!msfReader && !infraredReader) {
		return false;
		throw KinectLibraryException("getInfraredImage called incorrectly, call constructor first");
	}

	if (msfReader) {
		
		delete[] infraredBuffer;
		infraredBuffer = new UINT16[infraredHeight * infraredWidth];

		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_InfraredFrameReference(&infraredRef);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get infrared frame reference");
		}

		result = infraredRef->AcquireFrame(&infraredFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get infrared frame");
		}

		result = infraredFrame->CopyFrameDataToArray(infraredHeight * infraredWidth, infraredBuffer);

		output = cv::Mat(infraredHeight, infraredWidth, CV_16UC1, infraredBuffer);

		msf->Release();
		infraredRef->Release();
		infraredFrame->Release();
		return true;
	}
	else if (infraredReader) {

		delete[] infraredBuffer;
		infraredBuffer = new UINT16[height * width];

		result = infraredReader->AcquireLatestFrame(&infraredFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get infrared frame");
		}

		result = infraredFrame->CopyFrameDataToArray(height * width, infraredBuffer);

		output = cv::Mat(height, width, CV_16UC1, infraredBuffer);

		infraredReader->Release();
		infraredFrame->Release();
		return true;
	}

	return false;
}

bool KinectLibrary::getBody(IBody** body) {

	HRESULT result;

	if (!(_sensors & BODY_SENSOR)) {
		return false;
		throw KinectLibraryException("getBody called incorrectly");
	}

	if (!msfReader && !bodyReader) {
		return false;
		throw KinectLibraryException("getBody called incorrectly, call constructor first");
	}

	if (msfReader) {
		
		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_BodyFrameReference(&bodyRef);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body frame reference");
		}

		result = bodyRef->AcquireFrame(&bodyFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body frame");
		}

		result = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, body);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body data");
		}

		msf->Release();
		bodyRef->Release();
		bodyFrame->Release();

		return true;
	}
	else if (bodyReader) {
		
		result = bodyReader->AcquireLatestFrame(&bodyFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body frame");
		}

		result = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, body);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body data");
		}

		bodyFrame->Release();
		return true;
	}

	return false;
}

bool KinectLibrary::getBodyIndex(cv::Mat& output) {
	HRESULT result;

	if (!(_sensors & BODY_INDEX_SENSOR)) {
		return false;
		throw KinectLibraryException("getBodyIndex called incorrectly");
	}

	if (!msfReader && !bodyIndexReader) {
		return false;
		throw KinectLibraryException("getBodyIndex called incorrectly, call constructor first");
	}

	if (msfReader) {

		delete[] bodyIndexBuffer;
		bodyIndexBuffer = new UINT8[bodyIndexHeight * bodyIndexWidth];

		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_BodyIndexFrameReference(&bodyIndexRef);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body index frame reference");
		}

		result = bodyIndexRef->AcquireFrame(&bodyIndexFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body index frame");
		}
		
		result = bodyIndexFrame->CopyFrameDataToArray(bodyIndexHeight * bodyIndexWidth, bodyIndexBuffer);

		output = cv::Mat(bodyIndexHeight, bodyIndexWidth, CV_8UC1, bodyIndexBuffer);

		msf->Release();
		bodyIndexRef->Release();
		bodyIndexFrame->Release();
		return true;
	}
	else if (bodyIndexReader) {

		delete[] bodyIndexBuffer;
		bodyIndexBuffer = new UINT8[bodyIndexHeight * bodyIndexWidth];

		result = bodyIndexReader->AcquireLatestFrame(&bodyIndexFrame);

		if (FAILED(result)) {
			return false;
			throw KinectLibraryException("Unable to get body index frame");
		}

		result = bodyIndexFrame->CopyFrameDataToArray(bodyIndexHeight * bodyIndexWidth, bodyIndexBuffer);

		output = cv::Mat(bodyIndexHeight, bodyIndexWidth, CV_8UC1, bodyIndexBuffer);

		bodyIndexFrame->Release();
		return true;

	}

	return false;

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

void KinectLibrary::initBodyIndex() {
	HRESULT result = sensor->get_BodyIndexFrameSource(&bodyIndexSource);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get body index source");
	}

	result = bodyIndexSource->OpenReader(&bodyIndexReader);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get body index reader");
	}

	result = bodyIndexSource->get_FrameDescription(&frameDesc);

	if (FAILED(result)) {
		throw KinectLibraryException("Unable to get frame description");
	}
}

void KinectLibrary::initMSFSizes() {
	HRESULT result;

	if (_sensors & COLOR_SENSOR) {
		result = msf->get_ColorFrameReference(&colorRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get color frame reference");
		}

		result = colorRef->AcquireFrame(&colorFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get color frame");
		}

		result = colorFrame->get_FrameDescription(&frameDesc);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get color frame description");
		}

		frameDesc->get_Height(&colorHeight);
		frameDesc->get_Width(&colorWidth);

		colorRef->Release();
		colorFrame->Release();
		frameDesc->Release();
	}

	if (_sensors & DEPTH_SENSOR) {
		result = msf->get_DepthFrameReference(&depthRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get depth frame reference");
		}

		result = depthRef->AcquireFrame(&depthFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get depth frame");
		}

		result = depthFrame->get_FrameDescription(&frameDesc);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get depth frame description");
		}

		frameDesc->get_Height(&depthHeight);
		frameDesc->get_Width(&depthWidth);

		depthRef->Release();
		depthFrame->Release();
		frameDesc->Release();
	}

	if (_sensors & INFRARED_SENSOR) {
		result = msf->get_InfraredFrameReference(&infraredRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get infrared frame reference");
		}

		result = infraredRef->AcquireFrame(&infraredFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get infrared frame");
		}

		result = infraredFrame->get_FrameDescription(&frameDesc);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get infrared frame description");
		}

		frameDesc->get_Height(&infraredHeight);
		frameDesc->get_Width(&infraredWidth);

		infraredRef->Release();
		infraredFrame->Release();
		frameDesc->Release();
	}

	if (_sensors & BODY_INDEX_SENSOR) {
		result = msf->get_BodyIndexFrameReference(&bodyIndexRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get body index frame reference");
		}

		result = bodyIndexRef->AcquireFrame(&bodyIndexFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get body index frame");
		}

		result = bodyIndexFrame->get_FrameDescription(&frameDesc);

		if (FAILED(result)) {
			throw KinectLibraryException("Failed to get body index frame description");
		}

		frameDesc->get_Height(&bodyIndexHeight);
		frameDesc->get_Width(&bodyIndexWidth);

		bodyIndexRef->Release();
		bodyIndexFrame->Release();
		frameDesc->Release();
	}
}

KinectLibrary::~KinectLibrary() {
	if (sensor) {
		sensor->Release();
	}

	if (msfReader) {
		msfReader->Release();
	}

	if (msf) {
		msf->Release();
	}

	if (colorSource) {
		colorSource->Release();
	}

	if (colorReader) {
		colorReader->Release();
	}

	if (colorRef) {
		colorRef->Release();
	}

	if (colorFrame) {
		colorFrame->Release();
	}

	if (depthSource) {
		depthSource->Release();
	}

	if (depthReader) {
		depthReader->Release();
	}

	if (depthRef) {
		depthRef->Release();
	}

	if (depthFrame) {
		depthFrame->Release();
	}

	if (infraredSource) {
		infraredSource->Release();
	}

	if (infraredReader) {
		infraredReader->Release();
	}

	if (infraredRef) {
		infraredRef->Release();
	}

	if (infraredFrame) {
		infraredFrame->Release();
	}

	if (bodySource) {
		bodySource->Release();
	}

	if (bodyReader) {
		bodyReader->Release();
	}

	if (bodyRef) {
		bodyRef->Release();
	}

	if (bodyFrame) {
		bodyFrame->Release();
	}
}

