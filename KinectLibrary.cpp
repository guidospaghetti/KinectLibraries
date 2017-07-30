#include "KinectLibrary.h"
#include "KinectLibraryException.h"
#include "Windows.h"
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

void KinectLibrary::getColorImage(cv::Mat output) {

	HRESULT result;

	if (!(_sensors & COLOR_SENSOR)) {
		throw KinectLibraryException("getColorImage called incorrectly");
	}
	
	if (!msfReader && !colorReader) {
		throw KinectLibraryException("getColorImage called incorrectly, call constructor first");
	}

	if (msfReader) {
		result = msfReader->AcquireLatestFrame(&msf);
		
		UINT8* colorBuffer = new UINT8[colorHeight * colorWidth * 4];
		
		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_ColorFrameReference(&colorRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get color frame reference");
		}

		result = colorRef->AcquireFrame(&colorFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get color frame");
		}

		result = colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, colorBuffer,ColorImageFormat::ColorImageFormat_Bgra);
		cv::Mat image = cv::Mat::zeros(colorHeight, colorWidth, CV_8UC4);
		image = cv::Mat(colorHeight, colorWidth, CV_8UC4, colorBuffer);

		output = image;
		delete[] colorBuffer;
		msf->Release();
		colorRef->Release();
		colorFrame->Release();

	}
	else if (colorReader) {

		UINT8* colorBuffer = new UINT8[height * width * 4];

		result = colorReader->AcquireLatestFrame(&colorFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get color frame");
		}

		result = colorFrame->CopyConvertedFrameDataToArray(height * width * 4, colorBuffer, ColorImageFormat::ColorImageFormat_Bgra);

		
		cv::Mat image = cv::Mat(height, width, CV_8UC4, colorBuffer);
		output = image;

		delete[] colorBuffer;
		colorRef->Release();
		colorFrame->Release();
	}
}

void KinectLibrary::getDepthImage(cv::Mat output) {
	
	HRESULT result;

	if (!(_sensors & DEPTH_SENSOR)) {
		throw KinectLibraryException("getDepthImage called incorrectly");
	}

	if (!msfReader && !depthReader) {
		throw KinectLibraryException("getDepthImage called incorrectly. Call constructor first");
	}

	if (msfReader) {

		UINT16* depthBuffer = new UINT16[depthHeight * depthWidth];

		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_DepthFrameReference(&depthRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get depth frame reference");
		}

		result = depthRef->AcquireFrame(&depthFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get depth frame");
		}

		result = depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, depthBuffer);
		
		cv::Mat image = cv::Mat(depthHeight, depthWidth, CV_16UC1, depthBuffer);
		output = image;

		delete[] depthBuffer;

		msf->Release();
		depthRef->Release();
		depthFrame->Release();
		
	}
	else if (depthReader) {
		UINT16* depthBuffer = new UINT16[height * width];

		result = depthReader->AcquireLatestFrame(&depthFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get depth frame");
		}
		
		result = depthFrame->CopyFrameDataToArray(height * width, depthBuffer);

		cv::Mat image = cv::Mat(height, width, CV_16UC1, depthBuffer);
		output = image;

		delete[] depthBuffer;
		
		msf->Release();
		depthRef->Release();
		depthFrame->Release();
		
	}
}

void KinectLibrary::getInfraredImage(cv::Mat output) {
	HRESULT result;

	if (!(_sensors & INFRARED_SENSOR)) {
		throw KinectLibraryException("getInfraredImage called incorrectly");
	}

	if (!msfReader && !infraredReader) {
		throw KinectLibraryException("getInfraredImage called incorrectly, call constructor first");
	}

	if (msfReader) {
		
		UINT16* infraredBuffer = new UINT16[infraredHeight * infraredWidth];

		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_InfraredFrameReference(&infraredRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get infrared frame reference");
		}

		result = infraredRef->AcquireFrame(&infraredFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get infrared frame");
		}

		result = infraredFrame->CopyFrameDataToArray(infraredHeight * infraredWidth, infraredBuffer);

		cv::Mat image = cv::Mat(infraredHeight, infraredWidth, CV_16UC1, infraredBuffer);
		output = image;

		delete[] infraredBuffer;

		msf->Release();
		infraredRef->Release();
		infraredFrame->Release();
	}
	else if (infraredReader) {

		UINT16* infraredBuffer = new UINT16[height * width];

		result = infraredReader->AcquireLatestFrame(&infraredFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get infrared frame");
		}

		result = infraredFrame->CopyFrameDataToArray(height * width, infraredBuffer);

		cv::Mat image = cv::Mat(height, width, CV_16UC1, infraredBuffer);
		output = image;
		
		delete[] infraredBuffer;

		infraredReader->Release();
		infraredFrame->Release();
	}
}

void KinectLibrary::getBody(IBody** body) {

	HRESULT result;

	if (!(_sensors & BODY_SENSOR)) {
		throw KinectLibraryException("getBody called incorrectly");
	}

	if (!msfReader && !bodyReader) {
		throw KinectLibraryException("getBody called incorrectly, call constructor first");
	}

	if (msfReader) {
		
		result = msfReader->AcquireLatestFrame(&msf);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get multi-source frame");
		}

		result = msf->get_BodyFrameReference(&bodyRef);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get body frame reference");
		}

		result = bodyRef->AcquireFrame(&bodyFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get body frame");
		}

		result = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, body);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get body data");
		}

		msf->Release();
		bodyRef->Release();
		bodyFrame->Release();
	}
	else if (bodyReader) {
		
		result = bodyReader->AcquireLatestFrame(&bodyFrame);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get body frame");
		}

		result = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, body);

		if (FAILED(result)) {
			throw KinectLibraryException("Unable to get body data");
		}

		bodyFrame->Release();
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

	bodyReader->AcquireLatestFrame(&bodyFrame);

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

