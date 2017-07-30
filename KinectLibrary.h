#include "Kinect.h"
#include <stdint.h>
#include <opencv2\core\core.hpp>

#define	COLOR_SENSOR		0x01
#define DEPTH_SENSOR		0x01 << 1
#define INFRARED_SENSOR		0x01 << 2
#define BODY_SENSOR			0x01 << 3

class KinectLibrary {

public:
	KinectLibrary(uint8_t sensors);

	~KinectLibrary();

private:

	uint8_t _sensors;

	IKinectSensor* sensor;

	IMultiSourceFrameReader* msfReader;

	IMultiSourceFrame* msf;

	IColorFrameSource* colorSource;

	IColorFrameReader* colorReader;

	IColorFrameReference* colorRef;

	IColorFrame* colorFrame;

	IDepthFrameSource* depthSource;
	
	IDepthFrameReader* depthReader;

	IDepthFrameReference* depthRef;

	IDepthFrame* depthFrame;

	IInfraredFrameSource* infraredSource;

	IInfraredFrameReader* infraredReader;

	IInfraredFrameReference* infraredRef;

	IInfraredFrame* infraredFrame;

	IBodyFrameSource* bodySource;
	
	IBodyFrameReader* bodyReader;

	IBodyFrameReference* bodyRef;

	IBodyFrame* bodyFrame;

	IFrameDescription* frameDesc;

	int colorHeight;

	int colorWidth;
	
	int depthHeight;
	
	int depthWidth;
	
	int infraredHeight;
	
	int infraredWidth;

	int height;

	int width;

	void initColor();

	void initDepth();

	void initInfrared();

	void initBody();
};

