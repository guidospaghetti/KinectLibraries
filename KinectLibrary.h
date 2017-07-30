#include "Kinect.h"
#include <stdint.h>
#include <opencv2\imgproc\imgproc.hpp>


#define	COLOR_SENSOR		(0x01)
#define DEPTH_SENSOR		(0x01 << 1)
#define INFRARED_SENSOR		(0x01 << 2)
#define BODY_SENSOR			(0x01 << 3)
#define BODY_INDEX_SENSOR	(0x01 << 4)

class KinectLibrary {

public:
	/**
	 * @brief KinectLibrary constructor. Performs the backend startup for 
	 * the Kinect API. The sensors available are color, depth, infrared, and
	 * body tracking. In order to use 
	 */
	KinectLibrary(uint8_t sensors);

	bool getColorImage(cv::Mat& output);

	bool getDepthImage(cv::Mat& output);

	bool getInfraredImage(cv::Mat& output);

	bool getBody(IBody** body);

	bool getBodyIndex(cv::Mat& output);

	~KinectLibrary();

private:

	uint8_t _sensors;

	IKinectSensor* sensor = NULL;

	IMultiSourceFrameReader* msfReader = NULL;

	IMultiSourceFrame* msf = NULL;

	IColorFrameSource* colorSource = NULL;

	IColorFrameReader* colorReader = NULL;

	IColorFrameReference* colorRef = NULL;

	IColorFrame* colorFrame = NULL;

	IDepthFrameSource* depthSource = NULL;
	
	IDepthFrameReader* depthReader = NULL;

	IDepthFrameReference* depthRef = NULL;

	IDepthFrame* depthFrame = NULL;

	IInfraredFrameSource* infraredSource = NULL;

	IInfraredFrameReader* infraredReader = NULL;

	IInfraredFrameReference* infraredRef = NULL;

	IInfraredFrame* infraredFrame = NULL;

	IBodyFrameSource* bodySource = NULL;
	
	IBodyFrameReader* bodyReader = NULL;

	IBodyFrameReference* bodyRef = NULL;

	IBodyFrame* bodyFrame = NULL;

	IBodyIndexFrameSource* bodyIndexSource = NULL;

	IBodyIndexFrameReader* bodyIndexReader = NULL;

	IBodyIndexFrameReference* bodyIndexRef = NULL;

	IBodyIndexFrame* bodyIndexFrame = NULL;

	IFrameDescription* frameDesc = NULL;

	int colorHeight;

	int colorWidth;
	
	int depthHeight;
	
	int depthWidth;
	
	int infraredHeight;
	
	int infraredWidth;

	int bodyIndexHeight;

	int bodyIndexWidth;

	int height;

	int width;

	UINT8* colorBuffer;

	UINT16* depthBuffer;

	UINT16* infraredBuffer;

	UINT8* bodyIndexBuffer;

	void initColor();

	void initDepth();

	void initInfrared();

	void initBody();

	void initBodyIndex();

	void initMSFSizes();
};

