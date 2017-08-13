#include "Kinect.h"
#include <stdint.h>
#include <opencv2\imgproc\imgproc.hpp>


#define	COLOR_SENSOR		0x01
#define DEPTH_SENSOR		0x02
#define INFRARED_SENSOR		0x04
#define BODY_SENSOR			0x08
#define BODY_INDEX_SENSOR	0x10

class KinectLibrary {

public:

	typedef enum CoordinateMapping_t {
		CAMERA_TO_COLOR,
		CAMERA_TO_DEPTH,
		COLOR_TO_DEPTH,
		COLOR_TO_CAMERA,
		DEPTH_TO_CAMERA,
		DEPTH_TO_COLOR
	} CoordinateMapping_t;

	/**
	 * @brief KinectLibrary constructor. Performs the backend startup for 
	 * the Kinect API. The sensors available are color, depth, infrared, and
	 * body tracking. In order to use 
	 */
	KinectLibrary(uint8_t sensors);

	bool update();

	bool getColorImage(cv::Mat& output);

	bool getDepthImage(cv::Mat& output);

	bool getInfraredImage(cv::Mat& output);

	bool getBody(IBody** body);

	bool getBodyIndex(cv::Mat& output);

	bool getCoordinateMapping(CoordinateMapping_t mapping, void** points);

	bool getCoordinateMapping(CoordinateMapping_t mapping, cv::Mat& output);

	bool fillDepthHoles(cv::Mat& input, cv::Mat& output);

	~KinectLibrary();



private:

	uint8_t _sensors;

	bool multisource = false;

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

	ICoordinateMapper* mapper = NULL;

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

	CameraSpacePoint* cameraPts;

	DepthSpacePoint* depthPts;
	
	ColorSpacePoint* colorPts;

	cv::Mat colorImg;

	cv::Mat depthImg;

	cv::Mat infraredImg;

	cv::Mat bodyIndexImg;

	void initColor();

	void initDepth();

	void initInfrared();

	void initBody();

	void initBodyIndex();

	void initMSFSizes();
};

