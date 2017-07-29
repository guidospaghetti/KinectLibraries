
#define	COLOR_SENSOR		0x01
#define DEPTH_SENSOR		0x01 << 1
#define INFRARED_SENSOR		0x01 << 2
#define BODY_SENSOR			0x01 << 3

class KinectLibrary {

public:

	KinectLibrary(uint8_t sensors);

	~KinectLibrary();

private:

	IKinectSensor* sensor;

	IMultiSourceFrameReader* msfReader;

};

