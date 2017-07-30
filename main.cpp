#include "KinectLibrary.h"

int main() {
	
	KinectLibrary kl(COLOR_SENSOR | DEPTH_SENSOR);
	cv::Mat color, depth;
	kl.getColorImage(color);
	kl.getDepthImage(depth);

	kl.~KinectLibrary();
	return 0;
}