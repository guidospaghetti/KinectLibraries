#include "KinectLibrary.h"
#include <opencv2\highgui.hpp>
int main() {
	
	KinectLibrary kl(COLOR_SENSOR | BODY_INDEX_SENSOR);
	cv::Mat color, bodyIndex;
	while (1) {
		if (kl.getColorImage(color)) {
			cv::imshow("Image", color);
			cv::waitKey(1);
		}
		//if (kl.getBodyIndex(bodyIndex)) {
		//	cv::imshow("BodyIndex", bodyIndex);
		//	cv::waitKey(1);
		//}
	}
	kl.~KinectLibrary();

	return 0;
}