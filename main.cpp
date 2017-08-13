#include "KinectLibrary.h"
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>
#include <iostream>

int main() {
	
	KinectLibrary kl(COLOR_SENSOR | DEPTH_SENSOR);
	cv::Mat color, depth;
	IBody* body[BODY_COUNT] = { 0 };
	DepthSpacePoint* dsp = nullptr;
	cv::Mat mapped = cv::Mat::zeros(1080, 1920, CV_16UC1);
	cv::Rect mappedRect = cv::Rect(0, 0, 512, 424);

	while (1) {
		int64 ticks = cv::getTickCount();
		if (kl.update()) {
			if (kl.getDepthImage(depth) && kl.getColorImage(color)) {
				cv::imshow("Depth", depth);
				cv::imshow("Color", color);

				if (kl.getCoordinateMapping(kl.DEPTH_TO_COLOR, mapped)) {
					cv::imshow("Mapped", mapped);
				}
			}

			cv::waitKey(1);
		}
		double time = ((double)cv::getTickCount() - ticks) / cv::getTickFrequency();
		std::cout << "Time taken: " << time << "\n";
	}

	kl.~KinectLibrary();

	return 0;
}