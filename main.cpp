#include "KinectLibrary.h"
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>
int main() {
	
	KinectLibrary kl(COLOR_SENSOR | DEPTH_SENSOR);
	cv::Mat color, depth;
	IBody* body[BODY_COUNT] = { 0 };
	DepthSpacePoint* dsp = nullptr;
	cv::Mat mapped = cv::Mat::zeros(1080, 1920, CV_16UC1);
	cv::Rect mappedRect = cv::Rect(0, 0, 512, 424);

	while (1) {
		if (kl.update()) {
			if (kl.getDepthImage(depth) && kl.getColorImage(color)) {
				cv::imshow("Depth", depth);
				cv::imshow("Color", color);

				if (kl.getCoordinateMapping(kl.COLOR_TO_DEPTH, (void**)&dsp)) {
					if (!depth.empty()) {
						
						for (int i = 0; i < mapped.rows; i++) {
							for (int j = 0; j < mapped.cols; j++) {

								int curIndex = i * mapped.cols + j;
								DepthSpacePoint curCsp = dsp[curIndex];
								cv::Point pt = cv::Point((int)curCsp.X, (int)curCsp.Y);

								if (mappedRect.contains(pt)) {
									mapped.at<unsigned short>(i, j) = depth.at<unsigned short>(pt);
								}
							}
						}

						cv::imshow("mapped", mapped);
					}
				}
			}

			cv::waitKey(1);
		}
	}

	kl.~KinectLibrary();

	return 0;
}