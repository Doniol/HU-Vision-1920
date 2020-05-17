#include "StudentPreProcessing.h"
#include <opencv2/opencv.hpp>
#include <array>
#include "ImageFactory.h"
#include "HereBeDragons.h"
#include <math.h>


IntensityImage * StudentPreProcessing::stepToIntensityImage(const RGBImage &image) const {
	return nullptr;
}

IntensityImage * StudentPreProcessing::stepScaleImage(const IntensityImage &image) const {
	return nullptr;
}

IntensityImage * StudentPreProcessing::stepEdgeDetection(const IntensityImage &image) const {
	// Turn IntensityImage into an image useable by OpenCV
	cv::Mat base;
	HereBeDragons::HerLoveForWhoseDearLoveIRiseAndFall(image, base);

	// Apply Gaussian blur
	cv::Mat blurred;
	cv::GaussianBlur(base, blurred, cv::Size(5, 5), 0);


	// Apply Sobel
	// Calculate the image directional gradients with the Sobel method
	cv::Mat sobel(blurred.rows, blurred.cols, CV_8UC1, cv::Scalar(0, 0));

	// Apply masks
	for (int i = 1; i < blurred.rows - 1; i++) {
		for (int j = 1; j < blurred.cols - 1; j++) {
			int pixel_x = -(int)blurred.at<uchar>(i - 1, j - 1) +
				-2 * (int)blurred.at<char>(i - 1, j) +
				-(int)blurred.at<char>(i - 1, j + 1) +
				(int)blurred.at<char>(i + 1, j - 1) +
				2 * (int)blurred.at<char>(i + 1, j) +
				(int)blurred.at<char>(i + 1, j + 1);

			int pixel_y = -(int)blurred.at<uchar>(i - 1, j - 1) +
				-2 * (int)blurred.at<uchar>(i, j - 1) +
				-(int)blurred.at<uchar>(i + 1, j - 1) +
				(int)blurred.at<uchar>(i - 1, j + 1) +
				2 * (int)blurred.at<uchar>(i, j + 1) +
				(int)blurred.at<uchar>(i + 1, j + 1);

			sobel.at<uchar>(i, j) = sqrt(pixel_x + pixel_y);
		}
	}

	// Apply Non-Maximum Suppression
	// Note all possible directions for edge
	cv::Mat suppressed(sobel.rows, sobel.cols, CV_8UC1, cv::Scalar(0, 0));
	int angles[4][2] = { {1, -1}, {1, 0}, {1, 1}, {0, 1} };

	// Loop through all pixels, compare them with their neighbours of a certain direction, and deal with them accordingly
	for (int count = 0; count < 4; count++) {
		auto angle = angles[count];
		for (int i = 1; i < sobel.rows - 1; i++) {
			for (int j = 1; j < sobel.cols - 1; j++) {
				// Get grayscale value of to be compared pixels
				int selected = (int)sobel.at<uchar>(i, j);
				int next = (int)sobel.at<uchar>(i + angle[0], j + angle[1]);
				int prev = (int)sobel.at<uchar>(i - angle[0], j - angle[1]);
				// If selected pixel is better than its neighbours, keep it, otherwise throw the pixel away
				if (selected >= next && selected >= prev) {
					suppressed.at<uchar>(i, j) = selected;
				}
				else {
					suppressed.at<uchar>(i, j) = 0;
				}
			}
		}
	}

	// Apply double thresholding
	// Determine thresholds based on the max pixel value in the image
	int high_threshold, low_threshold;
	double min, max;
	cv::minMaxLoc(suppressed, &min, &max);
	high_threshold = (int)(max * 0.5);
	low_threshold = (int)(high_threshold * 0.3);

	// Create black image for final result and list for the coming weak edges
	cv::Mat thresholded(suppressed.rows, suppressed.cols, CV_8UC1, cv::Scalar(0, 0));
	std::vector<std::array<int, 2>> weak_edges;

	// Loop through suppressed image pixels, if pixel > high_threshold it's a storng pixel that will stay, if pixel > low_threshold
	// it's a weak pixel that will stay and if pixel < both thresholds it will be ignored
	for (int i = 0; i < suppressed.rows; i++) {
		for (int j = 0; j < suppressed.cols; j++) {
			int selected = (int)suppressed.at<uchar>(i, j);
			if (selected >= high_threshold) {
				thresholded.at<uchar>(i, j) = 255;
			}
			else if (selected >= low_threshold) {
				thresholded.at<uchar>(i, j) = 123;
				weak_edges.push_back({ i, j });
			}
		}
	}


	// Apply edge tracking by Hysteresis
	cv::Mat edge_tracking = thresholded;
	// Loop through all weak edges
	for (unsigned int weak = 0; weak < weak_edges.size(); weak++) {
		int i = weak_edges[weak][0];
		int j = weak_edges[weak][1];

		// If weak edge is bordered by a strong edge, this weak edge is now a strong edge
		if (edge_tracking.at<uchar>(i + 1, j) == 255 || edge_tracking.at<uchar>(i - 1, j) == 255 ||
			edge_tracking.at<uchar>(i + 1, j + 1) == 255 || edge_tracking.at<uchar>(i - 1, j - 1) == 255 ||
			edge_tracking.at<uchar>(i, j + 1) == 255 || edge_tracking.at<uchar>(i, j - 1) == 255 ||
			edge_tracking.at<uchar>(i - 1, j + 1) == 255 || edge_tracking.at<uchar>(i + 1, j - 1) == 255) {
			edge_tracking.at<uchar>(i, j) = 255;
		}
		// Else remove the weak edge
		else {
			edge_tracking.at<uchar>(i, j) = 0;
		}
	}

	cv::namedWindow("Test", CV_WINDOW_AUTOSIZE);
	cv::imshow("Test", thresholded);
	cvWaitKey(0);

	// Turn OpenCV image back into IntensityImage
	IntensityImage* result = ImageFactory::newIntensityImage();
	HereBeDragons::NoWantOfConscienceHoldItThatICall(edge_tracking, *result);

	return result;
}

IntensityImage * StudentPreProcessing::stepThresholding(const IntensityImage &image) const {
	return nullptr;
}