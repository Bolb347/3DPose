#include "PoseDetection.h"
#include "Stereo.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <array>

int main()
{
	nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
	inst.StartServer();
	nt::DoublePublisher xPub = inst.GetDoubleTopic("pose/x").Publish();
	nt::DoublePublisher yPub = inst.GetDoubleTopic("pose/y").Publish();
	nt::DoublePublisher zPub = inst.GetDoubleTopic("pose/z").Publish();
	xPub.SetDefault(0.0);
	yPub.SetDefault(0.0);
	zPub.SetDefault(0.0);
	PoseDetection poseDetect;
    cv::VideoCapture cam1(0, cv::CAP_AVFOUNDATION);
    cv::VideoCapture cam2(1, cv::CAP_AVFOUNDATION);
    StereoSolver *stereoSolver = new StereoSolver(
		&cam1, &cam2,
		(cv::Mat_<double>(3,3) << 
        	1000, 0,  2,
        	0,  1000, 2,
        	0,  0,    1),
		10,
		16 * 16,
		9
    );

	cv::Mat tmp;
    for (int i = 0; i < 10; ++i) {
        cam1 >> tmp;
        cam2 >> tmp;
        cv::waitKey(30);
    }
	
	while (true)
	{
		std::vector<cv::Mat> imgs;
		cv::Mat colorImg;
		cv::Mat depthImg;

		cam1 >> colorImg;

		if (colorImg.empty()) {
			continue;
		}

        stereoSolver->solve();
        depthImg = stereoSolver->m_depthImg;

		// --- Depth visualization (robust version) ---
		cv::Mat depthFloat, validMask, depthClamped, depthNormalized, depthColor;

		// Convert to float for safe math
		depthImg.convertTo(depthFloat, CV_32F);

		// Create a mask of valid pixels (ignore 0, negative, NaN, or inf)
		validMask = (depthFloat > 0) & (depthFloat < 5000); // adjust max range as needed

		// Replace invalid pixels with NaN for better min/max detection
		depthFloat.setTo(std::numeric_limits<float>::quiet_NaN(), ~validMask);

		// Compute min/max only on valid values
		double minVal, maxVal;
		cv::minMaxLoc(depthFloat, &minVal, &maxVal, nullptr, nullptr, validMask);

		// Guard against empty/invalid ranges
		if (std::isnan(minVal) || std::isnan(maxVal) || minVal == maxVal)
		{
			std::cout << "Invalid depth frame (no valid data)" << std::endl;
			cv::imshow("Depth (colored)", cv::Mat::zeros(depthImg.size(), CV_8UC3));
			continue;
		}

		// Optional: clamp max depth for better visibility
		double maxDisplayDepth = std::min(maxVal, 3000.0); // limit to 3m for visualization

		// Clamp values and normalize
		depthClamped = depthFloat.clone();
		depthClamped.setTo(maxDisplayDepth, depthFloat > maxDisplayDepth);
		cv::normalize(depthClamped, depthNormalized, 0, 255, cv::NORM_MINMAX, CV_8U, validMask);

		// Invert so near = red, far = blue
		cv::applyColorMap(255 - depthNormalized, depthColor, cv::COLORMAP_JET);

		// Fill invalid areas with black for clarity
		depthColor.setTo(cv::Scalar(0, 0, 0), ~validMask);

		cv::imshow("Depth (colored)", depthColor);

		cv::waitKey(1);
		continue;

		imgs.push_back(colorImg);
		imgs.push_back(depthImg);

		if (depthImg.empty()) {
			continue;
		}

		std::vector<ObjectPose> objPose;
		poseDetect.detect(imgs, "coral.ply", 1, objPose, true);

		if (!objPose.empty()) {
			xPub.Set(objPose[0].translation.x);
			yPub.Set(objPose[0].translation.y);
			zPub.Set(objPose[0].translation.z);
		}

		cv::waitKey(10);
	}
	return 0;
}