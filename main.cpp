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

		cv::Mat depthFloat, validMask, depthFilled;

		depthImg.convertTo(depthFloat, CV_32F);

		double minValidDepth = 0.0;
		double maxValidDepth = 1500.0;
		validMask = (depthFloat >= minValidDepth) & (depthFloat <= maxValidDepth);
		depthFloat.setTo(std::numeric_limits<float>::quiet_NaN(), ~validMask);

		depthFilled = depthFloat.clone();
		cv::medianBlur(depthFilled, depthFilled, 5);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
		cv::dilate(depthFilled, depthFilled, kernel);
		depthFilled.copyTo(depthFloat, ~validMask);  // only fill invalid pixels

		cv::Mat depthClamped = depthFloat.clone();
		depthClamped.setTo(minValidDepth, depthClamped < minValidDepth);
		depthClamped.setTo(maxValidDepth, depthClamped > maxValidDepth);

		cv::Mat depthNormalized;
		cv::normalize(depthClamped, depthNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

		cv::Mat depthGamma;
		depthNormalized.convertTo(depthGamma, CV_32F, 1.0/255.0);
		cv::pow(depthGamma, 0.5, depthGamma);  // gamma < 1 brightens mid-range
		depthGamma.convertTo(depthNormalized, CV_8U, 255.0);

		cv::Mat depthColor;
		cv::applyColorMap(255 - depthNormalized, depthColor, cv::COLORMAP_JET);
		cv::imshow("Depth (colored)", depthColor);
		cv::waitKey(10);
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