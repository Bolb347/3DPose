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
    cv::VideoCapture cam1("http://10.30.45.13:5801/image.mjpg");
    cv::VideoCapture cam2("http://10.30.45.14:5801/image.mjpg");
    StereoSolver *stereoSolver = new StereoSolver(
		&cam1, &cam2,
		(cv::Mat_<double>(3,3) << 
        	1000, 0,  2,
        	0,  1000, 2,
        	0,  0,  1),
		10
    );

	while (true)
	{
		std::vector<cv::Mat> imgs;
		cv::Mat colorImg;
		cv::Mat depthImg;

		cam1 >> colorImg;
        stereoSolver->solve();
        depthImg = stereoSolver->m_depthImg;

		imgs.push_back(colorImg);
		imgs.push_back(depthImg);

		std::vector<ObjectPose> objPose;
		poseDetect.detect(imgs, "coral.ply", 1, objPose, true);

		xPub.Set(objPose[0].translation.x);
		yPub.Set(objPose[0].translation.y);
		zPub.Set(objPose[0].translation.z);
	}
    delete stereoSolver;
}