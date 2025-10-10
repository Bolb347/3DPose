#ifndef STEREO_H
#define STEREO_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <ctime>

typedef class StereoSolver {
    private:
    cv::VideoCapture *m_cam1stream;
    cv::VideoCapture *m_cam2stream;
    cv::Ptr<cv::StereoSGBM> m_stereoBM;
    cv::Mat m_cameraIntrinsics;
    double m_baseline;

    public:
    cv::Mat m_depthImg;
    long timestamp;

    StereoSolver(cv::VideoCapture *cam1stream, cv::VideoCapture *cam2stream, cv::Mat cameraIntrinsics, double baseline, int numDisparities, int blockSize) {
        m_cam1stream = cam1stream;
        m_cam2stream = cam2stream;
        m_depthImg = cv::Mat();
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        m_stereoBM = cv::StereoSGBM::create(0, numDisparities, blockSize,
            8*blockSize * blockSize,
            32*blockSize * blockSize,
            1,
            31,
            5,
            50,
            16,
            cv::StereoSGBM::MODE_SGBM_3WAY);
        m_cameraIntrinsics = cameraIntrinsics;
        m_baseline = baseline;
    }

    void solve() {
        if (m_stereoBM.empty()) {
            std::cerr << "Failed to create StereoBM!" << std::endl;
            return;
        }
        cv::Mat img1, img2;

        *m_cam1stream >> img1;
        *m_cam2stream >> img2;

        // Convert to grayscale if needed
        if (img1.channels() == 3)
            cv::cvtColor(img1, img1, cv::COLOR_BGR2GRAY);
            cv::equalizeHist(img1, img1);
        if (img2.channels() == 3)
            cv::cvtColor(img2, img2, cv::COLOR_BGR2GRAY);
            cv::equalizeHist(img2, img2);

        // Ensure type is CV_8UC1
        if (img1.type() != CV_8UC1 || img2.type() != CV_8UC1) {
            std::cerr << "[StereoSolver] Warning: images not CV_8UC1 after conversion\n";
            return;
        }

        // Timestamp
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                    ).count();
        
        m_stereoBM->compute(img1, img2, m_depthImg);

        // Convert disparity to float
        m_depthImg.convertTo(m_depthImg, CV_32F, 1.0 / 16.0);

        // Mask invalid pixels
        cv::Mat mask = m_depthImg > 0.0f;
        m_depthImg.setTo(std::numeric_limits<float>::quiet_NaN(), ~mask);

        // Compute depth from disparity
        cv::divide(m_cameraIntrinsics.at<double>(0, 0) * m_baseline, m_depthImg, m_depthImg, 1, CV_32F);
    }

} StereoSolver;

#endif
