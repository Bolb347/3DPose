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
    cv::StereoBM *m_stereoBM;
    cv::Mat m_cameraIntrinsics;
    double m_baseline;

    public:
    cv::Mat m_depthImg;
    time_t timestamp;

    StereoSolver(cv::VideoCapture *cam1stream, cv::VideoCapture *cam2stream, cv::Mat cameraIntrinsics, double baseline) {
        m_cam1stream = cam1stream;
        m_cam2stream = cam2stream;
        m_depthImg = cv::Mat();
        timestamp = time(nullptr);
        m_stereoBM = cv::StereoBM::create(16 * 5, 21);
        m_cameraIntrinsics = cameraIntrinsics;
        m_baseline = baseline;
    }

    ~StereoSolver() {
        delete m_stereoBM;
        delete m_cam1stream;
        delete m_cam2stream;
    }

    void solve() {
        cv::Mat img1, img2;
        *m_cam1stream >> img1;
        *m_cam2stream >> img2;
        cv::cvtColor(img1, img1, cv::COLOR_RGB2GRAY);
        cv::cvtColor(img2, img2, cv::COLOR_RGB2GRAY);
        timestamp = time(nullptr);
        m_stereoBM->compute(img1, img2, m_depthImg);
        m_depthImg.convertTo(m_depthImg, CV_32F, 1.0 / 16.0);
        cv::Mat mask = m_depthImg > 0.0f;
        m_depthImg.setTo(std::numeric_limits<float>::quiet_NaN(), ~mask);
        cv::divide(m_cameraIntrinsics.at<double>(0, 0) * m_baseline, m_depthImg, m_depthImg, 1, CV_32F);
    }
} StereoSolver;

#endif