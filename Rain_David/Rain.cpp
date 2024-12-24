#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

// 절대 깊이 계산 함수
double computeDepthWithTilt(double focalLength, double realHeight, double pixelHeight, double sensorHeight, int imageHeight, double deltaTiltAngle) {
    double sensorPixelRatio = sensorHeight / imageHeight;
    double objectHeightOnSensor = pixelHeight * sensorPixelRatio;
    double baseDepth = (focalLength * realHeight) / objectHeightOnSensor;

    // 기울기 보정
    return baseDepth * std::cos(deltaTiltAngle);
}

// 기울기 각도 계산 함수
double computeTiltAngle(const cv::Point2f& objectCenter, const cv::Point2f& imageCenter, double focalLengthPixels) {
    return std::atan2(objectCenter.y - imageCenter.y, focalLengthPixels);
}

// 객체 높이 및 중심 계산 함수
double getObjectHeight(const cv::Mat& image, cv::Point2f& objectCenter) {
    cv::Mat gray, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Canny(gray, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return -1.0;

    double maxArea = 0;
    std::vector<cv::Point> largestContour;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea) {
            maxArea = area;
            largestContour = contour;
        }
    }

    // 객체 중심 계산
    cv::Moments m = cv::moments(largestContour);
    objectCenter = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);

    // 객체 높이 계산
    int minY = INT_MAX, maxY = INT_MIN;
    for (const auto& point : largestContour) {
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }
    return static_cast<double>(maxY - minY);
}

int main() {
    // 정면 사진 로드
    cv::Mat frontImage = cv::imread("C:\\Users\\samsung\\Desktop\\img\\front.jpg");
    if (frontImage.empty()) {
        std::cerr << "Error: Could not load front image!" << std::endl;
        return -1;
    }

    // 기울어진 사진 로드
    cv::Mat tiltedImage = cv::imread("C:\\Users\\samsung\\Desktop\\img\\tilted.jpg");
    if (tiltedImage.empty()) {
        std::cerr << "Error: Could not load tilted image!" << std::endl;
        return -1;
    }

    // 메타데이터 기반 설정
    double focalLength = 4.65;      // mm
    double sensorHeight = 5.55;     // mm
    int imageHeight = 3000;         // px
    double realHeight = 1700;       // mm
    double focalLengthPixels = 3356; // 픽셀 단위 초점 거리

    // 정면 사진에서 객체 중심 및 높이 계산
    cv::Point2f frontObjectCenter;
    double frontPixelHeight = getObjectHeight(frontImage, frontObjectCenter);
    if (frontPixelHeight <= 0) {
        std::cerr << "Failed to calculate object height in front image." << std::endl;
        return -1;
    }

    // 기울어진 사진에서 객체 중심 및 높이 계산
    cv::Point2f tiltedObjectCenter;
    double tiltedPixelHeight = getObjectHeight(tiltedImage, tiltedObjectCenter);
    if (tiltedPixelHeight <= 0) {
        std::cerr << "Failed to calculate object height in tilted image." << std::endl;
        return -1;
    }

    // 이미지 중심 계산
    cv::Point2f imageCenter(frontImage.cols / 2.0, frontImage.rows / 2.0);

    // 각도 계산
    double frontTiltAngle = computeTiltAngle(frontObjectCenter, imageCenter, focalLengthPixels);
    double tiltedTiltAngle = computeTiltAngle(tiltedObjectCenter, imageCenter, focalLengthPixels);
    double deltaTiltAngle = std::abs(tiltedTiltAngle - frontTiltAngle);

    // 깊이 계산
    double depth = computeDepthWithTilt(focalLength, realHeight, frontPixelHeight, sensorHeight, imageHeight, deltaTiltAngle);
    std::cout << "Estimated Depth (with tilt correction): " << depth << " mm" << std::endl;

    return 0;
}
