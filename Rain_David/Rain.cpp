#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 이미지 읽기
    cv::Mat img = cv::imread("../../Users/samsung/Desktop/image1.jpg");
    if (img.empty()) {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        return -1;
    }

    // 1. 그레이스케일 변환
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 2. 가우시안 블러로 노이즈 제거
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);

    // 3. Canny 엣지 검출
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    // 4. 컨투어 찾기
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 5. 컨투어 근사화 (모서리 추출)
    std::vector<std::vector<cv::Point>> approxContours;
    for (const auto& contour : contours) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);
        approxContours.push_back(approx);
    }

    // 6. 결과 시각화
    cv::Mat result = img.clone();
    for (const auto& approx : approxContours) {
        cv::polylines(result, approx, true, cv::Scalar(0, 255, 0), 2);
    }

    // 7. 결과 출력
    cv::imshow("Original Image", img);
    cv::imshow("Edges", edges);
    cv::imshow("Contours", result);

    cv::waitKey(0);
    return 0;
}
