#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// ─── 設定區（只需要改這裡）──────────────────────────
const int   BOARD_W      = 9;
const int   BOARD_H      = 6;
const float SQUARE_SIZE  = 25.0f;
const std::string IMAGE_DIR = "../images";
const std::string OUTPUT    = "../calib_result.yaml";
// ────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    // 解析命令列參數
    bool useFisheye = false;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--fisheye")
            useFisheye = true;
    }
    std::cout << "校正模式: "
              << (useFisheye ? "魚眼 (fisheye)" : "標準 (standard)") << "\n";

    cv::Size boardSize(BOARD_W, BOARD_H);

    // 1. 建立理想 3D 角點座標
    std::vector<cv::Point3f> objTemplate;
    for (int r = 0; r < BOARD_H; r++)
        for (int c = 0; c < BOARD_W; c++)
            objTemplate.emplace_back(c * SQUARE_SIZE,
                                     r * SQUARE_SIZE, 0.0f);

    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;

    // 2. 讀取圖片，找角點
    std::vector<std::string> paths;
    for (auto& entry : fs::directory_iterator(IMAGE_DIR)) {
        std::string ext = entry.path().extension().string();
        if (ext == ".jpg" || ext == ".png" || ext == ".jpeg")
            paths.push_back(entry.path().string());
    }
    std::sort(paths.begin(), paths.end());

    if (paths.empty()) {
        std::cerr << "錯誤：images/ 資料夾裡沒有圖片！\n";
        return 1;
    }

    std::cout << "找到 " << paths.size() << " 張圖片，開始偵測角點...\n";

    cv::Size imgSize;
    int successCount = 0;

    for (auto& path : paths) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) {
            std::cout << "  跳過（讀取失敗）: " << path << "\n";
            continue;
        }

        imgSize = img.size();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS +
                                 cv::TermCriteria::MAX_ITER, 30, 0.001));
            objPoints.push_back(objTemplate);
            imgPoints.push_back(corners);
            successCount++;
            std::cout << "  ✓ " << fs::path(path).filename().string() << "\n";
        } else {
            std::cout << "  ✗ 找不到角點: "
                      << fs::path(path).filename().string() << "\n";
        }
    }

    std::cout << "\n成功偵測：" << successCount
              << " / " << paths.size() << " 張\n";

    if (successCount < 4) {
        std::cerr << "錯誤：至少需要 4 張成功偵測的圖片才能校正。\n";
        return 1;
    }

    // 3. 執行相機校正
    std::cout << "\n執行校正中...\n";
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rpe = 0;

    if (useFisheye) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs   = cv::Mat::zeros(4, 1, CV_64F);
        rpe = cv::fisheye::calibrate(objPoints, imgPoints, imgSize,
            cameraMatrix, distCoeffs, rvecs, tvecs,
            cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
            cv::fisheye::CALIB_FIX_SKEW);
    } else {
        rpe = cv::calibrateCamera(objPoints, imgPoints, imgSize,
            cameraMatrix, distCoeffs, rvecs, tvecs);
    }

    std::cout << "重投影誤差 (RPE): " << rpe << " px\n";
    std::cout << "(RPE < 1.0 代表校正品質良好)\n\n";
    std::cout << "=== 內參矩陣 (K) ===\n" << cameraMatrix << "\n\n";
    std::cout << "=== 畸變係數 ===\n" << distCoeffs << "\n\n";

    // 4. 計算每張圖的 RPE 並畫長條圖
    std::cout << "=== 每張圖的重投影誤差 ===\n";
    std::vector<double> perImageRPE;

    for (int i = 0; i < (int)objPoints.size(); i++) {
        std::vector<cv::Point2f> reprojected;

        if (useFisheye) {
            cv::fisheye::projectPoints(objPoints[i], reprojected,
                rvecs[i], tvecs[i], cameraMatrix, distCoeffs);
        } else {
            cv::projectPoints(objPoints[i], rvecs[i], tvecs[i],
                cameraMatrix, distCoeffs, reprojected);
        }

        double err = 0;
        for (int j = 0; j < (int)reprojected.size(); j++) {
            double dx = reprojected[j].x - imgPoints[i][j].x;
            double dy = reprojected[j].y - imgPoints[i][j].y;
            err += std::sqrt(dx*dx + dy*dy);
        }
        err /= reprojected.size();
        perImageRPE.push_back(err);

        std::string fname = fs::path(paths[i]).filename().string();
        std::cout << "  " << fname << ": " << err << " px";
        if (err > 1.0) std::cout << "  ← 品質差";
        std::cout << "\n";
    }

    // 畫長條圖
    int barW = 60, barH = 400, margin = 60;
    int imgW = margin + (int)perImageRPE.size() * (barW + 10) + margin;
    int imgH = barH + margin * 2;
    cv::Mat chart(imgH, imgW, CV_8UC3, cv::Scalar(245, 245, 245));

    double maxRPE = *std::max_element(perImageRPE.begin(), perImageRPE.end());
    maxRPE = std::max(maxRPE, 1.0);

    int threshY = imgH - margin - (int)(1.0 / maxRPE * barH);
    cv::line(chart, {margin, threshY}, {imgW - margin, threshY},
        cv::Scalar(200, 80, 80), 1, cv::LINE_AA);
    cv::putText(chart, "1.0px threshold", {margin + 4, threshY - 6},
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 80, 80), 1);

    for (int i = 0; i < (int)perImageRPE.size(); i++) {
        int x = margin + i * (barW + 10);
        int h = (int)(perImageRPE[i] / maxRPE * barH);
        int y = imgH - margin - h;

        cv::Scalar color = (perImageRPE[i] > 1.0)
            ? cv::Scalar(80, 80, 200)
            : cv::Scalar(80, 180, 80);

        cv::rectangle(chart,
            cv::Point(x, y), cv::Point(x + barW, imgH - margin),
            color, -1);

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << perImageRPE[i];
        cv::putText(chart, ss.str(), {x + 4, y - 6},
            cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(50, 50, 50), 1);

        cv::putText(chart, std::to_string(i+1),
            {x + barW/2 - 6, imgH - margin + 18},
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(80, 80, 80), 1);
    }

    std::string chartTitle = std::string("RPE per Image - ") +
                             (useFisheye ? "Fisheye" : "Standard") + " model (px)";
    cv::putText(chart, chartTitle, {margin, 30},
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(40, 40, 40), 1);

    std::string chartPath = useFisheye
        ? "../rpe_fisheye.png" : "../rpe_per_image.png";
    cv::imwrite(chartPath, chart);
    std::cout << "\n長條圖已儲存至: " << chartPath << "\n";

    // 5. 存成 YAML
    cv::FileStorage fs_out(OUTPUT, cv::FileStorage::WRITE);
    fs_out << "image_width"        << imgSize.width;
    fs_out << "image_height"       << imgSize.height;
    fs_out << "model"              << (useFisheye ? "fisheye" : "standard");
    fs_out << "camera_matrix"      << cameraMatrix;
    fs_out << "dist_coeffs"        << distCoeffs;
    fs_out << "reprojection_error" << rpe;
    fs_out.release();
    std::cout << "結果已儲存至: " << OUTPUT << "\n";

    // 6. 輸出校正前後對比圖
    cv::Mat sample = cv::imread(paths[0]);
    cv::Mat undistorted;

    if (useFisheye) {
        cv::Mat newK;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            cameraMatrix, distCoeffs, imgSize,
            cv::Matx33d::eye(), newK);
        cv::fisheye::undistortImage(sample, undistorted,
            cameraMatrix, distCoeffs, newK);
    } else {
        cv::undistort(sample, undistorted, cameraMatrix, distCoeffs);
    }

    cv::Mat comparison;
    cv::hconcat(sample, undistorted, comparison);
    cv::putText(comparison, "Before",
        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
        1.2, cv::Scalar(0, 0, 255), 2);
    cv::putText(comparison, "After (undistorted)",
        cv::Point(sample.cols + 20, 40), cv::FONT_HERSHEY_SIMPLEX,
        1.2, cv::Scalar(0, 255, 0), 2);

    cv::imwrite("../comparison.jpg", comparison);
    std::cout << "對比圖已儲存至: comparison.jpg\n";

    return 0;
}