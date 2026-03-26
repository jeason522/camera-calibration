#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// ─── 設定區（只需要改這裡）──────────────────────────
const int   BOARD_W      = 9;      // 棋盤格內角點：寬
const int   BOARD_H      = 6;      // 棋盤格內角點：高
const float SQUARE_SIZE  = 25.0f;  // 實體方格邊長（mm），不確定就填 25
const std::string IMAGE_DIR = "../images";   // 圖片資料夾
const std::string OUTPUT    = "../calib_result.yaml"; // 輸出檔
// ────────────────────────────────────────────────────

int main() {
    cv::Size boardSize(BOARD_W, BOARD_H);

    // 1. 建立理想 3D 角點座標（Z=0 平面）
    std::vector<cv::Point3f> objTemplate;
    for (int r = 0; r < BOARD_H; r++)
        for (int c = 0; c < BOARD_W; c++)
            objTemplate.emplace_back(c * SQUARE_SIZE,
                                     r * SQUARE_SIZE, 0.0f);

    std::vector<std::vector<cv::Point3f>> objPoints; // 3D
    std::vector<std::vector<cv::Point2f>> imgPoints; // 2D

    // 2. 讀取所有圖片，找角點
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
        if (img.empty()) { std::cout << "  跳過（讀取失敗）: " << path << "\n"; continue; }

        imgSize = img.size();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            // 精細化角點位置
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            objPoints.push_back(objTemplate);
            imgPoints.push_back(corners);
            successCount++;
            std::cout << "  ✓ " << fs::path(path).filename().string() << "\n";
        } else {
            std::cout << "  ✗ 找不到角點: " << fs::path(path).filename().string() << "\n";
        }
    }

    std::cout << "\n成功偵測：" << successCount << " / " << paths.size() << " 張\n";

    if (successCount < 4) {
        std::cerr << "錯誤：至少需要 4 張成功偵測的圖片才能校正。\n";
        return 1;
    }

    // 3. 執行相機校正
    std::cout << "\n執行校正中...\n";
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rpe = cv::calibrateCamera(objPoints, imgPoints, imgSize,
                                     cameraMatrix, distCoeffs,
                                     rvecs, tvecs);

    std::cout << "重投影誤差 (RPE): " << rpe << " px\n";
    std::cout << "(RPE < 1.0 代表校正品質良好)\n\n";

    // 4. 印出結果
    std::cout << "=== 內參矩陣 (K) ===\n" << cameraMatrix << "\n\n";
    std::cout << "=== 畸變係數 (k1 k2 p1 p2 k3) ===\n" << distCoeffs << "\n\n";

    // 5. 存成 YAML
    cv::FileStorage fs_out(OUTPUT, cv::FileStorage::WRITE);
    fs_out << "image_width"   << imgSize.width;
    fs_out << "image_height"  << imgSize.height;
    fs_out << "camera_matrix" << cameraMatrix;
    fs_out << "dist_coeffs"   << distCoeffs;
    fs_out << "reprojection_error" << rpe;
    fs_out.release();
    std::cout << "結果已儲存至: " << OUTPUT << "\n";

    // 6. 輸出一張校正前後對比圖
    cv::Mat sample = cv::imread(paths[0]);
    cv::Mat undistorted;
    cv::undistort(sample, undistorted, cameraMatrix, distCoeffs);

    // 並排合成
    cv::Mat comparison;
    cv::hconcat(sample, undistorted, comparison);

    // 加標籤
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