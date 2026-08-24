#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

const string LEFT_FRAME_PATH = "calibration/images/left"; 
const string RIGHT_FRAME_PATH = "calibration/images/right";

const int CHECKERBOARD_WIDTH = 8;
const int CHECKERBOARD_HEIGHT = 5;
const float SQUARE_SIZE = 0.025f;
const Size boardSize(CHECKERBOARD_WIDTH, CHECKERBOARD_HEIGHT);

void create_folders();
void camera_stereo_calibration();
void capture_frames();

class CalibrationNode: public rclcpp::Node{
    public:
        CalibrationNode(): Node("calibrarion_node"){
            create_folders();
            capture_frames();
            camera_stereo_calibration();
        }
    private :
};

void create_folders(){
    try {
        if (!fs::exists(LEFT_FRAME_PATH)) {
            fs::create_directories(LEFT_FRAME_PATH);
            cout << "Folder created : " << LEFT_FRAME_PATH << endl;
        }
        if (!fs::exists(RIGHT_FRAME_PATH)) {
            fs::create_directories(RIGHT_FRAME_PATH);
            cout << "Folder created : " << RIGHT_FRAME_PATH << endl;
        }
    } catch (const fs::filesystem_error& e) {
        cerr << "Error with folders creation : " << e.what() << endl;
    }
}

void camera_stereo_calibration(){
    vector<vector<Point3f>> objpoints; 
    vector<vector<Point2f>> imgpointsL, imgpointsR; 
    vector<Point3f> objp;
    vector<String> imagesL, imagesR;

    Mat frameL, frameR, grayL, grayR;
    Mat K1, D1, K2, D2;
    Mat R, T, E, F; 
    K1 = Mat::eye(3, 3, CV_64F);
    K2 = Mat::eye(3, 3, CV_64F);

    for (int i = 0; i < CHECKERBOARD_HEIGHT; i++) {
        for (int j = 0; j < CHECKERBOARD_WIDTH; j++) {
            objp.push_back(Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0));
        }
    }

    glob("calibration/images/left/*.png", imagesL); 
    glob("calibration/images/right/*.png", imagesR);

    sort(imagesL.begin(), imagesL.end());
    sort(imagesR.begin(), imagesR.end());

    if (imagesL.size() != imagesR.size() || imagesL.empty()) {
        cerr << "Error : The number of images are different or folders are empty." << endl;
        return;
    }

    for (size_t i = 0; i < imagesL.size(); i++) {
        frameL = imread(imagesL[i]);
        frameR = imread(imagesR[i]);

        if (frameL.empty() || frameR.empty()) continue;

        cvtColor(frameL, grayL, COLOR_BGR2GRAY);
        cvtColor(frameR, grayR, COLOR_BGR2GRAY);

        vector<Point2f> cornersL, cornersR;

        bool foundL = findChessboardCorners(grayL, boardSize, cornersL, 
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        bool foundR = findChessboardCorners(grayR, boardSize, cornersR, 
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (foundL && foundR) {
            cornerSubPix(grayL, cornersL, Size(11, 11), Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));
            cornerSubPix(grayR, cornersR, Size(11, 11), Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));

            imgpointsL.push_back(cornersL);
            imgpointsR.push_back(cornersR);
            objpoints.push_back(objp);

            drawChessboardCorners(frameL, boardSize, cornersL, foundL);
            imshow("Detection Left", frameL);
            waitKey(10);
        }
    }

    destroyAllWindows();

    if (imgpointsL.empty()) {
        cerr << "No pair found for the calibration." << endl;
        return;
    }

    double rms = stereoCalibrate(objpoints, imgpointsL, imgpointsR, K1, D1, K2, D2, frameL.size(), R, T, E, F, CALIB_RATIONAL_MODEL, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    cout << "Calibration finished : RMS Error: " << rms << endl;
    cout << "Matrix (K1):\n" << K1 << endl;
    cout << "Matrix (K2):\n" << K2 << endl;
    cout << "Translation (T):\n" << T << endl;
    cout << "Rotation (R):\n" << R << endl;

    FileStorage fs("calibration/stereo_calib.xml", FileStorage::WRITE);
    fs << "K1" << K1 << "D1" << D1;
    fs << "K2" << K2 << "D2" << D2;
    fs << "R" << R << "T" << T;
    fs.release();

    cout << "Settings saved in 'calibration/stereo_calib.xml'" << endl;
}

void capture_frames(){
    VideoCapture cap(0, CAP_V4L2);
    
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 60);

    if (!cap.isOpened()) {
        cerr << "Error : Impossible to open the camera" << endl;
        return;
    }

    namedWindow("Left", WINDOW_NORMAL);
    namedWindow("Right", WINDOW_NORMAL);

    resizeWindow("Left", 640, 640);
    resizeWindow("Right", 640, 640);

    Mat frame, left_img, right_img;
    int capture_count = 0;

    cout << "Press 's' to save a frame or 'q' to exit." << endl;

    while (true) {
        cap >> frame;

        if (frame.empty()) {
            cerr << "Error: Could not capture a frame" << endl;
            break;
        }

        int width = frame.cols;
        int height = frame.rows;
        int mid = width / 2;

        if (width == 0 || height == 0) continue;

        Rect left_roi(0, 0, mid, height);
        Rect right_roi(mid, 0, width - mid, height);

        left_img = frame(left_roi);
        right_img = frame(right_roi);

        imshow("Left", left_img);
        imshow("Right", right_img);

        char key = (char)waitKey(1); 

        if (key == 'q') {
            break;
        }
        else if (key == 's') {
            try {
                string left_filename = LEFT_FRAME_PATH + "/" + to_string(capture_count) + "_frame.png";
                string right_filename = RIGHT_FRAME_PATH + "/" + to_string(capture_count) + "_frame.png";

                if (!imwrite(left_filename, left_img)) {
                    throw runtime_error("imwrite failed for left image.");
                }
                cout << "Frame saved (" << capture_count << ") : " << left_filename << endl;

                if (!imwrite(right_filename, right_img)) {
                    throw runtime_error("imwrite failed for right image.");
                }
                cout << "Frame saved (" << capture_count << ") : " << right_filename << endl;

                capture_count++;

                if (capture_count >= 20) {
                    cout << "Capture limit reached (20 frames)." << endl;
                    break;
                }
            }
            catch (const std::exception& e) {
                cerr << "Error while saving images: " << e.what() << endl;
            }
        }
    }

    destroyAllWindows();
    cap.release();
}
