// g++ -std=c++17 stereo_calibration.cpp -o calibration `pkg-config --cflags --libs opencv4`

#include <iostream>
#include <string>
#include <vector>
#include <filesystem> 
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

string left_frame_path = "images/left";
string right_frame_path = "images/right";

void create_folders(){
    try {
        if (!fs::exists(left_frame_path)) {
            fs::create_directories(left_frame_path);
            cout << "Folder created : " << left_frame_path << endl;
        }
        if (!fs::exists(right_frame_path)) {
            fs::create_directories(right_frame_path);
            cout << "Folder created : " << right_frame_path << endl;
        }
    } catch (const fs::filesystem_error& e) {
        cerr << "Error with folders creation : " << e.what() << endl;
    }
}

void capture_frames(){
    VideoCapture cap(2);
    
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 640); 

    if (!cap.isOpened()) {
        cerr << "Error : Impossible to open the camera" << endl;
    }

    namedWindow("Left", WINDOW_NORMAL);
    namedWindow("Right", WINDOW_NORMAL);

    resizeWindow("Left", 640, 480);
    resizeWindow("Right", 640, 480);

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
                string left_filename = left_frame_path + "/" + to_string(capture_count) + "_frame.png";
                string right_filename = right_frame_path + "/" + to_string(capture_count) + "_frame.png";

                if (!imwrite(left_filename, left_img)) {
                    throw runtime_error("imwrite failed for left image. Check directory permissions.");
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
            catch (const cv::Exception& e) {
                cerr << "OpenCV Error while saving images: " << e.what() << endl;
            }
            catch (const std::exception& e) {
                cerr << "Error while saving images: " << e.what() << endl;
            }
            catch (...) {
                cerr << "Unknown error while saving images." << endl;
            }
        }
    }

    destroyAllWindows();
    cap.release();
}


int main() {
    create_folders();
    capture_frames();

    return 0;
}