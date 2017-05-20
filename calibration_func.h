#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

#pragma once
class calibration_func
{
public:
	calibration_func();
	~calibration_func();
	void correction(Mat orginal_image_left, Mat orginal_image_righ);
	double fisheyeCalibrate();
	bool FishEyeFindPoint(Mat image, int lr);
	void FishEyeFindPointOnce(Mat image_left, Mat image_right);
	int calculatePic(Mat& displayImage);
	void calculateDepth(int x, int y);
	Mat get_img();
	int getFrame(){ return nFrame; }
	void get_DepthMat();
	void saveCalibrateData();
	void getCalibrateData(double k1[9], double d1[4], double k2[9], double d2[4], double r[9], double t[3]);

private:
	int board_width;
	int board_hight;
	int board_n;
	int N;
	int nFrame;
	Mat imgL;
	Mat imgR;
	Mat the_ObjectPoints;
	Mat the_LeftImage;
	Mat the_RightImage;
	Matx33d Kl, Kr;
	Vec4d Dl, Dr;
	vector<Mat> R1, T1, R2, T2;
	Vec3d T;
	Mat R, E, F, Q;
	double c_Q[4][4];
	float squareSize;
	Size the_left_size;
	Size the_right_size;
	vector<vector<cv::Point2d>> fisheyeImage_Points[2];
	Ptr<StereoBM> CSBMS;
	Ptr<StereoSGBM> SGBMS;
	Mat disp;
	Mat Image3DPoint;
	int file_x;
};

