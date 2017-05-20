#include "calibration_func.h"


calibration_func::calibration_func()
{
	board_width = 9;
	board_hight = 6;
	board_n = board_width * board_hight;
	nFrame = 0;
	N = 0;
	R = Mat(3, 3, CV_64F);
	E = Mat(3, 3, CV_64F);
	F = Mat(3, 3, CV_64F);
	Kl = cv::Matx33d();
	Dl = cv::Vec4d();
	Kr = cv::Matx33d();
	Dr = cv::Vec4d();
	CSBMS = StereoBM::create(16, 9);
	CSBMS->setPreFilterSize(13);
	CSBMS->setPreFilterCap(31);
	CSBMS->setBlockSize(19);
	CSBMS->setMinDisparity(0);
	CSBMS->setNumDisparities(32);
	CSBMS->setTextureThreshold(400);
	CSBMS->setUniquenessRatio(5);
	CSBMS->setSpeckleWindowSize(100);
	CSBMS->setSpeckleRange(32);
	SGBMS = StereoSGBM::create(0, 16, 3);
	SGBMS->setPreFilterCap(63);
	int sgbmWinSize = 11;
	int cn = 1;
	SGBMS->setBlockSize(sgbmWinSize);
	SGBMS->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	SGBMS->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	SGBMS->setMinDisparity(0);
	SGBMS->setNumDisparities(32);
	SGBMS->setUniquenessRatio(10);
	SGBMS->setSpeckleWindowSize(100);
	SGBMS->setSpeckleRange(32);
	SGBMS->setDisp12MaxDiff(1);
	SGBMS->setMode(StereoSGBM::MODE_SGBM);

	squareSize = 14.0f;
	file_x = 0;
}



calibration_func::~calibration_func()
{

}


double calibration_func::fisheyeCalibrate()
{
	if (nFrame == 0)
		return 0.0;
	vector<vector<Point3d>> objectPoints;
	for (int t = 0; t<nFrame; t++)
	{
		vector<Point3d> tempPointSet;
		for (int i = 0; i < board_hight; i++)
		{
			for (int j = 0; j < board_width; j++)
			{
				Point3d tempPoint;
				tempPoint.x = i*squareSize;
				tempPoint.y = j*squareSize;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		objectPoints.push_back(tempPointSet);
	}
	std::vector<cv::Vec3d> rotation_vectors_L;
	std::vector<cv::Vec3d> translation_vectors_L;
	std::vector<cv::Vec3d> rotation_vectors_R;
	std::vector<cv::Vec3d> translation_vectors_R;
	int fisheyeFlag = 0;
	fisheyeFlag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	fisheyeFlag |= cv::fisheye::CALIB_CHECK_COND;
	fisheyeFlag |= cv::fisheye::CALIB_FIX_SKEW;
	cv::fisheye::calibrate(objectPoints, fisheyeImage_Points[0], the_left_size, Kl, Dl, rotation_vectors_L, translation_vectors_L,
		fisheyeFlag, TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-6));
	cv::fisheye::calibrate(objectPoints, fisheyeImage_Points[1], the_right_size, Kr, Dr, rotation_vectors_R, translation_vectors_R,
		fisheyeFlag, TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-6));

	int	flag = fisheye::CALIB_FIX_INTRINSIC + fisheye::CALIB_RECOMPUTE_EXTRINSIC + fisheye::CALIB_USE_INTRINSIC_GUESS;
	double err = fisheye::stereoCalibrate(objectPoints, fisheyeImage_Points[0], fisheyeImage_Points[1], Kl, Dl, Kr, Dr,
		the_left_size, R, T, flag, TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
	return err;
}

void calibration_func::getCalibrateData(double k1[9], double d1[4], double k2[9], double d2[4], double r[9], double t[3])
{
	Kl = cv::Matx33d(k1);
	Dl = cv::Vec4d(d1);
	Kr = cv::Matx33d(k2);
	Dr = cv::Vec4d(d2);
	R = Mat(3, 3,CV_64F, r);
	T = Mat(1, 3, CV_64F, t);
}

void calibration_func::saveCalibrateData()
{
	//get Kl data
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double k1 = Kl.val[i * 3 + j];
			//do something
		}
	}

	//get Dl data
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			double d1 = Dl.val[j];
			//do something
		}
	}

	//get Kr data
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double k2 = Kr.val[i * 3 + j];
			//do something
		}			 
	}

	//get Dr data
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			double d2 = Dr.val[j];
			//do something
		}
	}

	//get T data
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 1; j++)
		{
			double t = T.val[i];
			//do something
		}
	}

	//get R data
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double r = R.at<double>(i, j);
			//do something
		}
	}
}

void calibration_func::correction(Mat orginal_image_left, Mat orginal_image_right)
{
	Mat left_mapx = Mat::eye(the_left_size, CV_32F);
	Mat left_mapy = Mat::eye(the_right_size, CV_32F);
	Mat right_mapx = Mat::eye(the_left_size, CV_32F);
	Mat right_mapy = Mat::eye(the_right_size, CV_32F);
	imgL = Mat::eye(the_left_size, CV_8U);
	imgR = Mat::eye(the_right_size, CV_8U);
	Mat image_L, image_R;
	cvtColor(orginal_image_left, image_L, CV_BGR2GRAY);
	cvtColor(orginal_image_right, image_R, CV_BGR2GRAY);
	double c_Rl[3][3], c_Rr[3][3], c_Pl[3][4], c_Pr[3][4];
	Mat Rl = Mat(3, 3, CV_64F, &c_Rl);
	Mat Rr = Mat(3, 3, CV_64F, &c_Rr);
	Mat Pl = Mat(3, 4, CV_64F, &c_Pl);
	Mat Pr = Mat(3, 4, CV_64F, &c_Pr);
	Q = Mat(4, 4, CV_64F, &c_Q);
	fisheye::stereoRectify(Kl, Dl, Kr, Dr, the_left_size,
		R, T, Rl, Rr, Pl, Pr, Q, CV_CALIB_ZERO_DISPARITY);

	fisheye::initUndistortRectifyMap(Kl, Dl, Rl, Pl, the_left_size, CV_32FC1, left_mapx, left_mapy);
	fisheye::initUndistortRectifyMap(Kr, Dr, Rr, Pr, the_right_size, CV_32FC1, right_mapx, right_mapy);
	remap(image_L, imgL, left_mapx, left_mapy, cv::INTER_LINEAR);
	remap(image_R, imgR, right_mapx, right_mapy, cv::INTER_LINEAR);

	Q.at<double>(3, 2) = -Q.at<double>(3, 2);
	const char* fileNameL = "D:\\test\\新建文件夹\\y\\corrPicL.jpg";
	const char* fileNameR = "D:\\test\\新建文件夹\\y\\corrPicR.jpg";
	imwrite(fileNameL, imgL);
	imwrite(fileNameR, imgR);
	cout << "the Q matrix is：" << endl;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			cout << Q.at<double>(i, j) << " ";
		cout << endl;
	}
	cout << endl;
}

void calibration_func::get_DepthMat()
{
	disp = Mat(the_left_size, CV_16SC1);
	Image3DPoint = Mat(the_left_size, CV_32FC3);
	//CSBMS->compute(imgL, imgR, disp);	
	SGBMS->compute(imgL, imgR, disp);
	//disp.convertTo(disp, CV_32FC1, 1.0 / 16);

	reprojectImageTo3D(disp, Image3DPoint, Q, true);
	Image3DPoint *= 16;
	imwrite("D:\\test\\其它\\disp.jpg", disp);
}


int calibration_func::calculatePic(Mat& displayImage)
{
	Mat vdisp;	
	disp.convertTo(vdisp, CV_8U, 255 / (32 * 16.0));
	//normalize(disp, vdisp, 0, 256, CV_MINMAX);			
	vdisp.copyTo(displayImage);
	return 1;
}

void calibration_func::calculateDepth(int x, int y)
{
	Point3f s1 = Image3DPoint.at<Point3f>(x, y);
	cout << "The point 3D point is：" << s1.x<< " , " << s1.y<< " , " << s1.z<< endl;
}

Mat calibration_func::get_img()
{
	return imgL;
}


bool calibration_func::FishEyeFindPoint(Mat image, int lr)
{
	vector<cv::Point2f> corners;
	int cornerFound = cv::findChessboardCorners(image, Size(board_width, board_hight), corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE +
		CV_CALIB_CB_FAST_CHECK);
	if (cornerFound)
	{
		Mat gray_image;
		cvtColor(image, gray_image, CV_RGB2GRAY);
		cv::cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 1e-5));
		vector<Point2d> corners_image;
		int x = corners.size();
		for (int i = 0; i < x; i++)
		{
			Point2d point;
			point.x = corners[i].x;
			point.y = corners[i].y;
			corners_image.push_back(point);
		}
		fisheyeImage_Points[lr].push_back(corners_image);
		drawChessboardCorners(image, Size(board_width, board_hight), corners, cornerFound);
		char ss[40];
		sprintf(ss, "D:\\test\\新建文件夹\\y\\test%d.jpg", file_x);
		imwrite(ss, image);
		file_x++;
		return true;
	}
	else
	{
		return false;
	}
}

void calibration_func::FishEyeFindPointOnce(Mat image_left, Mat image_right)
{
	bool isLeftFishEyeFind = FishEyeFindPoint(image_left, 0);
	if (isLeftFishEyeFind && FishEyeFindPoint(image_right, 1))
	{
		the_left_size = Size(image_left.cols, image_left.rows);
		the_right_size = Size(image_right.cols, image_right.rows);
		nFrame++;
		return;
	}
	if (isLeftFishEyeFind)
		fisheyeImage_Points[0].erase(fisheyeImage_Points[0].end() - 1);
}
