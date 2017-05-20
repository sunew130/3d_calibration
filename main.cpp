#include "calibration_func.h"

void MouseClick(int event, int x, int y, int flags, void *param)
{
	calibration_func *cf = (calibration_func*)param;
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		printf("This is (%d,%d)\n", x, y);
		cf->calculateDepth(y, x);
		break;
	}
}

void main()
{
	calibration_func *cf = new calibration_func();
	Mat image_left1 = imread("D:\\test\\新建文件夹\\cal6\\1l_eye.jpg");
	Mat image_right1 = imread("D:\\test\\新建文件夹\\cal6\\1r_eye.jpg");

	Mat image_left2 = imread("D:\\test\\新建文件夹\\cal6\\2l_eye.jpg");
	Mat image_right2 = imread("D:\\test\\新建文件夹\\cal6\\2r_eye.jpg");

	Mat image_left3 = imread("D:\\test\\新建文件夹\\cal6\\3l_eye.jpg");
	Mat image_right3 = imread("D:\\test\\新建文件夹\\cal6\\3r_eye.jpg");

	Mat image_left4 = imread("D:\\test\\新建文件夹\\cal6\\4l_eye.jpg");
	Mat image_right4 = imread("D:\\test\\新建文件夹\\cal6\\4r_eye.jpg");

	Mat image_left5 = imread("D:\\test\\新建文件夹\\cal6\\5l_eye.jpg");
	Mat image_right5 = imread("D:\\test\\新建文件夹\\cal6\\5r_eye.jpg");

	Mat image_left6 = imread("D:\\test\\新建文件夹\\cal6\\6l_eye.jpg");
	Mat image_right6 = imread("D:\\test\\新建文件夹\\cal6\\6r_eye.jpg");

	Mat image_left7 = imread("D:\\test\\新建文件夹\\cal6\\7l_eye.jpg");
	Mat image_right7 = imread("D:\\test\\新建文件夹\\cal6\\7r_eye.jpg");

	Mat image_left8 = imread("D:\\test\\新建文件夹\\cal6\\8l_eye.jpg");
	Mat image_right8 = imread("D:\\test\\新建文件夹\\cal6\\8r_eye.jpg");

	Mat image_left9 = imread("D:\\test\\新建文件夹\\cal6\\9l_eye.jpg");
	Mat image_right9 = imread("D:\\test\\新建文件夹\\cal6\\9r_eye.jpg");

	Mat image_left10 = imread("D:\\test\\新建文件夹\\cal6\\10l_eye.jpg");
	Mat image_right10 = imread("D:\\test\\新建文件夹\\cal6\\10r_eye.jpg");

	Mat image_left11 = imread("D:\\test\\新建文件夹\\cal6\\11l_eye.jpg");
	Mat image_right11 = imread("D:\\test\\新建文件夹\\cal6\\11r_eye.jpg");

	Mat image_left12 = imread("D:\\test\\新建文件夹\\cal6\\12l_eye.jpg");
	Mat image_right12 = imread("D:\\test\\新建文件夹\\cal6\\12r_eye.jpg");


	Mat image_left13 = imread("D:\\test\\新建文件夹\\cal6\\13l_eye.jpg");
	Mat image_right13 = imread("D:\\test\\新建文件夹\\cal6\\13r_eye.jpg");


	Mat image_left14 = imread("D:\\test\\新建文件夹\\cal6\\14l_eye.jpg");
	Mat image_right14 = imread("D:\\test\\新建文件夹\\cal6\\14r_eye.jpg");


	Mat image_left15 = imread("D:\\test\\新建文件夹\\cal6\\15l_eye.jpg");
	Mat image_right15 = imread("D:\\test\\新建文件夹\\cal6\\15r_eye.jpg");


	Mat image_left16 = imread("D:\\test\\新建文件夹\\cal6\\16l_eye.jpg");
	Mat image_right16 = imread("D:\\test\\新建文件夹\\cal6\\16r_eye.jpg");


	Mat image_left17 = imread("D:\\test\\新建文件夹\\cal6\\17l_eye.jpg");
	Mat image_right17 = imread("D:\\test\\新建文件夹\\cal6\\17r_eye.jpg");



	Mat image_left18 = imread("D:\\test\\新建文件夹\\cal6\\18l_eye.jpg");
	Mat image_right18 = imread("D:\\test\\新建文件夹\\cal6\\18r_eye.jpg");


	Mat image_left19 = imread("D:\\test\\新建文件夹\\cal6\\19l_eye.jpg");
	Mat image_right19 = imread("D:\\test\\新建文件夹\\cal6\\19r_eye.jpg");


	Mat image_left20 = imread("D:\\test\\新建文件夹\\cal6\\20l_eye.jpg");
	Mat image_right20 = imread("D:\\test\\新建文件夹\\cal6\\20r_eye.jpg");


/*	Mat image_left21 = imread("D:\\test\\其它\\cal3\\T3L.jpg");
	Mat image_right21 = imread("D:\\test\\其它\\cal3\\T3R.jpg");*/

	Mat image_left21 = imread("D:\\test\\其它\\cal2\\test7L.jpg");
	Mat image_right21 = imread("D:\\test\\其它\\cal2\\test7R.jpg");
	cvNamedWindow("test1");
	cvNamedWindow("test2");

	cf->FishEyeFindPointOnce(image_left1, image_right1);
	cf->FishEyeFindPointOnce(image_left2, image_right2);
	cf->FishEyeFindPointOnce(image_left3, image_right3);
	cf->FishEyeFindPointOnce(image_left4, image_right4);
	cf->FishEyeFindPointOnce(image_left5, image_right5);
	cf->FishEyeFindPointOnce(image_left6, image_right6);
	cf->FishEyeFindPointOnce(image_left7, image_right7);
	cf->FishEyeFindPointOnce(image_left8, image_right8);
	cf->FishEyeFindPointOnce(image_left9, image_right9);
	cf->FishEyeFindPointOnce(image_left10, image_right10);
	cf->FishEyeFindPointOnce(image_left11, image_right11);
	cf->FishEyeFindPointOnce(image_left12, image_right12);
	cf->FishEyeFindPointOnce(image_left13, image_right13);
	cf->FishEyeFindPointOnce(image_left14, image_right14);
	cf->FishEyeFindPointOnce(image_left15, image_right15);
	cf->FishEyeFindPointOnce(image_left16, image_right16);
	cf->FishEyeFindPointOnce(image_left17, image_right17);
	cf->FishEyeFindPointOnce(image_left18, image_right18);
	cf->FishEyeFindPointOnce(image_left19, image_right19);
	cf->FishEyeFindPointOnce(image_left20, image_right20);


	double err = cf->fisheyeCalibrate();
	printf("The error is %g\n", err);
	cf->correction(image_left21, image_right21);

	cf->get_DepthMat();
	Mat s;
	cf->calculatePic(s);
	Mat x = cf->get_img();
	const char* fileNameR = "D:\\test\\新建文件夹\\y\\result.jpg";
	imwrite(fileNameR, s);
	imshow("test1", s);
	imshow("test2", x);

	cvSetMouseCallback("test1", MouseClick, (void*)cf);
	cvSetMouseCallback("test2", MouseClick, (void*)cf);

	waitKey(0);
	destroyWindow("test1");
	destroyWindow("test2");

}