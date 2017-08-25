/*
Given a distortion and two cameras paramters (the real camera, and the virtual one), this script triangulates the surface points, to observe in the reflection in that surface the same distortion as given.
*/

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/features2d.hpp>
#include <fstream>  

using namespace std;
using namespace cv;

int w = 800;
int h = 0;
int wr = 0;
int hr = 0;
int wv = 0;
int hv = 0;
float sensor = 32;

void readcam(char* file, Vec3f& loc, Mat& rot, float& lens){
	fstream f;
	f.open(file);
	float x,y,z;
	f >> x;	f >> y; f >> z;
	loc = Vec3f(x,y,z);
	std::cout << loc << endl;
	for (int i = 0; i < 3; i+= 1)
	for (int j = 0; j < 3; j+= 1)
	{
		float r;
		f >> r;
		rot.at<float>(i,j)=r;
	}
	std::cout << rot << endl;
	f >> lens;
	std::cout << lens << endl;
	f.close();
}
void readdist(char* file, Mat& cameraMatrix, Mat& distCoeffs){
	float k1, k2,p1,p2,k3,k4,k5,k6,tx,ty,cx,cy,fx,fy;
	fstream f;
	f.open(file);
	f >> k1; f >> k2; f >> k3; f >> p1; f >> p2; f >> cx;f >> cy; f >> fx; f >> fy;
	f.close();
	distCoeffs.at<float>(0) = k1;
	distCoeffs.at<float>(1) = k2;
	distCoeffs.at<float>(4) = k3;
	distCoeffs.at<float>(2) = p1;
	distCoeffs.at<float>(3) = p2;
	cameraMatrix.at<float>(0,2) = cx;
	cameraMatrix.at<float>(1,2) = cy;
	cameraMatrix.at<float>(0,0) = fx;
	cameraMatrix.at<float>(1,1) = fy;
	cameraMatrix.at<float>(2,2) = 1;
}                      

Mat distort2(const cv::Mat& src, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
	float wv = src.cols;
	float hv = src.rows;
	cv::Mat pixel_locations_src = cv::Mat(Size(1,src.cols*src.rows), CV_32FC2);
	for (int i = 0; i < src.size().height; i++) {
		for (int j = 0; j < src.size().width; j++) {
			pixel_locations_src.at<cv::Point2f>(0,src.cols*i+j) = cv::Point2f(j,i);
		}
	}

	cv::Mat fractional_locations_dst = cv::Mat::zeros(1,src.cols*src.rows, CV_32FC2);
	cv::Mat pixel_locations_dst = cv::Mat::zeros(1, src.cols*src.rows, CV_32FC2);
	cv::undistortPoints(pixel_locations_src, fractional_locations_dst, cameraMatrix, distCoeffs);
	const float fx = cameraMatrix.at<float>(0,0);
	const float fy = cameraMatrix.at<float>(1,1);
	const float cx = cameraMatrix.at<float>(0,2);
	const float cy = cameraMatrix.at<float>(1,2);
	//return fractional_locations_dst;
	for (int i = 0; i < src.size().height; i++) {
		for (int j = 0; j < src.size().width; j++) {
			const float x = (fractional_locations_dst.at<cv::Point2f>(0,src.cols*i+j).x*fx + cx)/wv-0.5;
			const float y = (fractional_locations_dst.at<cv::Point2f>(0,src.cols*i+j).y*fy + cy)/hv-0.5;
			pixel_locations_dst.at<cv::Point2f>(0,src.cols*i+j) = cv::Point2f(x,y);
		}
	}
	return pixel_locations_dst;
}



void buildPointCloud(const Mat& img, const Mat& cameraMatrix, const Mat& distCoeffs, const Vec3f& Cr, const Mat& Rr, const float& fr, const Vec3f& Cv, const Mat& Rv, const float& fv, Size s)
{
	fstream f;
	f.open("surface.ply");
	f << "ply\n"
	  << "format ascii 1.0\n"
	  << "element vertex 11541\n"
	  << "property double x\n"
	  << "property double y\n"
	  << "property double z\n"
	  << "property double nx\n"
	  << "property double ny\n"
	  << "property double nz\n"
	  << "end_header\n";

	float k1, k2,p1,p2,k3,k4,k5,k6,tx,ty,cx,cy,fx,fy;
	k1 = distCoeffs.at<float>(0);
	k2 = distCoeffs.at<float>(1);
	k3 = distCoeffs.at<float>(4);
	p1 = distCoeffs.at<float>(2);
	p2 = distCoeffs.at<float>(3);
	cx = cameraMatrix.at<float>(0,2);
	cy = cameraMatrix.at<float>(1,2);
	fx = cameraMatrix.at<float>(0,0);
	fy = cameraMatrix.at<float>(1,1);
	cx=w/2;
	cy=h/2;


	std::cout << cy << std::endl;
	Vec3f X (-1,0,0);
	Vec3f Y (0,-1,0);
	Vec3f Z (0,0,1);
	Mat zcam = -1.0*Rr*(Mat)Z;
	Vec3f Zcam (zcam.at<float>(0,0),zcam.at<float>(0,1),zcam.at<float>(0,2));
	std::cout << Zcam << std::endl;

	Mat floc = distort2(img, cameraMatrix, distCoeffs);

	std::cout << floc.cols <<std::endl;


	int count =0;
	std::cout <<img.cols << " " << img.rows << std::endl;
	for (int i = 0; i < floc.cols; i+=119)
		for (int j =0; j<floc.rows; j+=1)
		{
			//undistorted pixel			
			float xu = ((float)i-cx)/w;-0.5;
			float yu = ((float)j-cy)*wv/hv/w;-0.5;
			//xu, yu \in [-0.5,0.5]
			xu = ((float)i)/1000.0-0.5;
			yu = ((float)j)/1000.0-0.5;
			xu = (floc.at<cv::Point2f>(j,i).x);
			yu = (floc.at<cv::Point2f>(j,i).y);

			float r2 = xu*xu+yu*yu;
			//distorted corresponding pixel
			float xd = (xu*(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)+2*p1*xu*yu+p2*(r2+2*xu*xu));
			float yd = (yu*(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)+2*p2*xu*yu+p1*(r2+2*yu*yu));
			xd = (-((float)(i%img.cols)/img.cols-0.5)*651.0+500)/4000.0-0.5;
			yd = (-((float)(i/img.cols)/img.rows-0.5)*1396.0+1000)/3000.0-0.5;
			if (true)
			//if (xd > -0.5 && yd > -0.5 && xd < 0.5 && yd < 0.5)
			//if (xu > -0.5 && yu > -0.5 && xu < 0.5 && yu < 0.5)
			{
				//compute vector Cvxu
				Mat CvXu = -1.0*Mat(fv*Rv*(Mat)Z+sensor*xu*Rv*(Mat)X+sensor*yu*s.height/s.width*Rv*(Mat)Y);
				Vec3f cvxu = 0.001*Vec3f(CvXu.at<float>(0,0),CvXu.at<float>(0,1),CvXu.at<float>(0,2));
				
				//compute vector Crxd
				Mat CrXd = -1.0*Mat(fr*Rr*(Mat)Z+sensor*xd*Rr*(Mat)X+sensor*yd*h/w*Rr*(Mat)Y);
				Vec3f crxd = 0.001*Vec3f(CrXd.at<float>(0,0),CrXd.at<float>(0,1),CrXd.at<float>(0,2));
				
				//compute normal
				Vec3f n = normalize(normalize(cvxu)-normalize(crxd));
				if (n.dot(Zcam) > 0)
					continue;

				//triangulation
				Mat A = (Mat_<float>(3,2) << cvxu[0], -crxd[0], cvxu[1], -crxd[1], cvxu[2], -crxd[2]); 
				Mat b = (Mat_<float>(3,1) << Cr[0]-Cv[0], Cr[1]-Cv[1], Cr[2]-Cv[2]);
				Mat X;
				solve(A,b,X,DECOMP_SVD);
				if (/*X.at<float>(0,0) < 0 ||*/ X.at<float>(1,0) < 0)
					continue;
				count++;
				Vec3f P = 2.0*(/*Cv+X.at<float>(0,0)*cvxu +*/ Cr+X.at<float>(1,0)*crxd)/2.0;
				f << P[0] << " " << P[1] << " " << P[2] << " " << n[0] << " " << n[1] << " " << n[2] << "\n";
			}
		}
	f.close();
	std::cout << count << std::endl;
	return;
}
int main(int argc, char** argv )
{
	Vec3f Cr = Vec3f(0,0,0);
	Mat Rr = Mat::zeros(3,3,CV_32F);
	float fr = 0.0;
	Vec3f Cv = Vec3f(0,0,0);
	Mat Rv = Mat::zeros(3,3,CV_32F);
	float fv = 0.0;
	Mat distCoeffs = Mat::zeros(1,5,CV_32F);
	Mat cameraMatrix = Mat::zeros(3,3,CV_32F);
	readcam(argv[1], Cr, Rr, fr);
	readcam(argv[2], Cv, Rv, fv);
	readdist(argv[3], cameraMatrix,distCoeffs);
	std::cout << distCoeffs << std::endl << cameraMatrix << std::endl;

	//fv=fr;

	Mat imgr = imread( argv[4], 1 );
	//must be of the size of the distorted photograph
	Mat imgv = imread( argv[5], 1 );
	Mat imgvu = imread( argv[6], 1 );
	//imgv = imread( argv[4], 1 );
	wr = imgr.cols;
	hr = imgr.rows;
	wv = imgv.cols;
	hv = imgv.rows;
	h = w*hr/wr;
	std::cout << h << std::endl;

	buildPointCloud(imgv, cameraMatrix,distCoeffs,Cr,Rr,fr,Cv,Rv,fv, imgvu.size());
	return 0;
}	
