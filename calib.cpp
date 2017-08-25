/*
This script compute a distortion thanks to 2D-3D correspondences and write the undistorted image. It also distorts the same way another image, to check
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
int h = 1720;

void distort2(const cv::Mat& src, cv::Mat& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
	cv::Mat pixel_locations_src = cv::Mat(Size(1,src.cols*src.rows), CV_32FC2);
	for (int i = 0; i < src.size().height; i++) {
		for (int j = 0; j < src.size().width; j++) {
			pixel_locations_src.at<cv::Point2f>(0,src.cols*i+j) = cv::Point2f(j,i);
		}
	}
	std::cout << ":)" << std::endl;
	cv::Mat fractional_locations_dst = cv::Mat(src.size(), CV_32FC2);
	cv::Mat pixel_locations_dst = cv::Mat(src.size(), CV_32FC2);
	cv::undistortPoints(pixel_locations_src, fractional_locations_dst, cameraMatrix, distCoeffs);
	const float fx = cameraMatrix.at<double>(0,0);
	const float fy = cameraMatrix.at<double>(1,1);
	const float cx = cameraMatrix.at<double>(0,2);
	const float cy = cameraMatrix.at<double>(1,2);
	for (int i = 0; i < src.size().height; i++) {
		for (int j = 0; j < src.size().width; j++) {
			const float x = fractional_locations_dst.at<cv::Point2f>(0,src.cols*i+j).x*fx + cx;
			const float y = fractional_locations_dst.at<cv::Point2f>(0,src.cols*i+j).y*fy + cy;
			pixel_locations_dst.at<cv::Point2f>(i,j) = cv::Point2f(x,y);
		}
	}
	cv::remap(src, dst, pixel_locations_dst, cv::Mat(), CV_INTER_LINEAR);
}


void distort(const Mat & undist, Mat& dist, const Mat& cameraMatrix, const Mat& distCoeffs)
{
	float k1, k2,p1,p2,k3,k4,k5,k6,tx,ty,cx,cy,fx,fy;
	k1 = distCoeffs.at<double>(0);
	k2 = distCoeffs.at<double>(1);
	k3 = distCoeffs.at<double>(4);
	p1 = distCoeffs.at<double>(2);
	p2 = distCoeffs.at<double>(3);
	cx = cameraMatrix.at<double>(0,2);
	cy = cameraMatrix.at<double>(1,2);
	fx = cameraMatrix.at<double>(0,0);
	fy = cameraMatrix.at<double>(1,1);
	
	fstream f;
	f.open("camera.txt");
	f << k1 << std::endl << k2 << std::endl << k3 << std::endl << p1 << std::endl << p2 << std::endl << cx << std::endl << cy << std::endl << fx << std::endl << fy; 
	f.close();

	std::cout << cy << std::endl;
	undist.copyTo(dist);
	resize(dist,dist,Size(w,h));
	Mat mean = Mat::zeros(dist.size(),CV_32S);
	for (int i =0; i < dist.cols; i++)
		for (int j =0; j < dist.rows; j++)
			dist.at<Vec3b>(Point(i,j))=0;

	std::cout << dist.size() << std::endl;
	std::cout << undist.size() << std::endl;
	for (int i =0; i < undist.cols; i++)
		for (int j =0; j < undist.rows; j++)
		{
			float xu = ((float)i-cx)/undist.cols;-0.5;
			float yu = ((float)j-cy)/undist.rows;-0.5;
			Vec3b col = undist.at<Vec3b>(Point(i,j));
			float r2 = xu*xu+yu*yu;
			int xd = (xu*(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)+2*p1*xu*yu+p2*(r2+2*xu*xu))*((float)undist.cols)+cx;+w/2+0.5;
			int yd = (yu*(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)+2*p2*xu*yu+p1*(r2+2*yu*yu))*((float)(h))+undist.rows;+h/2+0.5;
			if (xd > -1 && yd > -1 && xd < dist.cols && yd < dist.rows)
			{
				Vec3b prevcol = dist.at<Vec3b>(Point(xd,yd));
				mean.at<int>(Point(xd,yd))+=1;
				for (int k = 0; k < 3; ++k)
					dist.at<Vec3b>(Point(xd,yd))[k]=(col[k]+((float)(mean.at<int>(Point(xd,yd))-1.0))*prevcol[k])/(float)mean.at<int>(Point(xd,yd));
			}
		}
	for (int i =0; i < dist.cols; i++)
	for (int j =0; j < dist.rows; j++)
	{
		Vec3b col = dist.at<Vec3b>(Point(i,j));
		if (col==Vec3b(0,0,0))
		{
			int sum =0;
			for (int x=-1; x<2;x++)
			for (int y=-1; y<2;y++)
			{
				if (i+x < 0 || i+x>=dist.cols || j+y < 0 || j+y >= dist.rows)
					continue;
				Vec3b col1 = dist.at<Vec3b>(Point(i+x,j+y));
				if (col1!=Vec3b(0,0,0))
				{
					sum++;
					for (int k = 0; k < 3; ++k)
						dist.at<Vec3b>(Point(i,j))[k]=(dist.at<Vec3b>(Point(i,j))[k]+((float)(sum-1.0))*col1[k])/(float)sum;
				}
			}
		}
	}
	imshow("test",2000.0*mean);
	imwrite("distfield.png", 2000.0*mean);
	waitKey(0);
	return;
}
int main(int argc, char** argv )
{
	vector<Point3f> p3d = vector<Point3f>();
	vector<Point2f> p2d = vector<Point2f>();
	fstream f;
	f.open(argv[1]);
	int n=0;
	f >> n;
	std::cout << n << std::endl;
	for (int i = 0; i < n; i+= 1)
	{
		float x,y,z;
		f >> x;	f >> y; f >> z;
		Point3f p(x,y,z);
		std::cout << p << endl;
		p3d.push_back(p);
	}
	for (int i = 0; i < n; i+= 1)
	{
		float u,v;
		f >> u;	f >> v;
		Point2f p((w/2)*u+w/2,((w/2)*v+w/2)*h/1720.0);
		//Point2f p(u,v);
		std::cout << p << endl;
		p2d.push_back(p);
	}
	f.close();

	vector<vector<Point3f> > p3ds = {p3d};
	vector<vector<Point2f> > p2ds = {p2d};
	Mat cameraMatrix = Mat::eye(3,3,CV_32F);
	cameraMatrix.at<float>(0,0)=w;
	cameraMatrix.at<float>(1,1)=h;
	cameraMatrix.at<float>(0,2)=/*87;/ */w/2;
	cameraMatrix.at<float>(1,2)=/*434;/ */h/2;
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	Mat rvecs, tvecs;
	calibrateCamera	(p3ds, p2ds, Size(w,h), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS 
			//|CALIB_TILTED_MODEL
			//|CV_CALIB_FIX_ASPECT_RATIO
		       	//|CV_CALIB_FIX_PRINCIPAL_POINT	
			|CV_CALIB_ZERO_TANGENT_DIST 
			//|CV_CALIB_RATIONAL_MODEL 
			//| CALIB_THIN_PRISM_MODEL
			,  TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 3000000, 0.00001*DBL_EPSILON));

	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;

	Mat image;
	Mat image2;
	image = imread( argv[2], 1 );
	Size newSize(w,h);
	resize(image,image,newSize);
	imshow("test", image);
	waitKey(0);
	undistort(image, image2, cameraMatrix, distCoeffs);
	imshow("test", image2);
	waitKey(0);
	imwrite("result.jpg",image2);
	Mat undist;
	Mat dist;
	undist = imread( argv[3], 1 );
	resize(undist,undist,Size(image.cols,image.rows));
	imshow("test", undist);
	waitKey(0);
	distort2(undist, dist, cameraMatrix, distCoeffs);
	imshow("test", dist);
	waitKey(0);
	imwrite("distored.jpg",dist);
	return 0;
}
