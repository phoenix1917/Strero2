#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _MAIN
#define _MAIN

using namespace cv;
using namespace std;

string num2str(int num);
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners);
double computeReprojectionErrors(vector<vector<Point3f> >& objectPoints, vector<vector<Point2f> >& imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, Mat& cameraMatrix, Mat& distCoeffs, vector<float>& perViewErrors);

void printCalibResults(Mat &cameraMatrix, Mat &distCoeffs, double reprojectionError, Mat &stdDevIntrinsics, Mat &stdDevExtrinsics, vector<double> &perViewErrors);
void printCalibResults(Mat &cameraMatrix, Mat &distCoeffs, double reprojectionError, Mat &stdDevIntrinsics, Mat &stdDevExtrinsics, vector<double> &perViewErrors, ofstream &fout);
void printCalibResults(Mat &cameraMatrix, Mat &distCoeffs, double reprojectionError, Mat &stdDevIntrinsics);
void printCalibResults(Mat &cameraMatrix, Mat &distCoeffs, double reprojectionError);

void onMouseL(int event, int x, int y, int flags, void *param);
void onMouseR(int event, int x, int y, int flags, void *param);

void reconstruct(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure);
void reconstruct(Mat& K1, Mat& K2, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure);
bool findTransform(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& mask);
bool findTransform(Mat& K1, Mat& K2, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& mask);
void maskoutPoints(vector<Point2f>& p1, Mat& mask);
bool fixPrinciplePoint(Mat& K, Point2f point);
void maskoutPoints(vector<Point2f>& p1, Mat& mask);
void toPoints3D(Mat& points4D, Mat& points3D);

#endif