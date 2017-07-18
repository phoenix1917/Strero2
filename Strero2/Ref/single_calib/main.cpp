#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "main.h"

using namespace cv;
using namespace std;

int main() {
    // 标定所用图像文件的路径
    ifstream fin("calibdata2.txt");
    // 保存标定结果的文件
    ofstream fout("caliberation_result.txt");
    // 定义所使用的标定板
    #define USING_BOARD_1
    // 显示角点提取结果
    bool showCornerExt = false;

#ifdef USING_BOARD_1
    // 标定板1：9x7，35mm，白色边缘
    // 标定版上棋盘格内角点的个数
    Size boardSize(8, 6);
    // 标定板上每个方格的大小
    float squareSize = 35.0;
#endif 

#ifdef USING_BOARD_2
    // 标定板2：12x9，30mm，黑色边缘
    // 标定版上棋盘格内角点的个数
    Size boardSize(11, 8);
    // 标定板上每个方格的大小
    float squareSize = 30.0;
#endif 

#ifdef USING_BOARD_3
    // 标定板3：10x9，90mm，白色边缘
    // 标定版上棋盘格内角点的个数
    Size boardSize(9, 8);
    // 标定板上每个方格的大小
    float squareSize = 90.0;
#endif 

    // 每行读入的图像路径
    string fileName;
    // 读入后的图像序列
    vector<Mat> imageSet;
    // 每幅图像的大小
    Size imageSize;
    // 输入图像数
    int imgCount;
    // 找到全部角点的图像数
    int acceptedCount = 0;
    // 找到全部角点的图像编号
    vector<int> acceptedImages;
    // 所有标定图像的角点
    vector<vector<Point2f> > allCorners;
    // 内参数矩阵和畸变系数
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    // 每幅图像的旋转矩阵和平移向量
    vector<Mat> rVecs, tVecs;
    // 每幅图像使用的标定板的角点的三维坐标
    vector<vector<Point3f> > objectPoints(1);
    // 内参数误差
    Mat stdDevIntrinsics;
    // 外参数误差
    Mat stdDevExtrinsics;
    // 每幅图像的重投影误差
    vector<double> perViewErrors;

    // 读取图像
    while(getline(fin, fileName)) {
        Mat img = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
        imageSet.push_back(img);
    }
    imageSize = imageSet[0].size();
    imgCount = imageSet.size();

    // 逐图像提取角点
    for(int i = 0; i < imgCount; i++) {
        bool foundAllCorners = false;
        vector<Point2f> cornerBuf;
        Mat view = imageSet[i];
        Mat viewSubpix = imageSet[i];

        // 寻找棋盘格的内角点位置
        // flags:
        // CV_CALIB_CB_ADAPTIVE_THRESH
        // CV_CALIB_CB_NORMALIZE_IMAGE
        // CV_CALIB_CB_FILTER_QUADS
        // CALIB_CB_FAST_CHECK
        foundAllCorners = findChessboardCorners(view, boardSize, cornerBuf, 
                                                CV_CALIB_CB_NORMALIZE_IMAGE);
        if(showCornerExt) {
            // 绘制内角点。若检出全部角点，连线展示；若未检出，绘制检出的点
            drawChessboardCorners(view, boardSize, Mat(cornerBuf), foundAllCorners);
            imshow("corners", view);
            waitKey(500);
        }

        if(foundAllCorners) {
            acceptedCount += 1;
            acceptedImages.push_back(i);
            // 寻找亚像素级角点
            find4QuadCornerSubpix(viewSubpix, cornerBuf, Size(5, 5));
            allCorners.push_back(cornerBuf);
            if(showCornerExt) {
                // 绘制调整后的角点
                drawChessboardCorners(viewSubpix, boardSize, Mat(cornerBuf), true);
                imshow("corners", viewSubpix);
                waitKey(500);
            } else {
                cout << ">>>" << i + 1;
            }
        }
    }
    if(!showCornerExt) {
        cout << endl << endl;
    }
    
    // 输出提取结果统计
    destroyWindow("corners");
    if(acceptedCount <= 3) {
        cout << "角点检测失败" << endl;
        system("pause");
        return 0;
    } else {
        cout << "使用 " << acceptedCount << " 幅图像进行标定：" << endl;
        fout << "使用 " << acceptedCount << " 幅图像进行标定：" << endl;
        for(auto iter = acceptedImages.cbegin(); iter != acceptedImages.cend(); ++iter) {
            cout << (*iter) + 1;
            fout << (*iter) + 1;
            if(*iter != (int)acceptedImages.size() - 1) {
                cout << ", ";
                fout << ", ";
            } else {
                cout << endl << endl;
                fout << endl << endl;
            }
        }
    }

    // 对所有已接受图像，初始化标定板上角点的三维坐标
    calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);
    objectPoints.resize(allCorners.size(), objectPoints[0]);
    
    // 初始化内参数矩阵
    cameraMatrix = initCameraMatrix2D(objectPoints, allCorners, imageSize);
    cout << "内参数预估：" << endl;
    cout << "cameraMatrix = " << endl << cameraMatrix << endl << endl;
    fout << "内参数预估：" << endl;
    fout << "cameraMatrix = " << endl << cameraMatrix << endl << endl;

    // 标定
    // flags:
    // CV_CALIB_USE_INTRINSIC_GUESS         使用预估的内参数矩阵
    // CV_CALIB_FIX_PRINCIPAL_POINT         固定主点坐标
    // CV_CALIB_FIX_ASPECT_RATIO            固定焦距比
    // CV_CALIB_ZERO_TANGENT_DIST           不计算切向畸变(p1, p2)
    // CV_CALIB_FIX_K1,...,CV_CALIB_FIX_K6  固定径向畸变K1-K6参数
    // CV_CALIB_RATIONAL_MODEL              计算8参数畸变模型(K4-K6)，不写则计算5参数
    // CALIB_THIN_PRISM_MODEL               计算S1-S4参数，不写则不计算
    // CALIB_FIX_S1_S2_S3_S4                固定S1-S4参数值
    // CALIB_TILTED_MODEL                   计算(tauX, tauY)参数，不写则不计算
    // CALIB_FIX_TAUX_TAUY                  固定(tauX, tauY)参数值
    double reprojectionError = calibrateCamera(objectPoints, allCorners, imageSize,
                                               cameraMatrix, distCoeffs, rVecs, tVecs,
                                               stdDevIntrinsics, stdDevExtrinsics, perViewErrors,
                                               CV_CALIB_USE_INTRINSIC_GUESS | 
                                               CV_CALIB_FIX_PRINCIPAL_POINT);
    
    // 输出标定结果
    cout << "标定结果：" << endl;
    cout << "cameraMatrix = " << endl << cameraMatrix << endl << endl;
    cout << "distCoeffs = " << endl << distCoeffs << endl << endl;
    cout << "reprojectionError = " << endl << reprojectionError << endl << endl;
    
    cout << "内参数：" << endl;
    cout << "[fx, fy] = ";
    cout << "[" << cameraMatrix.at<double>(0, 0) << ", " << cameraMatrix.at<double>(1, 1) << "]";
    cout << " +/- [" << stdDevIntrinsics.at<double>(0) << ", " << stdDevIntrinsics.at<double>(1) << "]" << endl;
    cout << "[Cx, Cy] = ";
    cout << "[" << cameraMatrix.at<double>(0, 2) << ", " << cameraMatrix.at<double>(1, 2) << "]";
    cout << " +/- [" << stdDevIntrinsics.at<double>(2) << ", " << stdDevIntrinsics.at<double>(3) << "]" << endl;
    cout << "[k1, k2, p1, p2, k3] = ";
    cout << "[" << distCoeffs.at<double>(0) << ", " << distCoeffs.at<double>(1) << ", ";
    cout << distCoeffs.at<double>(2) << ", " << distCoeffs.at<double>(3) << ", ";
    cout << distCoeffs.at<double>(4) << "]";
    cout << " +/- [" << stdDevIntrinsics.at<double>(4) << ", " << stdDevIntrinsics.at<double>(5) << ", ";
    cout << stdDevIntrinsics.at<double>(6) << ", " << stdDevIntrinsics.at<double>(7) << ", ";
    cout << stdDevIntrinsics.at<double>(8) << "]" << endl << endl;

    //cout << "误差：" << endl;
    //cout << "[fx, fy] = ";
    //cout << "[" << stdDevIntrinsics.at<double>(0) << ", " << stdDevIntrinsics.at<double>(1) << "]" << endl;
    //cout << "[Cx, Cy] = ";
    //cout << "[" << stdDevIntrinsics.at<double>(2) << ", " << stdDevIntrinsics.at<double>(3) << "]" << endl;
    //cout << "[k1, k2, p1, p2, k3] = ";
    //cout << "[" << stdDevIntrinsics.at<double>(4) << ", " << stdDevIntrinsics.at<double>(5) << ", ";
    //cout << stdDevIntrinsics.at<double>(6) << ", " << stdDevIntrinsics.at<double>(7) << ", ";
    //cout << stdDevIntrinsics.at<double>(8) << "]" << endl;
    
    cout << "误差：" << endl;
    cout << "[k4, k5, k6] = ";
    cout << "[" << stdDevIntrinsics.at<double>(9) << ", " << stdDevIntrinsics.at<double>(10) << ", ";
    cout << stdDevIntrinsics.at<double>(11) << "]" << endl;
    cout << "[s1, s2, s3, s4] = ";
    cout << "[" << stdDevIntrinsics.at<double>(12) << ", " << stdDevIntrinsics.at<double>(13) << ", ";
    cout << stdDevIntrinsics.at<double>(14) << ", " << stdDevIntrinsics.at<double>(15) << "]" << endl;
    cout << "[tauX, tauY] = ";
    cout << "[" << stdDevIntrinsics.at<double>(16) << ", " << stdDevIntrinsics.at<double>(17) << "]" << endl << endl;

    for(int i = 0; i < stdDevExtrinsics.rows; i += 6) {
        cout << "R" << i / 6 + 1 << " = ";
        cout << "[" << stdDevExtrinsics.at<double>(i) << ", " << stdDevExtrinsics.at<double>(i + 1) << ", " << stdDevExtrinsics.at<double>(i + 2) << "]" << endl;
        cout << "t" << i / 6 + 1 << " = ";
        cout << "[" << stdDevExtrinsics.at<double>(i + 3) << ", " << stdDevExtrinsics.at<double>(i + 4) << ", " << stdDevExtrinsics.at<double>(i + 5) << "]" << endl << endl;
    }
    cout << endl;

    int size = perViewErrors.size();
    for(int i = 0; i < size; i++) {
        cout << "reprojectionError[" << i + 1 << "] = " << perViewErrors[i] << endl;
    }
    cout << endl;

    fout << "标定结果：" << endl;
    fout << "cameraMatrix = " << endl << cameraMatrix << endl << endl;
    fout << "distCoeffs = " << endl << distCoeffs << endl << endl;
    fout << "reprojectionError = " << endl << reprojectionError << endl << endl;

    system("pause");
    return 0;
}


/**
 * 数字转字符串
 * @param num
 * @return outStr
 */
string num2str(int num) {
    ostringstream s1;
    s1 << num;
    string outStr = s1.str();
    return(outStr);
}


/**
* 计算标定板角点位置
* @param boardSize
* @param squareSize
* @param corners
*/
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners) {
    for(int i = 0; i < boardSize.height; ++i) {
        for(int j = 0; j < boardSize.width; ++j) {
            corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
        }
    }
}


/**
* 计算投影误差
* @param objectPoints
* @param imagePoints
* @param rvecs
* @param tvecs
* @param cameraMatrix
* @param distCoeffs
* @param perViewErrors
*/
double computeReprojectionErrors(vector<vector<Point3f> >& objectPoints,
                                 vector<vector<Point2f> >& imagePoints,
                                 vector<Mat>& rvecs, vector<Mat>& tvecs,
                                 Mat& cameraMatrix, Mat& distCoeffs,
                                 vector<float>& perViewErrors) {
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    //perViewErrors.resize(objectPoints.size());

    for(i = 0; i < (int)objectPoints.size(); ++i) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        //perViewErrors[i] = (float)sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    return sqrt(totalErr / totalPoints);
}