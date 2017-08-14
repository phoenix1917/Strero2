#include <iostream>
#include <fstream>

#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>

#include "main.hpp"
#include "reconstruct.hpp"

using namespace cv;
using namespace std;


/**
 * usingBoard��������ʹ�õı궨��
 * @param boardNum   [Input]�궨����
 *                      �궨��1��9x7��35mm����ɫ��Ե
 *                      �궨��2��12x9��30mm����ɫ��Ե
 *                      �궨��3��10x9��90mm����ɫ��Ե
 *                      �궨��4��13x9��30mm����ɫ��Ե
 * @param boardSize  [Output]���ر궨�����̸��ڽǵ���Ŀ
 * @param squareSize [Output]���ر궨�巽��߳�
 */
void usingBoard(int boardNum, Size& boardSize, float& squareSize) {
    switch(boardNum) {
    case 1:
        boardSize = Size(8, 6);
        squareSize = 35.0;
        break;
    case 2:
        boardSize = Size(11, 8);
        squareSize = 30.0;
        break;
    case 3:
        boardSize = Size(9, 8);
        squareSize = 90.0;
        break;
    case 4:
        boardSize = Size(12, 8);
        squareSize = 30.0;
        break;
    default:
        cout << "�޶�Ӧ�궨��" << endl;
        break;
    }
}


/**
 * ����궨��ǵ�λ��
 * @param boardSize  [input]�궨���ڽǵ�size
 * @param squareSize [input]�궨�巽��߳�
 * @param corners    [output]���صĽǵ���λ��
 */
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners) {
    for(int i = 0; i < boardSize.height; ++i) {
        for(int j = 0; j < boardSize.width; ++j) {
            corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
        }
    }
}


/**
 * ����ͶӰ���
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


/**
 * �̶��������굽ָ��ֵ
 * @param K     [InputOutputArray] �ڲ�������(CV_64FC1)
 * @param point [InputArray] Ҫ�̶�����������λ��
 * @return      �Ƿ�ɹ�����
 */
bool fixPrinciplePoint(Mat& K, Point2f point) {
    if(K.size() == Size(3, 3)) {
        K.at<double>(0, 2) = point.x;
        K.at<double>(1, 2) = point.y;
        return true;
    } else {
        cout << "�ڲ��������С����ȷ������" << endl;
        return false;
    }
}


/**
 * extractFeatures ��ͼ���б���ȡSIFT����
 * @param imageNames     [input]ͼ�������б�
 * @param keyPoints4All  [output]����ͼ��������λ���б�
 * @param descriptor4All [output]����ͼ�������������б�
 * @param colors4All     [output]����ͼ�������������б�
 */
void extractFeatures(vector<string>& imageNames,
                     vector<vector<KeyPoint>>& keyPoints4All,
                     vector<Mat>& descriptor4All,
                     vector<vector<Vec3b>>& colors4All) {
    keyPoints4All.clear();
    descriptor4All.clear();
    colors4All.clear();
    Mat image;

    // ��ȡͼ�񣬻�ȡͼ�������㣬������
    Ptr<Feature2D> sift = xfeatures2d::SIFT::create(0, 3, 0.04, 10);

    for(auto it = imageNames.begin(); it != imageNames.end(); ++it) {
        vector<KeyPoint> keyPoints;
        Mat descriptor;
        
        image = imread(*it);
        if(image.empty()) {
            continue;
        }

        // ż�������ڴ����ʧ�ܵĴ���
        sift->detectAndCompute(image, noArray(), keyPoints, descriptor);
        //// ��������٣����ų���ͼ��
        //if(keyPoints.size() <= 10) {
        //    continue;
        //}

        // ����������λ�û�ȡ����ֵ
        vector<Vec3b> colors(keyPoints.size());
        for(int i = 0; i < keyPoints.size(); ++i) {
            Point2f& p = keyPoints[i].pt;
            colors[i] = image.at<Vec3b>(p.y, p.x);
        }

        keyPoints4All.push_back(keyPoints);
        descriptor4All.push_back(descriptor);
        colors4All.push_back(colors);
    }

    //// ������ͼ��û������㹻��������㣬�򽫷��ص��б�Ϊͼ���б�ĳ���
    //if(keyPoints4All.size() == 0) {
    //    int size = imageNames.size();
    //    keyPoints4All.resize(size);
    //    descriptor4All.resize(size);
    //    colors4All.resize(size);
    //}
}


/**
* extractFeatures ��ͼ���б���ȡSIFT����
* @param images         [input]ͼ���б�
* @param keyPoints4All  [output]����ͼ��������λ���б�
* @param descriptor4All [output]����ͼ�������������б�
* @param colors4All     [output]����ͼ�������������б�
*/
void extractFeatures(vector<Mat>& images,
                     vector<vector<KeyPoint>>& keyPoints4All,
                     vector<Mat>& descriptor4All,
                     vector<vector<Vec3b>>& colors4All) {
    keyPoints4All.clear();
    descriptor4All.clear();
    colors4All.clear();
    Mat image;

    // ��ȡͼ�񣬻�ȡͼ�������㣬������
    Ptr<Feature2D> sift = xfeatures2d::SIFT::create(0, 3, 0.04, 10);

    for(auto it = images.begin(); it != images.end(); ++it) {
        vector<KeyPoint> keyPoints;
        Mat descriptor;

        image = *it;
        // ż�������ڴ����ʧ�ܵĴ���
        sift->detectAndCompute(image, noArray(), keyPoints, descriptor);
        //// ��������٣����ų���ͼ��
        //if(keyPoints.size() <= 10) {
        //    continue;
        //}

        // ����������λ�û�ȡ����ֵ
        vector<Vec3b> colors(keyPoints.size());
        for(int i = 0; i < keyPoints.size(); ++i) {
            Point2f& p = keyPoints[i].pt;
            colors[i] = image.at<Vec3b>(p.y, p.x);
        }

        keyPoints4All.push_back(keyPoints);
        descriptor4All.push_back(descriptor);
        colors4All.push_back(colors);
    }

    //// ������ͼ��û������㹻��������㣬�򽫷��ص��б�Ϊͼ���б�ĳ���
    //if(keyPoints4All.size() == 0) {
    //    int size = imageNames.size();
    //    keyPoints4All.resize(size);
    //    descriptor4All.resize(size);
    //    colors4All.resize(size);
    //}
}


/**
 * matchFeatures ����ƥ��
 * @param query   ͼ��1���������������
 * @param train   ͼ��2���������������
 * @param matches ƥ����
 */
void matchFeatures(Mat& query, Mat& train, vector<DMatch>& matches) {
    vector<vector<DMatch> > knnMatches;
    BFMatcher matcher(NORM_L2);
    matcher.knnMatch(query, train, knnMatches, 2);

    //��ȡ����Ratio Test����Сƥ��ľ���
    float minDist = FLT_MAX;
    for(int r = 0; r < knnMatches.size(); ++r) {
        //Ratio Test
        if(knnMatches[r][0].distance > 0.6 * knnMatches[r][1].distance) {
            continue;
        }
        float dist = knnMatches[r][0].distance;
        if(dist < minDist) {
            minDist = dist;
        }
    }

    matches.clear();
    for(size_t r = 0; r < knnMatches.size(); ++r) {
        //�ų�������Ratio Test�ĵ��ƥ��������ĵ�
        if(knnMatches[r][0].distance > 0.6 * knnMatches[r][1].distance ||
           knnMatches[r][0].distance > 5 * max(minDist, 10.0f)) {
            continue;
        }
        //����ƥ���
        matches.push_back(knnMatches[r][0]);
    }
}


/**
 * getMatchedPoints ��ȡƥ���Ե�λ��
 * @param p1      
 * @param p2      
 * @param matches 
 * @param out_p1  
 * @param out_p2  
 */
void getMatchedPoints(vector<KeyPoint>& p1, vector<KeyPoint>& p2, 
                      vector<DMatch> matches,
                      vector<Point2f>& out_p1, vector<Point2f>& out_p2) {
    out_p1.clear();
    out_p2.clear();
    for(int i = 0; i < matches.size(); ++i) {
        out_p1.push_back(p1[matches[i].queryIdx].pt);
        out_p2.push_back(p2[matches[i].trainIdx].pt);
    }
}


/**
 * getMatchedColors ��ȡƥ���Ե�����ֵ
 * @param c1      
 * @param c2      
 * @param matches 
 * @param out_c1  
 * @param out_c2  
 */
void getMatchedColors(vector<Vec3b>& c1, vector<Vec3b>& c2, vector<DMatch> matches,
                      vector<Vec3b>& out_c1, vector<Vec3b>& out_c2) {
    out_c1.clear();
    out_c2.clear();
    for(int i = 0; i < matches.size(); ++i) {
        out_c1.push_back(c1[matches[i].queryIdx]);
        out_c2.push_back(c2[matches[i].trainIdx]);
    }
}


/**
 * �������󣬲��ֽ�����˫Ŀ��Σ�λ�˹�ϵ�����ڲ���ʹ����ͬ����
 * @param K    [InputArray] �ڲ�������
 * @param R    [OutputArray] camera2 �� camera1 ����ת����
 * @param T    [OutputArray] camera2 �� camera1 ��ƽ������
 * @param p1   [InputArray] ͬ������� camera1 ͼ���ϵĵ�����
 * @param p2   [InputArray] ͬ������� camera2 ͼ���ϵĵ�����
 * @param mask [InputOutputArray] ƥ����flag�����ܵ�ƥ��㣨inliers��������ֵ
 */
bool findTransform(Mat& K, Mat& R, Mat& T,
                   vector<Point2f>& p1, vector<Point2f>& p2, Mat& mask) {
    //�����ڲξ����ȡ����Ľ���͹������꣨�������꣩
    double focalLength = 0.5 * (K.at<double>(0) + K.at<double>(4));
    Point2d principlePoint(K.at<double>(2), K.at<double>(5));

    //����ƥ�����ȡ��������ʹ��RANSAC����һ���ų�ʧ���
    Mat E = findEssentialMat(p1, p2, focalLength, principlePoint, RANSAC, 0.999, 1.0, mask);
    if(E.empty()) {
        return false;
    }

    double feasibleCount = countNonZero(mask);
    cout << (int)feasibleCount << " -in- " << p1.size() << endl;
    //����RANSAC���ԣ�outlier��������50%ʱ������ǲ��ɿ���
    if(feasibleCount <= 15 || (feasibleCount / p1.size()) < 0.6) {
        return false;
    }

    //�ֽⱾ�����󣬻�ȡ��Ա任������inliers��Ŀ
    int passCount = recoverPose(E, p1, p2, R, T, focalLength, principlePoint, mask);

    //ͬʱλ���������ǰ���ĵ������Ҫ�㹻��
    if(((double)passCount) / feasibleCount < 0.7) {
        return false;
    }

    return true;
}


/**
 * �������󣬲��ֽ�����˫Ŀ��Σ�λ�˹�ϵ�����ڲ���ʹ�ò�ͬ����
 * @param K1   [InputArray] camera1 �ڲ�������
 * @param K2   [InputArray] camera2 �ڲ�������
 * @param R    [OutputArray] camera2 �� camera1 ����ת����
 * @param T    [OutputArray] camera2 �� camera1 ��ƽ������
 * @param p1   [InputArray] ͬ������� camera1 ͼ���ϵĵ�����
 * @param p2   [InputArray] ͬ������� camera2 ͼ���ϵĵ�����
 * @param mask [InputOutputArray] ƥ����flag�����ܵ�ƥ��㣨inliers��������ֵ
 */
bool findTransform(Mat& K1, Mat& K2, Mat& R, Mat& T,
                   vector<Point2f>& p1, vector<Point2f>& p2, Mat& mask) {
    //�����ڲξ����ȡ����Ľ���͹������꣨�������꣩
    double focalLength = 0.5 * (K1.at<double>(0) + K1.at<double>(4) +
                                K2.at<double>(0) + K2.at<double>(4));
    Point2d principlePoint((K1.at<double>(2) + K2.at<double>(2)) / 2,
                           (K1.at<double>(5) + K2.at<double>(5)) / 2);

    //����ƥ�����ȡ��������ʹ��RANSAC����һ���ų�ʧ���
    Mat E = findEssentialMat(p1, p2, focalLength, principlePoint, RANSAC, 0.999, 1.0, mask);
    if(E.empty()) {
        return false;
    }

    double feasibleCount = countNonZero(mask);
    cout << (int)feasibleCount << " -in- " << p1.size() << endl;
    //����RANSAC���ԣ�outlier��������50%ʱ������ǲ��ɿ���
    if(feasibleCount <= 15 || (feasibleCount / p1.size()) < 0.6) {
        return false;
    }

    //�ֽⱾ�����󣬻�ȡ��Ա任������inliers��Ŀ
    int passCount = recoverPose(E, p1, p2, R, T, focalLength, principlePoint, mask);

    //ͬʱλ���������ǰ���ĵ������Ҫ�㹻��
    if(((double)passCount) / feasibleCount < 0.7) {
        return false;
    }

    return true;
}


/**
 * ȥ����ƥ���ԣ�λ�ã�
 * @param p1   [InputOutputArray]
 * @param mask [InputArray]
 */
void maskoutPoints(vector<Point2f>& p1, Mat& mask) {
    vector<Point2f> p1_copy = p1;
    p1.clear();

    for(int i = 0; i < mask.rows; ++i) {
        if(mask.at<uchar>(i) > 0) {
            p1.push_back(p1_copy[i]);
        }
    }
}


/**
 * ȥ����ƥ���ԣ�����ֵ��
 * @param p1   [InputOutputArray]
 * @param mask [InputArray]
 */
void maskoutColors(vector<Vec3b>& p1, Mat& mask) {
    vector<Vec3b> p1_copy = p1;
    p1.clear();
    
    for(int i = 0; i < mask.rows; ++i) {
        if(mask.at<uchar>(i) > 0) {
            p1.push_back(p1_copy[i]);
        }
    }
}


/**
 * ���ǻ��ؽ��ռ�㣨�ڲ���ʹ����ͬ����
 * @param K  [InputArray] �ڲ�������
 * @param R  [InputArray] camera2 �� camera1 ����ת����
 * @param T  [InputArray] camera2 �� camera1 ��ƽ������
 * @param p1 [InputArray] ͬ������� camera1 ͼ���ϵĵ�����
 * @param p2 [InputArray] ͬ������� camera2 ͼ���ϵĵ�����
 * @param structure [OutputArray] �ؽ����Ŀռ�㣨������꣩
 */
void reconstruct(Mat& K, Mat& R, Mat& T,
                 vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure) {
    // ���������ͶӰ���󣨵�Ӧ�Ծ���K[R T]��triangulatePointsֻ֧��float�ͣ�CV_32FC1��
    Mat proj1(3, 4, CV_32FC1);
    Mat proj2(3, 4, CV_32FC1);

    // ��ʼ��camera1��ͶӰ����
    // ��������ϵ������camera1�����������ϵ��
    proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
    proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);
    // ��ʼ��camera2��ͶӰ����
    R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
    T.convertTo(proj2.col(3), CV_32FC1);

    Mat fK;
    K.convertTo(fK, CV_32FC1);
    proj1 = fK*proj1;
    proj2 = fK*proj2;

    // �����ؽ�
    triangulatePoints(proj1, proj2, p1, p2, structure);
}


/**
 * ���ǻ��ؽ��ռ�㣨�ڲ���ʹ�ò�ͬ����
 * @param K1 [InputArray] camera1 �ڲ�������
 * @param K2 [InputArray] camera2 �ڲ�������
 * @param R  [InputArray] camera2 �� camera1 ����ת����
 * @param T  [InputArray] camera2 �� camera1 ��ƽ������
 * @param p1 [InputArray] ͬ������� camera1 ͼ���ϵĵ�����
 * @param p2 [InputArray] ͬ������� camera2 ͼ���ϵĵ�����
 * @param structure [OutputArray] �ؽ����Ŀռ�㣨������꣩
 */
void reconstruct(Mat& K1, Mat& K2, Mat& R, Mat& T,
                 vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure) {
    // ���������ͶӰ���󣨵�Ӧ�Ծ���K[R T]��triangulatePointsֻ֧��float�ͣ�CV_32FC1��
    Mat proj1(3, 4, CV_32FC1);
    Mat proj2(3, 4, CV_32FC1);

    // ��ʼ��camera1��ͶӰ����
    // ��������ϵ������camera1�����������ϵ��
    proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
    proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);
    // ��ʼ��camera2��ͶӰ����
    R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
    T.convertTo(proj2.col(3), CV_32FC1);

    Mat fK1, fK2;
    K1.convertTo(fK1, CV_32FC1);
    K2.convertTo(fK2, CV_32FC1);
    proj1 = fK1*proj1;
    proj2 = fK2*proj2;

    // �����ؽ�
    triangulatePoints(proj1, proj2, p1, p2, structure);
}


/**
 * ���ǻ��ؽ��ռ�㣨�ڲ���ʹ����ͬ����
 * @param K  [InputArray] �ڲ�������
 * @param R1 [InputArray] camera1 ����������ϵ����ת����
 * @param T1 [InputArray] camera1 ����������ϵ��ƽ������
 * @param R2 [InputArray] camera2 ����������ϵ����ת����
 * @param T2 [InputArray] camera2 ����������ϵ��ƽ������
 * @param p1 [InputArray] ͬ������� camera1 ͼ���ϵĵ�����
 * @param p2 [InputArray] ͬ������� camera2 ͼ���ϵĵ�����
 * @param structure [OutputArray] �ؽ����Ŀռ�㣨�ռ����꣩
 */
void reconstruct(Mat& K, Mat& R1, Mat& T1, Mat& R2, Mat& T2,
                 vector<Point2f>& p1, vector<Point2f>& p2, vector<Point3f>& structure) {
    //���������ͶӰ���󣨵�Ӧ�Ծ���K[R T]��triangulatePointsֻ֧��float��
    Mat proj1(3, 4, CV_32FC1);
    Mat proj2(3, 4, CV_32FC1);

    R1.convertTo(proj1(Range(0, 3), Range(0, 3)), CV_32FC1);
    T1.convertTo(proj1.col(3), CV_32FC1);

    R2.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
    T2.convertTo(proj2.col(3), CV_32FC1);

    Mat fK;
    K.convertTo(fK, CV_32FC1);
    proj1 = fK*proj1;
    proj2 = fK*proj2;

    //�����ؽ�
    Mat structure4D;
    triangulatePoints(proj1, proj2, p1, p2, structure4D);

    structure.clear();
    structure.reserve(structure4D.cols);
    for(int i = 0; i < structure4D.cols; ++i) {
        //������꣬�������һ��Ԫ��תΪ�ռ�����
        Mat_<float> col = structure4D.col(i);
        col /= col(3);
        structure.push_back(Point3f(col(0), col(1), col(2)));
    }
}


/**
 * toPoints3D ���������ת��Ϊ�ռ����ꡣ����Ϊfloat�ͣ�CV_32FC1����
 * @param points4D [InputArray] �������㣬4xN
 * @param points3D [OutputArray] �ռ������
 */
void toPoints3D(Mat& points4D, Mat& points3D) {
    points3D = Mat::zeros(3, points4D.size().width, CV_32FC1);
    for(int i = 0; i < points4D.size().width; i++) {
        points3D.at<float>(0, i) = points4D.at<float>(0, i) / points4D.at<float>(3, i);
        points3D.at<float>(1, i) = points4D.at<float>(1, i) / points4D.at<float>(3, i);
        points3D.at<float>(2, i) = points4D.at<float>(2, i) / points4D.at<float>(3, i);
    }
}


/**
 * saveStructure �������λ�˺͵������굽�ļ�
 * @param fileName  
 * @param rotations 
 * @param motions   
 * @param structure 
 * @param colors    
 */
void saveStructure(string fileName, vector<Mat>& rotations,
                   vector<Mat>& motions, vector<Point3f>& structure,
                   vector<Vec3b>& colors) {
    int n = (int)rotations.size();

    FileStorage fs(fileName, FileStorage::WRITE);
    fs << "Camera Count" << n;
    fs << "Point Count" << (int)structure.size();

    fs << "Rotations" << "[";
    for(size_t i = 0; i < n; ++i) {
        fs << rotations[i];
    }
    fs << "]";

    fs << "Motions" << "[";
    for(size_t i = 0; i < n; ++i) {
        fs << motions[i];
    }
    fs << "]";

    fs << "Points" << "[";
    for(size_t i = 0; i < structure.size(); ++i) {
        fs << structure[i];
    }
    fs << "]";

    fs << "Colors" << "[";
    for(size_t i = 0; i < colors.size(); ++i) {
        fs << colors[i];
    }
    fs << "]";

    fs.release();
}


/**
 * ranging ������λ�߾���
 * @param structure ������������ϵ�µ����꣨��λ��mm��
 * @param R ��ת����
 * @param t ƽ������
 * @return ������ľ��루��λ��m��
 */
vector<double> ranging(Mat structure, Mat R, Mat t) {
    vector<double> range;
    // �����С���󣬷��ؿ�
    if(structure.size().height != 3 || R.size() != Size(3, 3) || t.size() != Size(1, 3)) {
        return range;
    }
    for(int i = 0; i < structure.size().width; i++) {
        Mat pointL, pointR;
        structure.colRange(i, i + 1).clone().convertTo(pointL, CV_64FC1);
        pointR = R * pointL + t;
        double distL, distR, distT, median;
        distL = sqrt(pointL.at<double>(0) * pointL.at<double>(0) +
                     pointL.at<double>(1) * pointL.at<double>(1) +
                     pointL.at<double>(2) * pointL.at<double>(2));
        distR = sqrt(pointR.at<double>(0) * pointR.at<double>(0) +
                     pointR.at<double>(1) * pointR.at<double>(1) +
                     pointR.at<double>(2) * pointR.at<double>(2));
        distT = sqrt(t.at<double>(0) * t.at<double>(0) +
                     t.at<double>(1) * t.at<double>(1) +
                     t.at<double>(2) * t.at<double>(2));
        median = 0.5 * sqrt(2 * distL * distL + 2 * distR * distR - distT * distT) / 1000;
        range.push_back(median);
    }
    return range;
}
    
