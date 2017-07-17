#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main() {
    // 标定所用图像文件的路径
    ifstream fin("calibdata1.txt");
    // 保存标定结果的文件
    ofstream fout("caliberation_result.txt");
    // 标定版上棋盘格内角点的个数
    Size boardSize(11, 8);

    // 每行读入的图像路径
    string fileName;
    // 读入后的图像序列
    vector<Mat> imageSet;
    // 每幅图像的大小
    Size imageSize;
    // 图像数
    int imgCount;

    // 读取图像
    while(getline(fin, fileName)) {
        Mat img = imread(fileName);
        imageSet.push_back(img);
    }
    imageSize = imageSet[0].size();
    imgCount = imageSet.size();

    // 逐图像
    for(int i = 0; i < imgCount; i++) {
        
    }

}


