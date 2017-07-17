#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main() {
    // �궨����ͼ���ļ���·��
    ifstream fin("calibdata1.txt");
    // ����궨������ļ�
    ofstream fout("caliberation_result.txt");
    // �궨�������̸��ڽǵ�ĸ���
    Size boardSize(11, 8);

    // ÿ�ж����ͼ��·��
    string fileName;
    // ������ͼ������
    vector<Mat> imageSet;
    // ÿ��ͼ��Ĵ�С
    Size imageSize;
    // ͼ����
    int imgCount;

    // ��ȡͼ��
    while(getline(fin, fileName)) {
        Mat img = imread(fileName);
        imageSet.push_back(img);
    }
    imageSize = imageSet[0].size();
    imgCount = imageSet.size();

    // ��ͼ��
    for(int i = 0; i < imgCount; i++) {
        
    }

}


