#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/edge_filter.hpp>

using namespace cv;
using namespace std;

Mat toDarkChannel(const Mat& src, int coreSize = 15)
{
    Mat chanels[3];
    split(src, chanels);

    Mat darkMask = min(min(chanels[0], chanels[1]), chanels[2]);

    Mat darkChannel;
    erode(darkMask, darkChannel, getStructuringElement(MORPH_RECT, Size(coreSize, coreSize)));
    // dilate(darkChannel, darkChannel, getStructuringElement(MORPH_RECT, Size(2, 2)));
    return darkChannel;
}

// atomphospheric light estimation
Vec3f getAtmLight(const Mat& src, const Mat& darkChannel, float threshold = 0.001)
{
    Mat darkFlat = darkChannel.reshape(1, 1);
    Mat srcFlat = src.reshape(1, 1);

    Mat indices;
    sortIdx(darkFlat, indices, SORT_DESCENDING);

    int numPixels = static_cast<int>(src.rows * src.cols * threshold);

    Vec3f atmLight(0, 0, 0);
    for (int i = 0; i < numPixels; i++) {
        int idx = indices.at<int>(0, i);
        atmLight += srcFlat.at<Vec3f>(0, idx);
    }

    atmLight = atmLight / static_cast<float>(numPixels);
    return atmLight;
}

Mat getTransmittance(const Mat& src, const Vec3f& atmLight, float omega = 0.95, int coreSize = 15)
{
    Mat transmittance(src.size(), CV_32F, Scalar(1.0f));

    Mat channels[3];
    split(src, channels);

    const float minVal = 1e-6f; // to avoid division by zero
    for (int i = 0; i < 3; ++i) {
        channels[i] = channels[i] / max(minVal, atmLight[i]);
    }

    Mat normalized;
    merge(channels, 3, normalized);

    transmittance = 1.0f - omega * toDarkChannel(normalized, coreSize);

    return transmittance;
}

// ref : https://blog.csdn.net/piaoxuezhong/article/details/78372787
Mat guidedFilter(const Mat& p, const Mat& i, int r, float epsilon)
{ // p -> input, i -> guidance
    Mat meanI, meanP, meanIP, meanII, varI, covIP; // mean -> 局部均值, var -> 方差, cov -> 协方差

    boxFilter(i, meanI, CV_32F, Size(r, r));
    boxFilter(p, meanP, CV_32F, Size(r, r));
    boxFilter(i.mul(p), meanIP, CV_32F, Size(r, r));
    boxFilter(i.mul(i), meanII, CV_32F, Size(r, r));

    varI = meanII - meanI.mul(meanI);
    covIP = meanIP - meanI.mul(meanP);

    Mat a = covIP / (varI + epsilon);
    Mat b = meanP - a.mul(meanI);

    Mat meanA, meanB;
    boxFilter(a, meanA, CV_32F, Size(r, r));
    boxFilter(b, meanB, CV_32F, Size(r, r));

    Mat output = meanA.mul(i) + meanB;
    return output;
}

Mat transmittanceRefine(const Mat& src, const Mat& initTransmittance, int r = 45, float epsilon = 0.0001)
{
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY, 1);

    ximgproc::guidedFilter(gray, initTransmittance, gray, r, epsilon);

    return gray;

    // return guidedFilter(initTransmittance, gray, r, epsilon);
}

Mat dehaze(const Mat& src, const Mat& transmittance, const Vec3f& atmLight, const float minTrans = 0.01)
{
    Mat output(src.size(), CV_32FC3);

    Mat channels[3];
    split(src, channels);

    Mat limitTrans;
    max(transmittance, minTrans, limitTrans);

    for (int i = 0; i < 3; ++i) {
        channels[i] = (channels[i] - atmLight[i]) / limitTrans;
        channels[i] = channels[i] + atmLight[i];
    }

    merge(channels, 3, output);

    return output;
}

int main(int argc, char* argv[])
{
    Mat image = imread("../res/Fog.png");

    if (image.empty()) {
        cout << "Could not open or find the image!" << endl;
        return -1;
    }

    // if (argc >= 2) {
    //     try {
    //         image = imread(argv[1]);
    //     } catch (const cv::Exception& e) {
    //         cerr << e.msg << endl;
    //         return -1;
    //     }
    // } else {
    //     cout << "Please provide an image path." << endl;
    //     return -1;
    // }

    namedWindow("Dark Channel", WINDOW_NORMAL);
    namedWindow("transmittance", WINDOW_NORMAL);
    namedWindow("Original Image", WINDOW_NORMAL);
    namedWindow("Dehazed Image", WINDOW_NORMAL);
    namedWindow("Initial Transmittance", WINDOW_NORMAL);

    Mat img_32f;
    image.convertTo(img_32f, CV_32FC3, 1.0f / 255.0f);

    Mat darkChannel = toDarkChannel(img_32f);
    Vec3f atmLight = getAtmLight(img_32f, darkChannel);
    Mat transmittance = getTransmittance(img_32f, atmLight, 0.95, 15);

    Mat showTransInit = transmittance.clone();
    // pyrDown(transmittance, showTransInit);

    imshow("Initial Transmittance", showTransInit);
    //waitKey(1);

    transmittance = transmittanceRefine(img_32f, transmittance, 60, 0.0001);
    Mat dehazedImage = dehaze(img_32f, transmittance, atmLight, 0.1);

    cout << "Atmospheric Light: " << atmLight << endl;

    // pyrDown(image, image);
    // pyrDown(darkChannel, darkChannel);
    // pyrDown(img_32f, img_32f);
    // pyrDown(dehazedImage, dehazedImage);
    // pyrDown(transmittance, transmittance);

    imshow("Dark Channel", darkChannel);
    //waitKey(1);

    imshow("transmittance", transmittance);
    //waitKey(1);

    imshow("Original Image", img_32f);
    //waitKey(1);

    imshow("Dehazed Image", dehazedImage);
    //waitKey(1);

    waitKey(0);
    destroyAllWindows();
    return 0;
}