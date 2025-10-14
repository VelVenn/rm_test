#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace cv;
using namespace std;

class RoiSelector {
private:
    Mat img, buffer;
    Point startPoint, endPoint;
    bool selecting = false;
    Rect roi;
    string windowName = "Image";

private:
    stringstream ss;

    static void callback(int event, int x, int y, int flags, void* userdata)
    {
        RoiSelector* self = static_cast<RoiSelector*>(userdata); // Cast the userdata to the class type
        self->reactor(event, x, y, flags);
    } // The callback function for setMouseCallback must be static

    void reactor(int event, int x, int y, int flags)
    {
        img.copyTo(buffer);

        if (x > 0 && y > 0 && x <= img.cols && y <= img.rows) {
            Vec3i color = img.at<Vec3b>(y, x); // at<type>(row, col)
            ss << "(" << x << ", " << y << "): "
               << "[ "
               << "R: " << color[2] << ", G: " << color[1] << ", B: " << color[0] << " ]";
            putText(buffer, ss.str(), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

            ss.str("");
            ss.clear();
        }

        switch (event) {
        case EVENT_LBUTTONDOWN:
            startPoint = Point(x, y);
            selecting = true;
            break;
        case EVENT_MOUSEMOVE:
            if (selecting) {
                endPoint = Point(x, y);
                roi = Rect(
                    min(startPoint.x, endPoint.x),
                    min(startPoint.y, endPoint.y),
                    abs(startPoint.x - endPoint.x),
                    abs(startPoint.y - endPoint.y)); // Use min and abs to ensure the ROI is valid dragging in any direction
                rectangle(buffer, roi, Scalar(0, 255, 0), 2);
            }
            break;
        case EVENT_LBUTTONUP:
            selecting = false;
            if (roi.width > 0 && roi.height > 0) {
                Mat imgCrop = img(roi);
                Point centre = Point(roi.x + roi.width / 2, roi.y + roi.height / 2);
                cout << "Centre: " << centre << endl;
                imshow("Cropped Image", imgCrop);
            }
            roi = Rect(); // Reset ROI
        }

        imshow(windowName, buffer);
    }

public:
    RoiSelector(const string& filename, const string& label = "Image")
    {
        img = imread(filename);
        if (img.empty()) {
            throw runtime_error("Could not open or find the image");
        }
        windowName = label;
    }
    // Reference is an alias, it's a new name for the same object
    // Const reference is a read-only reference, using to avoid extraneous copy

    ~RoiSelector()
    {
        img.release();
        buffer.release();
        destroyAllWindows();
    }

    void monitor()
    {
        imshow(windowName, img);
        setMouseCallback(windowName, callback, this);
    }
};

int main()
{
    try {
        RoiSelector rs("../res/Cat.png");
        rs.monitor();

        if (waitKey(0) == 27) {
            return 0;
        }
    } catch (const exception& e) {
        cerr << e.what() << endl;
        return -1;
    }
}
