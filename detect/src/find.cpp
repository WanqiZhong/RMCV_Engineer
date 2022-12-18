#include "detector.hpp"

void Detector::Detect_Run()
{
    Mat img;
    while (1)
    {
        img = ImgUpdate(img);
        if (!img.empty())
        {
            ImgProcessCorner(img, thresh, maxual);

            vector<vector<Point>> logo_R;
            logo_R = FindR(side_number);

            vector<vector<Point>> side;
            side = StoreSide(logo_R, img);

            for (int i = 0; i < side_number; i++)
            {
                DrawSide(img, side[i]);
            }
        }
        else
        {
            logger.info("Detector img is empty");
        }
        // this_thread::sleep_for(chrono::milliseconds(10));
    }
}

/// @brief process the RGB image to get the contours of corner(include cvtColor, threshold, GaussianBlur, findContours)
/// @param img current RGB frame image

void Detector::ImgProcessCorner(const Mat &img, int thresh, int maxual)
{
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    Mat gaussian;
    GaussianBlur(gray, gaussian, Size(3, 3), 2, 2);
    Mat thre;
    threshold(gaussian, thre, thresh, maxual, THRESH_BINARY_INV);

    Mat dst;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(thre, thre, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    dilate(thre, dst, kernel, Point(-1, -1), 1);
    medianBlur(dst, dst, 1);

    findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}

/// @brief find contours of logo "R" (include contourArea)
/// @param side_number the number of sides
/// @return vector<vector<Point>> logo_R
vector<vector<Point>> Detector::FindR(int side_number)
{
    vector<double> contour_area;
    vector<vector<Point>> logo_R;

    for (int i = 0; i < contours.size(); i++)
    {
        contour_area.push_back(contourArea(contours[i]));
    }
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours.size() - i - 1; j++)
        {

            if (contour_area[j] < contour_area[j + 1])
            {
                double temp = contour_area[j];
                contour_area[j] = contour_area[j + 1];
                contour_area[j + 1] = temp;
            }
        }
    }

    for (int i = 0; i < side_number; i++)
    {
        for (int j = 0; j < contours.size(); j++)
        {
            if (contour_area[i] == contourArea(contours[j]))
            {
                logo_R.push_back(contours[j]);
                break;
            }
        }
    }
    return logo_R;
}

/// @brief find the four corners closet to "R"
/// @param p the center of "R"
/// @return vector<vector<Point>> logo_R
vector<int> Detector::LenSort(Point p)
{
    double len = 0;

    Point corner_center;

    vector<double> len_temp;
    for (int i = 0; i < contours.size(); i++)
    {
        corner_center = Point(0, 0);
        for (int j = 0; j < contours[i].size(); j++)
        {
            corner_center.x += contours[i][j].x;
            corner_center.y += contours[i][j].y;
        }
        corner_center.x /= contours[i].size();
        corner_center.y /= contours[i].size();

        len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));
        len_temp.push_back(len);
    }
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours.size() - i - 1; j++)
        {
            if (len_temp[j] > len_temp[j + 1])
            {
                double temp = len_temp[j];
                len_temp[j] = len_temp[j + 1];
                len_temp[j + 1] = temp;
            }
        }
    }

    vector<int> min_temp;
    for (int j = 0; j < 5; j++)
    {
        for (int i = 0; i < contours.size(); i++)
        {

            corner_center = Point(0, 0);
            for (int j = 0; j < contours[i].size(); j++)
            {
                corner_center.x += contours[i][j].x;
                corner_center.y += contours[i][j].y;
            }
            corner_center.x /= contours[i].size();
            corner_center.y /= contours[i].size();

            len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));

            if (len == len_temp[j])
            {
                min_temp.push_back(i);
                break;
            }
        }
    }
    return min_temp;
}

/// @brief store all contours'points in one side
/// @param logo_R the contours of R
/// @return vector<vector<Point>> side;
vector<vector<Point>> Detector::StoreSide(vector<vector<Point>> logo_R, Mat img)
{
    // 找到R的中心点
    vector<Point> p_R;
    for (int i = 0; i < logo_R.size(); i++)
    {
        int sum_x = 0, sum_y = 0;
        for (int j = 0; j < logo_R[i].size(); j++)
        {
            sum_x += logo_R[i][j].x;
            sum_y += logo_R[i][j].y;
        }
        Point p_temp(sum_x / logo_R[i].size(), sum_y / logo_R[i].size());
        p_R.push_back(p_temp);
    }
    // 找到离R最近的5个轮廓的索引(包括R)
    vector<vector<int>> len_min;
    for (int i = 0; i < logo_R.size(); i++)
    {
        len_min.push_back(LenSort(p_R[i]));
    }

    // 将4个轮廓的点放入一个容器
    vector<vector<Point>> side;
    vector<Point> side_;
    for (int i = 0; i < logo_R.size(); i++)
    {
        vector<Point> side_temp;
        side_ = side_temp;
        for (int j = 1; j < 5; j++)
        {
            for (int k = 0; k < contours[len_min[i][j]].size(); k++)
            {
                side_.push_back(contours[len_min[i][j]][k]);
            }
        }
        side.push_back(side_);
    }
    return side;
}

/// @brief draw sides
/// @param img, the original picture
/// @param side, all contours'points in each side

void Detector::DrawSide(Mat img, vector<Point> side)
{
    vector<Point> hull;
    vector<Point> poly;

    convexHull(Mat(side), hull, false);

    approxPolyDP(hull, poly, 25, true);
    if (poly.size() == 4)
    {
        polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
    }
}
