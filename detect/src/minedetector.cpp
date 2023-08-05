#include "minedetector.hpp"

void Minedetector::Detector_Run(Mat &img)
{   
    int offset = 100;
    valid_cnt = 0;
    valid_contours.clear();
    all_contours.clear();
    square_contour.clear();
    first_points.clear();
    umt::Subscriber<std::vector<std::vector<cv::Point>>> sub("Armor");
    vector<vector<Point>> armors;
    anchor_point.clear();
    logger.critical("Waiting for UMT::[Armor]");
    armors = sub.pop();
    logger.info("armors size:{}", armors.size());
    if(!armors.empty()){
        get_corner_withnet(img, armors);
    }    
    // if(!first_points.empty() && !third_points.empty()){
    //     find_corner(img);
    // }
}
void Minedetector::drawLine(Mat &img,            // 要标记直线的图像
                            vector<Vec2f> lines, // 检测的直线数据
                            double rows,         // 原图像的行数（高）
                            double cols,         // 原图像的列数（宽）
                            Scalar scalar,       // 绘制直线的颜色
                            int n                // 绘制直线的线宽
)
{
    Point pt1, pt2;
    for (size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0];           // 直线距离坐标原点的距离
        float theta = lines[i][1];         // 直线过坐标原点垂线与x轴夹角
        double a = cos(theta);             // 夹角的余弦值
        double b = sin(theta);             // 夹角的正弦值
        double x0 = a * rho, y0 = b * rho; // 直线与过坐标原点的垂线的交点
        double length = max(rows, cols);   // 图像高宽的最大值
                                         // 计算直线上的一点
        pt1.x = cvRound(x0 + length * (-b));
        pt1.y = cvRound(y0 + length * (a));
        // 计算直线上另一点
        pt2.x = cvRound(x0 - length * (-b));
        pt2.y = cvRound(y0 - length * (a));
        // 两点绘制一条直线
        line(img, pt1, pt2, scalar, n);
    }
}

void Minedetector::get_corner_withnet(Mat &img, vector<vector<Point>>& anchor_point)
{
    enhance_img(img);
    Mat canvas = img.clone();
    if(!anchor_point.empty()){
       logger.warn("{}", anchor_point[0][0].x);
    }
    
    if(!anchor_point.empty()){
        for (auto &mine_side : anchor_point)
        {
            int x = 0;
            int y = 0;
            int x_max = 0;
            int y_max = 0;
            int bound_small = param.bound_small;
            int bound_big = param.bound_big;
            int w = param.w;
            int h = param.h;
            square_contour.clear();
            for (int t = 0; t < mine_side.size(); t++)
            {
                Mat img_ori = img.clone();
                Mat mask = Mat::zeros(img.size(), CV_8UC1);
                vector<Point> border_points;
                switch (t)
                {
                case 0:
                    // logger.warn("first: {}, {}", mine_side[t].x, mine_side[t].y);
                    // first_points.push_back(mine_side[t]);
                    x = max(mine_side[t].x - bound_small, 0);
                    y = max(mine_side[t].y - bound_small, 0);
                    x_max = min(x + w, img.cols);
                    y_max = min(y + h, img.rows);
                    border_points.push_back(Point(x, y));
                    border_points.push_back(Point(x_max, y_max));
                    for (int i = x; i < x_max; i++)
                    {
                        for (int j = y; j < y_max; j++)
                        {
                            mask.at<uchar>(j, i) = 255;
                        }
                    }
                    break;
                case 1:
                    x = max(mine_side[t].x - bound_small, 0);
                    y = max(mine_side[t].y - bound_big, 0);
                    x_max = min(x + w, img.cols);
                    y_max = min(y + h, img.rows);
                    border_points.push_back(Point(x, y));
                    border_points.push_back(Point(x_max, y_max));
                    for (int i = x; i < x_max; i++)
                    {
                        for (int j = y; j < y_max; j++)
                        {
                            mask.at<uchar>(j, i) = 255;
                        }
                    }
                    break;
                case 2:
                    // third_points.push_back(mine_side[t]);
                    x = max(mine_side[t].x - 120, 0);
                    y = max(mine_side[t].y - 120, 0);
                    x_max = min(x + w, img.cols);
                    y_max = min(y + h, img.rows);
                    border_points.push_back(Point(x, y));
                    border_points.push_back(Point(x_max, y_max));
                    for (int i = x; i < x_max; i++)
                    {
                        for (int j = y; j < y_max; j++)
                        {
                            mask.at<uchar>(j, i) = 255;
                        }
                    }
                    square_contour.push_back(mine_side[2]);
                    break;
                case 3:
                    x = max(mine_side[t].x - 120, 0);
                    y = max(mine_side[t].y - 120, 0);
                    x_max = min(x + w, img.cols);
                    y_max = min(y + h, img.rows);
                    border_points.push_back(Point(x, y));
                    border_points.push_back(Point(x_max, y_max));
                    for (int i = x; i < x_max; i++)
                    {
                        for (int j = y; j < y_max; j++)
                        {
                            mask.at<uchar>(j, i) = 255;
                        }
                    }
                    break;
                }
                get_mask(img_ori, mask);
                // imshow("mask", mask);
                imshow("img_aft_mask", img_ori);
                waitKey(1);
                get_main_corner_withnet(img_ori, canvas, border_points, mine_side, t);
            }
            find_anchor(canvas);
        }          
    }
    imshow("aft_img", canvas);
    waitKey(1);
}

void Minedetector::get_main_corner_withnet(Mat &img, Mat &canvas, vector<Point> border, vector<Point>& net_point, int net_index)
{
    Mat gray_img;
    int corner_cnt = 0;
    double ratio_thres = param.ratio_thres;
    double area_ratio_thres = param.area_ratio_thres;
    vector<vector<Point>> contours;
    vector<CornerContour> min_distance_contours;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);
    threshold(gray_img, gray_img, param.corner_thresh, 255, THRESH_BINARY); // >130 -> White     < 130 -> black  (higher threshold, more black)
    threshold(gray_img, gray_img, 0, 255, THRESH_BINARY_INV);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    // 膨胀
    dilate(gray_img, gray_img, kernel);
    // 反色，黑色换白色
    imshow("threshold", gray_img);
    findContours(gray_img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // cout << "contours size:" << contours.size() << endl;
    while (corner_cnt <= 1 && ratio_thres >= param.ratio_thres_min && area_ratio_thres >= param.area_ratio_thres_min)
    {
        // logger.info("corner_cnt:{}", contours.size());
        for (auto &contour : contours)
        {
            if (contourArea(contour) > param.corner_contour_area_min && contourArea(contour) < param.corner_contour_area_max)
            {
                RotatedRect con_rect = minAreaRect(contour);
                double ratio = con_rect.size.width / con_rect.size.height;
                double area_ratio = contourArea(contour) / (con_rect.size.width * con_rect.size.height);
                putText(canvas, "area_ratio:" + to_string(area_ratio), Point(con_rect.center.x,con_rect.center.y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
                putText(canvas, "aera:" + to_string(contourArea(contour)), con_rect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
                // logger.info("ratio:{}", ratio);
                // logger.info("area_ratio:{}", area_ratio);
                if (con_rect.size.width * con_rect.size.height < param.corner_rec_area_min || con_rect.size.height == 0)
                {
                    continue;
                }
                // RotatedRect con_rect = minAreaRect(contour);
                if (ratio > 1 && ratio != 0)
                {
                    ratio = 1 / ratio;
                }
                if (ratio < ratio_thres || ratio == 1 || area_ratio < area_ratio_thres)
                {
                    continue;
                }
                Mat contour_binary = Mat::zeros(img.size(), CV_8UC1);
                vector<vector<Point>> contours_poly;
                contours_poly.push_back(contour);
                drawContours(contour_binary, contours_poly, -1, Scalar(255), -1);
                Mat canny = Mat::zeros(contour_binary.size(), CV_8UC1);
                Canny(contour_binary, canny, 50, 150);
                imshow("canny", canny);
                waitKey(1);

                vector<Vec4f> lines;
                HoughLinesP(canny, lines, 1, CV_PI / 180, 20, 5, 3);
                // logger.info("lines size:{}", lines.size());
                bool is_border = false;
                for (auto &l : lines)
                {
                    for (auto &bp : border)
                    {
                        if (l[0] == bp.x || l[1] == bp.y || l[2] == bp.x || l[3] == bp.y)
                        {
                            is_border = true;
                            break;
                        }
                    }
                    if (is_border)
                    {
                        break;
                    }
                }
                if (is_border)
                {
                    continue;
                }
                // if(!lines.empty())
                //     putText(canvas, "line.size():" + to_string(lines.size()), Point(con_rect.center.x,con_rect.center.y + 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);

                // vector<Vec3d> lines_3d;
                // double rhoMin = 5.0f;  //最小长度
                // double rhoMax = 10.0f;  //最大长度
                // double rhoStep = 1;  //离散化单位距离长度
                // double thetaMin = 0.0f;  //最小角度
                // double thetaMax = CV_PI / 2.0f;  //最大角度
                // double thetaStep = CV_PI / 180.0f;  ////离散化单位角度弧度
                // HoughLinesPointSet(contour, lines_3d, 20, 1, rhoMin, rhoMax, rhoStep,thetaMin, thetaMax, thetaStep);
                // logger.info("lines_3d size:{}", lines_3d.size());
                // drawLine(canvas, lines, canvas.rows, canvas.cols, Scalar(0, 0, 255), 2);
                // 划线
                // for (auto &l : lines)
                // {
                //     line(canvas, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
                // }
                // if (lines.size() < 1)
                // {
                //     continue;
                // }

                //用轮廓周长与外接矩形周长比值判断
                double rec_length = (con_rect.size.width + con_rect.size.height) * 2;
                double length_rate = arcLength(contour, true) / rec_length;
                // putText(canvas, "length_rate:" + to_string(length_rate), con_rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);
                if(length_rate > 1.2)continue;

                // 计算轮廓的矩
                cv::Moments moments = cv::moments(contour);
                // 计算轮廓的中心点坐标
                double cX = moments.m10 / moments.m00;
                double cY = moments.m01 / moments.m00;
                cv::circle(canvas, Point((int)cX, (int)cY), 5, cv::Scalar(0, 255, 0), 5);

                //计算网络识别结果的中心点座标
                CornerContour cornercontour;
                cornercontour.contour = contour;
                double distance = 0;
                double nX = net_point[net_index].x;
                double nY = net_point[net_index].y;
                distance = pow(nX - cX, 2) + pow(nY - cY, 2);
                cornercontour.distance = distance;
                min_distance_contours.push_back(cornercontour);

                // cout << "net_point.size():" << net_point.size() << endl;
                // if(!net_point.empty()){
                    
                    // distance = pow(nX - cX, 2) + pow(nY - cY, 2);
                    // cv::circle(canvas, Point((int)nX, (int)nY), 5, cv::Scalar(0, 255, 255), 5);
                    
                    //if(distance < 30 || distance > 200) continue;
                // }
                // putText(canvas, "distance:" + to_string(distance), Point(con_rect.center.x,con_rect.center.y + 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);

                corner_cnt++;

                Point2f con_points[4];
                con_rect.points(con_points);
                vector<Point> con_point;
                for (int i = 0; i < 4; i++)
                {
                    con_point.push_back(con_points[i]);
                }
                polylines(canvas, con_point, true, Scalar(0, 255, 0), 2);

            }
        }
        ratio_thres -= 0.1;
        area_ratio_thres -= 0.1;
    }
    sort(min_distance_contours.begin(),min_distance_contours.end(),[](CornerContour a, CornerContour b){
        return a.distance < b.distance;
    });
    for(auto& min_distance_contour:min_distance_contours){
        valid_cnt ++;
        valid_contours.push_back(min_distance_contour.contour);
    }
}

Point Minedetector::getTargetPoint(cv::Point pt_center, cv::Mat warpMatrix)
{

    cv::Mat_<double> mat_pt(3, 1);
    mat_pt(0, 0) = pt_center.x;
    mat_pt(0, 1) = pt_center.y;
    mat_pt(0, 2) = 1;
    Point pt_correct;
    Mat mat_tmp = warpMatrix * mat_pt;
    double a1 = mat_tmp.at<double>(0, 0);
    double a2 = mat_tmp.at<double>(1, 0);
    double a3 = mat_tmp.at<double>(2, 0);
    pt_correct = Point(a1, a2);
    return pt_correct;
}

void Minedetector::perspective_transformation(const vector<Point2f> &final_points, Mat &gray_src, Mat &src)
{
    Point2f _srcTriangle[4];
    Point2f _dstTriangle[4];
    vector<Point2f> srcTriangle(_srcTriangle, _srcTriangle + 4);
    vector<Point2f> dstTriangle(_dstTriangle, _dstTriangle + 4);
    Mat after_transform;
    Mat after_transform_gray;

    const int leftTopX = final_points[0].x;
    const int leftTopY = final_points[0].y;
    const int rightTopX = final_points[1].x;
    const int rightTopY = final_points[1].y;
    const int rightDownX = final_points[2].x;
    const int rightDownY = final_points[2].y;
    const int leftDownX = final_points[3].x;
    const int leftDownY = final_points[3].y;

    int newWidth = 1000;
    int newHeight = 1000;

    after_transform = Mat::zeros(newHeight, newWidth, src.type());
    after_transform_gray = Mat::zeros(newHeight, newWidth, gray_src.type());

    srcTriangle[0] = Point2f(leftTopX, leftTopY);
    srcTriangle[1] = Point2f(leftDownX, leftDownY);
    srcTriangle[2] = Point2f(rightDownX, rightDownY);
    srcTriangle[3] = Point2f(rightTopX, rightTopY);

    int base_x = 2000;
    int base_y = (int)((double)2000 / 1.77);
    dstTriangle[0] = Point2f(base_x, base_y);
    dstTriangle[1] = Point2f(base_x, base_y + offset);
    dstTriangle[2] = Point2f(base_x + offset, base_y + offset);
    dstTriangle[3] = Point2f(base_x + offset, base_y);

    Mat m1 = Mat(srcTriangle);
    Mat m2 = Mat(dstTriangle);
    Mat status;
    Mat h = findHomography(m1, m2, status, 0, 3);
    cout << "h: " << h << endl;
    // imshow("src",src);
    perspectiveTransform(srcTriangle, dstTriangle, h);
    warpPerspective(src, after_transform, h, Size(src.size().width * 2, src.size().height * 2), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
    warpPerspective(gray_src, after_transform_gray, h, Size(src.size().width * 2, src.size().height * 2), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
    square_contour[0] = getTargetPoint(square_contour[0], h);
    // square_contour[0] = outputPoint;

    imshow("after_transform", after_transform);
    imshow("after_transform_gray", after_transform_gray);
    get_corner(after_transform_gray, after_transform);

    // 求透视变换投影回去的矩阵
    Mat h_inv = h.inv();
    Mat src_inv = Mat::zeros(src.size(), src.type());
    warpPerspective(after_transform, src_inv, h_inv, src.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
    for (auto &point : anchor_point)
    {
        for (auto &p : point)
        {
            p = getTargetPoint(p, h_inv);
        }
    }
    imshow("src_inv", src_inv);

    // debug->show_img("after_transform", after_transform);
}

void Minedetector::get_corner(Mat &gray_img, Mat &img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(gray_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    for (auto &contour : contours)
    {
        if (contourArea(contour) > 2000)
        {
            Rect con_rect = boundingRect(contour);
            // RotatedRect con_rect = minAreaRect(contour);
            // RotatedRect con_rect = minAreaRect(contour);
            // double ratio = con_rect.width / con_rect.height;
            // if(ratio > 1){
            //     ratio = 1 / ratio;
            // }
            // if(ratio < 0.2){
            //     continue;
            // }
            double area_ratio = contourArea(contour) / (con_rect.width * con_rect.height);
            double rect_area = con_rect.width * con_rect.height;
            // putText(img, "area " + to_string(rect_area), con_rect.tl(), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
            if (rect_area > 0.7 * pow(offset, 2) && rect_area < 2 * pow(offset, 2) && area_ratio > 0.3)
            {
                valid_contours.push_back(contour);
                valid_cnt++;
            }
        }
    }
    // find_anchor(img);
    imshow("debug", img);
    waitKey(1);
}

void Minedetector::enhance_img(Mat &img)
{
    cvtColor(img, img, COLOR_BGR2HSV);
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            img.at<Vec3b>(i, j)[2] = img.at<Vec3b>(i, j)[2] * 1.2 > 255 ? 255 : img.at<Vec3b>(i, j)[2] * 1.2;
        }
    }
    cvtColor(img, img, COLOR_HSV2BGR);
}

void Minedetector::get_mine(Mat &img, Mat &mask)
{
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            if (mask.at<uchar>(i, j) != 255)
            {
                img.at<Vec3b>(i, j)[0] = 255;
                img.at<Vec3b>(i, j)[1] = 255;
                img.at<Vec3b>(i, j)[2] = 255;
            }
        }
    }
}

void Minedetector::get_mask(Mat &img, Mat &mask)
{
    for (int i = 0; i < img.rows; ++i)
    {
        for (int j = 0; j < img.cols; ++j)
        {
            if (mask.at<uchar>(i, j) == 0)
            {
                img.at<Vec3b>(i, j)[0] = 255;
                img.at<Vec3b>(i, j)[1] = 255;
                img.at<Vec3b>(i, j)[2] = 255;
            }
        }
    }
}

void Minedetector::find_anchor(Mat &img)
{
    DebugUI debug_ui;
    debug_ui.right_flag = false;
    debug_ui.area = 0;
    debug_ui.match_rate = 0;
    debug_ui.min_index = 0;
    debug_ui.small_square_point.clear();
    debug_ui.small_square_area.clear();
    debug_ui.poly.clear();
    // logger.info("valid_cnt:{}", valid_cnt);

    
    if (valid_cnt < 4)
    {
        logger.warn("Wrong number of anchor_poly:{}", valid_cnt);
        // if(!anchor_contour.empty())
        //     anchor_point.push_back(anchor_contour);
    }
    else if (valid_cnt >= 4)
    {
        int valid_index = 0;
        // 对于大于4个的统计数，遍历所有四个点的组合
        for (int i = 0; i < valid_contours.size(); i++)
        {
            for (int j = i + 1; j < valid_contours.size(); j++)
            {
                for (int k = j + 1; k < valid_contours.size(); k++)
                {
                    for (int l = k + 1; l < valid_contours.size(); l++)
                    {
                        vector<vector<Point>> temp;
                        bool valid = false;
                        Mat canvas = img.clone();
                        vector<Point> temp_station_contours;
                        temp.push_back(valid_contours[i]);
                        temp.push_back(valid_contours[j]);
                        temp.push_back(valid_contours[k]);
                        temp.push_back(valid_contours[l]);
                        for (auto &t : temp)
                        {
                            for (const auto &q : t)
                            {
                                temp_station_contours.push_back(q);
                            }
                        }
                        get_anchor(img, temp_station_contours, debug_ui, valid_index++);
                    }
                }
            }
        }
    }
    draw_debug_ui(img, debug_ui);
}

void Minedetector::get_anchor(Mat &img, const vector<Point> &four_station_contours, DebugUI &debug_ui, int index)
{

    vector<vector<Point>> temp_anchor_point;
    vector<double> distance = vector<double>(4, 0);
    vector<Point> anchor_temp;
    vector<Point> anchor_hull;
    vector<Point> anchor_poly;
    vector<vector<Point>> anchor_contours;
    vector<Vec4i> anchor_hierarchy;
    int min_index = 0;
    cv::Point2f vertices[4];
    Mat anchor_mask;

    convexHull(Mat(four_station_contours), anchor_hull, false);
    approxPolyDP(anchor_hull, anchor_poly, 25, true);

    if (!square_contour.empty())
    {
        for (int i = 0; i < 4; ++i)
        {
            distance[i] = sqrt((anchor_poly[i].x - square_contour[0].x) * (anchor_poly[i].x - square_contour[0].x) + (anchor_poly[i].y - square_contour[0].y) * (anchor_poly[i].y - square_contour[0].y));
        }
        double len_min = distance[0];
        for (int i = 1; i < 4; ++i)
        {
            if (distance[i] < len_min)
            {
                len_min = distance[i];
                min_index = i;
            }
        }
        for (int i = 0; i < 4; ++i)
        {
            anchor_temp.push_back(anchor_poly[(min_index + i + 2) % 4]);
        }
        temp_anchor_point.push_back(anchor_temp);
    }
    else
    {
        logger.info("square_contour is empty");
    }

    RotatedRect res_rect = minAreaRect(anchor_poly);
    double res_area = res_rect.size.width * res_rect.size.height;
    double poly_area = contourArea(anchor_poly);
    double res_rate = res_rect.size.width / res_rect.size.height;
    if(res_rate < 1)
        res_rate = 1 / res_rate;

    // polylines(img, anchor_poly, true, Scalar(255, 0, 0), 2, 8, 0);
    // putText(img, "match_rate:"+to_string(poly_area / res_area), anchor_poly[min_index] , FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);

    // res_rect.points(vertices);
    // for (int i = 0; i < 4; ++i){
    //     line(img, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
    // }
    writeImageRaw(index, img);
        

    if ( poly_area / res_area > debug_ui.match_rate && poly_area >= param.poly_area && res_rate < param.res_rate)
    {
        logger.warn("match rate:{}", poly_area / res_area);
        if(!temp_anchor_point.empty())
            anchor_point.push_back(temp_anchor_point[0]);

        // cout << "(in)anchor_point.size:" << anchor_point.size() << endl;
        debug_ui.match_rate = poly_area / res_area;
        debug_ui.poly = anchor_poly;
        debug_ui.area = res_area;
        debug_ui.min_index = min_index;
        debug_ui.small_square_point.clear();
        // cout << "param.match_rate:" <<param.match_rate << endl;
        if (debug_ui.match_rate < param.match_rate)
        {
            // cout << "debug_ui.match_rate:" << debug_ui.match_rate <<endl;
            anchor_point.clear();
            debug_ui.right_flag = false;
        }
        else
        {
            debug_ui.right_flag = true;
            cout << "right_flag:" << debug_ui.right_flag <<endl;
        }
    }
}

void Minedetector::draw_debug_ui(Mat &img, DebugUI &debug_ui)
{
    for (int i = 0; i < debug_ui.small_square_point.size(); ++i)
    {
        putText(img, "area:" + to_string(debug_ui.small_square_area[i]), debug_ui.small_square_point[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
        cv::circle(img, debug_ui.small_square_point[i], 5, cv::Scalar(0, 255, 255), 5);
    }
    if (!debug_ui.poly.empty())
    {
        cv::circle(img, debug_ui.poly[debug_ui.min_index], 5, cv::Scalar(255, 0, 255), 5);
        polylines(img, debug_ui.poly, true, Scalar(0, 255, 0), 2, 8, 0);
        putText(img, "poly_area:" + to_string(contourArea(debug_ui.poly)), Point(0, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
    }
    putText(img, "area:" + to_string(debug_ui.area), Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
    putText(img, "rate:" + to_string(debug_ui.match_rate), Point(0, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
    if (debug_ui.right_flag)
    {
        putText(img, "Right!", Point(0, 250), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 2, 5);
    }
    else
    {
        putText(img, "Wrong!", Point(0, 250), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2, 5);
    }
}

void Minedetector::find_corner(Mat &img)
{
    Mat gray_img;
    double max_cornere_ratio = 0;
    vector<Point> max_corner_contour;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);
    threshold(gray_img, gray_img, 160, 255, THRESH_BINARY);
    imshow("threshold", gray_img);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(gray_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    for (auto &contour : contours)
    {
        if (contourArea(contour) > 1000 && contourArea(contour) < 3000)
        {
            RotatedRect con_rect = minAreaRect(contour);
            // RotatedRect con_rect = minAreaRect(contour);
            double ratio = con_rect.size.width / con_rect.size.height;
            double area_ratio = contourArea(contour) / (con_rect.size.width * con_rect.size.height);
            // putText(img, "ratio:"+to_string(ratio), con_rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);
            // putText(img, "area_ratio:"+to_string(area_ratio), con_rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);
            if (ratio > 1)
            {
                ratio = 1 / ratio;
            }
            if (ratio < 0.6 || ratio == 1 || area_ratio < 0.6)
            {
                continue;
            }
            if (area_ratio > max_cornere_ratio)
            {
                max_cornere_ratio = area_ratio;
                max_corner_contour = contour;
            }
        }
    }
    if (!max_corner_contour.empty())
    {
        RotatedRect con_rect = minAreaRect(max_corner_contour);
        Point2f con_rect_vertices[4];
        vector<Point2f> con_rect_points;
        vector<Point> contour_hull;
        vector<Point> contour_poly;
        Point first_point = first_points[0];
        int min_distance_point_index = 0;
        double min_distance = DBL_MAX;
        convexHull(max_corner_contour, contour_hull);
        approxPolyDP(contour_hull, contour_poly, 10, true);
        square_contour.push_back(con_rect.center);
        circle(img, con_rect.center, 5, Scalar(0, 0, 255), 5);
        putText(img, "coord" + to_string(con_rect.center.x), con_rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2, 5);
        // for(int i = 0; i < contour_poly.size(); i++){
        //     line(img, contour_poly[i], contour_poly[(i + 1) % contour_poly.size()], Scalar(0, 255, 0), 2);
        // }
        // for(int i = 0; i < contour_hull.size(); i++){
        //     line(img, contour_hull[i], contour_hull[(i + 1) % contour_hull.size()], Scalar(0, 255, 0), 2);
        // }all_contours
        // if(contour_hull.size() == 4){
        // }
        con_rect.points(con_rect_vertices);
        for (int i = 0; i < 4; i++)
        {
            if (pow(con_rect_vertices[i].x - first_point.x, 2) + pow(con_rect_vertices[i].y - first_point.y, 2) < min_distance)
            {
                min_distance = pow(con_rect_vertices[i].x - first_point.x, 2) + pow(con_rect_vertices[i].y - first_point.y, 2);
                min_distance_point_index = i;
            }
        }
        sort(con_rect_points.begin(), con_rect_points.end(), [first_point](Point2f a, Point2f b)
             { return pow(a.x - first_point.x, 2) + pow(a.y - first_point.y, 2) < pow(b.x - first_point.x, 2) + pow(b.y - first_point.y, 2); });
        for (int i = 0; i < 4; i++)
        {
            putText(img, to_string(i), con_rect_points[i], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        perspective_transformation(con_rect_points, gray_img, img);
    }

    imshow("corner_img", img);
    waitKey(1);
}

void Minedetector::find_mine(Mat &img)
{
    // Up Light
    enhance_img(img);
    // imshow("enhance_img", img);
    Mat mine_hsv, mine_binary;
    Mat corner_img = img.clone();
    Mat max_mine_binary = Mat::zeros(img.size(), CV_8UC1);
    Mat mine_canny = Mat::zeros(img.size(), CV_8UC1);
    vector<Point> mine_hull;
    vector<vector<Point>> mine_contour;
    vector<Point> max_mine_contour = {};
    vector<vector<Point>> max_mine_contours = {};
    double max_area = 0;
    cvtColor(img, mine_hsv, COLOR_BGR2HSV);
    inRange(mine_hsv, Scalar(0, 86, 86), Scalar(180, 255, 255), mine_binary);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat hierary;
    // dilate(mine_binary, mine_binary, kernel);
    Canny(mine_binary, mine_canny, 150, 200);
    imshow("mine_canny", mine_canny);
    waitKey(1);
    findContours(mine_binary, mine_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // imshow("mine_binary", mine_binary);
    // waitKey(1);
    for (auto &contour : mine_contour)
    {
        if (contourArea(contour) > max_area)
        {
            max_area = contourArea(contour);
            max_mine_contour = contour;
        }
    }
    if (!max_mine_contour.empty())
    {
        max_mine_contours.push_back(max_mine_contour);
        drawContours(max_mine_binary, max_mine_contours, -1, Scalar(255), -1);
        kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
        dilate(max_mine_binary, max_mine_binary, kernel);
        get_mine(corner_img, max_mine_binary);
        imshow("corner_img", corner_img);
        find_corner(corner_img);
    }
    // for(auto& point: max_mine_contour){
    //     max_mine_binary.at<uchar>(point.x, point.y) = mine_binary.at<uchar>(point.x, point.y);
    // }
    // imshow("max_mine_binary", max_mine_binary);
    // waitKey(1);
    // convexHull(mine_binary, mine_hull);
    // return max_mine_binary;
}

// void Minedetector::Detector_Run(Mat &img)
// {
//     int offset = 100;
//     valid_cnt = 0;
//     valid_contours.clear();
//     all_contours.clear();
//     square_contour.clear();
//     first_points.clear();
//     Mat res_img = img.clone();
//     find_mine(img);
//     if (!anchor_point.empty())
//     {
//         circle(res_img, anchor_point[0][0], 5, Scalar(0, 0, 255), 5);
//         for (int i = 0; i < anchor_point[0].size(); i++)
//         {
//             putText(res_img, to_string(i), anchor_point[0][i], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
//             line(res_img, anchor_point[0][i], anchor_point[0][(i + 1) % anchor_point[0].size()], Scalar(0, 0, 255), 2);
//         }
//     }
//     imshow("res_img", res_img);
// }

// Mat Minedetector::get_gold_mine(Mat &img, Mat &colorhist)
// {
//     vector<vector<Point>> side_contours;
//     vector<Vec4i> side_hierarchy;
//     vector<vector<Point>> hull;

//     //正式代码中注释掉
//     // cvtColor(colorhist, colorhist, COLOR_BGR2GRAY);

//     findContours(colorhist, side_contours, side_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//     for (int i = 0; i < side_contours.size(); i++)
//     {
//         RotatedRect rec_temp = minAreaRect(side_contours[i]);
//         double rate = float(rec_temp.size.width) / float(rec_temp.size.height);
//         double area = float(rec_temp.size.width) * float(rec_temp.size.height);
//         if (rate >= 0.5 && rate <= 2.5 && area >= 80000)
//         {
//             // Point2f p[4];
//             // rec_temp.points(p);
//             // for (int i = 0; i < 4; i++)
//             // {
//             //     line(img, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
//             // }

//             vector<Point> hull_temp;
//             convexHull(Mat(side_contours[i]), hull_temp, false); // 寻找金矿的凸包
//             hull.push_back(hull_temp);
//         }
//     }

//     // 抠图
//     Mat mask;
//     mask = Mat::zeros(img.size(), CV_8UC1); // 设置蒙版
//     for (int i = 0; i < hull.size(); i++)
//     {
//         fillPoly(mask, hull[i], Scalar(255, 255, 255)); // 将凸包区域设置为白色
//     }

//     // mask(rec).setTo(255);
//     img.copyTo(mask, mask);

//     // 背景变为白色
//     for (int i = 0; i < mask.rows; i++)
//     {
//         for (int j = 0; j < mask.cols; j++)
//         {
//             if (mask.at<Vec3b>(i, j)[0] == 0 && mask.at<Vec3b>(i, j)[1] == 0 && mask.at<Vec3b>(i, j)[2] == 0)
//             {
//                 mask.at<Vec3b>(i, j)[0] = 255;
//                 mask.at<Vec3b>(i, j)[1] = 255;
//                 mask.at<Vec3b>(i, j)[2] = 255;
//             }
//         }
//     }
//     return mask;
// }

// /// @brief process the RGB image to get the contours of corner(include cvtColor, threshold, GaussianBlur, findContours)
// /// @param img current RGB frame image
// /// @param thresh the threshold of function "threshold"
// /// @param maxval the max value of function "threshold"
// Mat Minedetector::process_img_corner(const Mat &img, int thresh, int maxval)
// {
//     Mat dst;
//     cvtColor(img, dst, COLOR_BGR2GRAY);
//     // GaussianBlur(dst, dst, Size(3, 3), 2, 2);
//     threshold(dst, dst, thresh, maxval, THRESH_BINARY_INV);

//     Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
//     morphologyEx(dst, dst, MORPH_CLOSE, kernel, Point(-1, -1), 3);
//     morphologyEx(dst, dst, MORPH_OPEN, kernel, Point(-1, -1), 5);
//     dilate(dst, dst, kernel, Point(-1, -1), 4);
//     erode(dst, dst, kernel, Point(-1, -1), 2);
//     medianBlur(dst, dst, 1);
//     vector<vector<Point>> contour_temp;
//     findContours(dst, contour_temp, gold_mine_hierarchy_2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // 寻找角点轮廓
//     for (int i = 0; i < contour_temp.size(); i++)
//     {
//         RotatedRect rec = minAreaRect(contour_temp[i]);
//         double area = rec.size.height * rec.size.width;
//         double rate = double(rec.size.height) / double(rec.size.width);
//         // cout<<rate<<endl;

//         // 筛选角点轮廓
//         if (contourArea(contour_temp[i]) >= 1000 && contourArea(contour_temp[i]) <= 15000 && area >= 4000 && area <= 18000 && rate >= 0.3 && rate <= 3)
//         {
//             Point2f p[4];
//             rec.points(p);
//             cout<<area<<endl;
//             for (int i = 0; i < 4; i++)
//             {
//                 line(img, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
//             }
//             gold_mine_contours_2.push_back(contour_temp[i]);
//         }
//     }

//     // cout<<"contours.size():"<<gold_mine_contours_2.size()<<endl;

//     namedWindow("corner", WINDOW_NORMAL);
//     imshow("corner", dst);

//     // for (int i = 0; i < gold_mine_contours_2.size(); i++)
//     // {
//     //     Rect rec = boundingRect(gold_mine_contours_2[i]);
//     //
//     // }
//     return dst;
// }
// /// @brief find contours of logo "R" (include findContours)
// /// @param logo_R to store the contours of "R"
// /// @param process the image which have been processed
// /// @return int side_number
// int Detector::find_R(vector<vector<Point>> &logo_R, Mat process)
// {
//     vector<vector<Point>> contours;
//     vector<Vec4i> hierarchy;

//     Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
//     morphologyEx(process, process, MORPH_CLOSE, kernel, Point(-1, -1), 2);

//     findContours(process, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//     for (int i = 0; i < contours.size(); i++)
//     {
//         double len = arcLength(contours[i], true); // 获取轮廓周长
//         // cout << "contour len:" << len << endl;
//         RotatedRect rec = minAreaRect(contours[i]);
//         double rec_len = (rec.size.height + rec.size.width) * 2; // 获取外接旋转矩形周长
//         // cout << "rec len:" << rec_len << endl;
//         if (hierarchy[i][2] >= 0 && contourArea(contours[i]) >= 20000 && contourArea(contours[i]) <= 50000 && len > rec_len)
//         {
//             logo_R.push_back(contours[i]);

//             // Point2f p[4];
//             // rec.points(p);
//             // for (int i = 0; i < 4; i++)
//             // {
//             //     line(process, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
//             // }
//         }    vector<Point> poly;

//     }
//     // namedWindow("corner", WINDOW_NORMAL);
//     // imshow("corner", process);
//     int side_number = logo_R.size();
//     return side_number;
// }
// /// @brief find the four corners closest to "R"
// /// @param p the center of "R"
// /// @return vector<int> min
// vector<int> Detector::sort_length(Point p)
// {
//     double len = 0;
//     Point corner_center;
//     vector<double> len_temp;
//     // 找到各轮廓的中心点与R的中心点的距离并排序
//     for (int i = 0; i < gold_mine_contours_2.size(); i++)
//     {
//         corner_center = Point(0, 0);
//         for (int j = 0; j < gold_mine_contours_2[i].size(); j++)
//         {
//             corner_center.x += gold_mine_contours_2[i][j].x;
//             corner_center.y += gold_mine_contours_2[i][j].y;
//         }
//         corner_center.x /= gold_mine_contours_2[i].size();
//         corner_center.y /= gold_mine_contours_2[i].size();
//         len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));

//         len_temp.push_back(len);
//     }
//     for (int i = 0; i < gold_mine_contours_2.size(); i++)
//     {
//         for (int j = 0; j < gold_mine_contours_2.size() - i - 1; j++)
//         {
//             if (len_temp[j] > len_temp[j + 1])
//             {
//                 double temp = len_temp[j];
//                 len_temp[j] = len_temp[j + 1];
//                 len_temp[j + 1] = temp;
//             }
//         }
//     }
//     vector<int> min;
//     for (int j = 0; j <len_temp.size(); j++)
//     {
//         for (int i = 0; i < gold_mine_contours_2.size(); i++)
//         {
//             corner_center = Point(0, 0);
//             for (int k = 0; k < gold_mine_contours_2[i].size(); k++)//计算轮廓中心点
//             {
//                 corner_center.x += gold_mine_contours_2[i][k].x;
//                 corner_center.y += gold_mine_contours_2[i][k].y;
//             }
//             corner_center.x /= gold_mine_contours_2[i].size();
//             corner_center.y /= gold_mine_contours_2[i].size();

//             //计算轮廓中心点与R标中心点的距离
//             len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));
//             //cout<<len<<endl;

//             if (len == len_temp[j] && len >= 120 && len <= 350)
//             {
//                 min.push_back(i);
//                 // cout<<len<<endl;
//                 break;
//             }
//         }

//     }
//     //cout<<"min.size()"<<min.size()<<endl;
//     return min;
// }
// /// @brief store all contours'points in one side
// /// @param logo_R the contours of R
// /// @param square to store points of square corners
// /// @return vector<Point> side
// vector<vector<Point>> Detector::store_side(vector<vector<Point>> logo_R, vector<Point> &square, vector<int> &corner_number)
// {
//     // 找到R的中心点
//     vector<Point> p_R;
//     for (int i = 0; i < logo_R.size(); i++)
//     {
//         int sum_x = 0, sum_y = 0;
//         for (int j = 0; j < logo_R[i].size(); j++)
//         {
//             sum_x += logo_R[i][j].x;
//             sum_y += logo_R[i][j].y;
//         }
//         Point p_temp(sum_x / logo_R[i].size(), sum_y / logo_R[i].size());
//         p_R.push_back(p_temp);
//     }
//     // 找到角点的索引
//     vector<vector<int>> len_min;
//     for (int i = 0; i < logo_R.size(); i++)
//     {
//         len_min.push_back(sort_length(p_R[i]));
//     }
//     // 找到正方形角点
//     for (int i = 0; i < logo_R.size(); i++)
//     {
//         for (int j = 0; j < len_min[i].size(); j++)

//         {
//             RotatedRect rec = minAreaRect(gold_mine_contours_2[len_min[i][j]]);
//             double area = float(rec.size.width) * float(rec.size.height);
//             double rate = contourArea(gold_mine_contours_2[len_min[i][j]]) / area;
//             //cout<<"rate:"<<rate<<"   area:"<<area<<endl;

//             if (rate >= 0.6 && area >= 5000 && area <= 15000)
//             {
//                 square.push_back(gold_mine_contours_2[len_min[i][j]][0]);
//                 cout << square[0] << endl;
//             }
//         }
//     }
//     cout << "square.size():" << square.size() << endl;
//     cout << "logo_R.size():" << logo_R.size() << endl;
//     // cout<<logo_R[0][0]<<endl;

//     //  将4个轮廓的点放入一个容器
//     vector<vector<Point>> side;
//     vector<Point> side_;
//     for (int i = 0; i < logo_R.size(); i++)
//     {
//         vector<Point> side_temp;
//         side_ = side_temp;
//         if (len_min[i].size() >= 3)
//         {
//             for (int j = 0; j < len_min[i].size(); j++)
//             {
//                 for (int k = 0; k < gold_mine_contours_2[len_min[i][j]].size(); k++)
//                 {
//                     side_.push_back(gold_mine_contours_2[len_min[i][j]][k]);
//                 }
//             }
//             side.push_back(side_);
//         }
//         else
//             cout << "cannot find enough corner" << endl;

//         corner_number.push_back(len_min[i].size());
//     }
//     return side;
// }
// /// @brief draw sides
// /// @param img, the original picture
// /// @param side, all contours'points in each side
// /// @param squre, points of square corners
// void Detector::draw_side(Mat img, vector<Point> side, Point square, int corner_number)
// {
//     vector<Point> hull;
//     vector<Point> poly;
//     double len[4] = {0};
//     vector<vector<Point>> anchor_point;
//     anchor_point.clear();
//     vector<Point> anchor_temp;

//     if (corner_number == 4)
//     {
//         convexHull(Mat(side), hull, false);
//         approxPolyDP(hull, poly, 25, true);

//         cout << "poly.size():" << poly.size() << endl;
//     }
//     else if (corner_number == 3)
//     {

//         vector<Point> poly_temp;
//         vector<Point> hull_temp;
//         convexHull(Mat(side), hull_temp, false);
//         approxPolyDP(hull_temp, poly_temp, 5, true);
//         // cout<<poly_temp.size()<<endl;
//         // polylines(img, poly_temp, true, Scalar(255, 0, 0), 2, 8, 0);
//         if (poly_temp.size() != 5)
//         {
//             cout << "wrong number of poly_temp,number:" << poly_temp.size() << endl;
//         }
//         else
//         {

//             // 计算相邻两点间距，找到最短的两条线段
//             double len[5] = {0};
//             for (int i = 0; i < 5; i++)
//             {
//                 len[i] = sqrt((poly_temp[i].x - poly_temp[(i + 1) % 5].x) * (poly_temp[i].x - poly_temp[(i + 1) % 5].x) + (poly_temp[i].y - poly_temp[(i + 1) % 5].y) * (poly_temp[i].y - poly_temp[(i + 1) % 5].y));
//             }
//             int index[5] = {0, 1, 2, 3, 4};

//             for (int i = 0; i < 5; i++)
//             {
//                 for (int j = 0; j < 4 - i; j++)
//                 {
//                     if (len[j] > len[j + 1])
//                     {
//                         double temp = len[j];
//                         len[j] = len[j + 1];
//                         len[j + 1] = temp;
//                         int temp_index = index[j];
//                         index[j] = index[j + 1];
//                         index[j + 1] = temp_index;
//                     }
//                 }
//             }
//             Vec4i LineA, LineB;
//             LineA = {poly_temp[index[0]].x, poly_temp[index[0]].y, poly_temp[(index[0] + 1) % 5].x, poly_temp[(index[0] + 1) % 5].y};
//             LineB = {poly_temp[index[1]].x, poly_temp[index[1]].y, poly_temp[(index[1] + 1) % 5].x, poly_temp[(index[1] + 1) % 5].y};

//             // 求两直线的交点
//             double ka, kb;
//             ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); // 求出LineA斜率
//             kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); // 求出LineB斜率

//             Point2f crosspoint_temp;
//             crosspoint_temp.x = (ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kb);
//             crosspoint_temp.y = (ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb);
//             Point crosspoint;
//             crosspoint.x = int(crosspoint_temp.x);
//             crosspoint.y = int(crosspoint_temp.y);
//             poly_temp.push_back(crosspoint);

//             // 对加入交点的点集再次凸包和多边形拟合
//             convexHull(Mat(poly_temp), hull, false);
//             approxPolyDP(hull, poly, 25, true);

//             if (poly.size() != 4)
//             {
//                 cout << "wrong number of poly,number:" << poly.size() << endl;
//             }
//             else
//             {
//                 double k1 = (double)(poly[0].y - poly[1].y) / (double)(poly[0].x - poly[1].x);
//                 double k2 = (double)(poly[2].y - poly[3].y) / (double)(poly[2].x - poly[3].x);
//                 double k3 = (double)(poly[1].y - poly[2].y) / (double)(poly[1].x - poly[2].x);
//                 double k4 = (double)(poly[3].y - poly[0].y) / (double)(poly[3].x - poly[0].x);

//                 // cout<<"k1:"<<k1<<endl;
//                 // cout<<"k2:"<<k2<<endl;
//                 // cout<<"k3:"<<k3<<endl;
//                 // cout<<"k4:"<<k4<<endl;
//                 // polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
//                 if (abs(k1 - k2) >= 0.4 || abs(k3 - k4) >= 0.4)
//                 {
//                     cout << "cannot detect 3 clear corners" << endl;
//                     poly.clear();
//                 }
//             }
//         }
//     }
//     else
//     {
//         cout<<"wrong corner number:"<<corner_number<<endl;
//     }

//     if (poly.size() == 4)
//     {
//         for (int i = 0; i < 4; i++)
//         {
//             len[i] = sqrt((poly[i].x - square.x) * (poly[i].x - square.x) + (poly[i].y - square.y) * (poly[i].y - square.y));
//         }
//         double len_min = len[0];
//         int min_index = 0;
//         for (int i = 1; i < 4; i++)
//         {
//             if (len[i] < len_min)
//             {
//                 len_min = len[i];
//                 min_index = i;
//             }
//         }
//         min_index = (min_index + 2) % 4;
//         for (int i = 0; i < 4; i++)
//         {
//             // cout << poly[(min_index + i) % 4] << endl;
//             anchor_temp.push_back(poly[(min_index + i) % 4]);
//         }
//         anchor_point.push_back(anchor_temp);
//         polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
//     }
//     if (img.empty())
//     {
//         cout << "img is empty" << endl;
//     }
//     else
//         imshow("img_poly", img);
// }
