#include "sitedetector.hpp"

void Sitedetector::Detector_Run(Mat &img) {

    umt::Subscriber<cv::Mat> sub("channel0");
    umt::Publisher<MINE_POSITION_MSG> mine_sub("anchor_point_data");
    all_contours.clear();
    station_contours.clear();
    square_contour.clear();
    anchor_contour.clear();
    valid_contour.clear();
    find_corner(img);
    if (all_contours.empty()){
        logger.warn("ExchangeSite_Run can't find normal corner");
    }else if (square_contour.empty()){
        logger.warn("ExchangeSite_Run can't find square corner");
    }else{
        find_anchor(img);
        cv::imshow("debug_ui",img);
    }
    if(!anchor_point.empty()){
        reverse(anchor_point[0].begin(), anchor_point[0].end());
    }
    writeVideoRaw(img);
}

/// @brief get coordinates of exchange station
/// @param img, the original picture
/// @param corner_contour, points of corners' contours;
/// @param square_corner_contour, points of square corner's contour;
void Sitedetector::find_anchor(Mat &img)
{
    DebugUI debug_ui;
    debug_ui.right_flag = false;
    debug_ui.area = 0;
    debug_ui.match_rate = 0;
    debug_ui.min_index = 0;
    debug_ui.small_square_point.clear();
    debug_ui.small_square_area.clear();
    debug_ui.poly.clear();

    logger.info("corner_cnt:{}", corner_cnt);

    if (corner_cnt == 4){
        get_anchor(img, all_contours, debug_ui, 0);
    }else if(corner_cnt < 4){
        logger.warn("Wrong number of anchor_poly:{}", corner_cnt);
        if(!anchor_contour.empty())
            anchor_point.push_back(anchor_contour);
    }else if(corner_cnt > 4){
        int valid_index = 0;
        // 对于大于4个的统计数，遍历所有四个点的组合
        for (int i = 0; i < valid_contour.size(); i++)
        {
            for (int j = i + 1; j < valid_contour.size(); j++)
            {
                for (int k = j + 1; k < valid_contour.size(); k++)
                {
                    for (int l = k + 1; l < valid_contour.size() ; l++)
                    {
                        vector<vector<Point>> temp;
                        Mat canvas = img.clone();
                        vector<Point> temp_station_contours;
                        temp.push_back(valid_contour[i]);
                        temp.push_back(valid_contour[j]);
                        temp.push_back(valid_contour[k]);
                        temp.push_back(valid_contour[l]);
                        for(auto & t : temp) {
                            for (const auto &q: t) {
                                temp_station_contours.push_back(q);
                            }
                        }
                        get_anchor(canvas, temp_station_contours, debug_ui, valid_index++);
                    }
                }
            }
        }
    }
    draw_debug_ui(img, debug_ui);

}



/// @brief find the L-shaped corner and the square corner of the exchange changesite
/// @param img, the original picture
/// @return void
void Sitedetector::find_corner(Mat &img)
{
    vector<vector<Point>> corner_contours;
    vector<Vec4i> corner_hierarchy;
    double min_corner_area = 10000; // 记录当前最小的面积
    min_corner_index = 0; corner_cnt = 0;


    cvtColor(img, thresh_output, COLOR_BGR2HSV);

    if (param.camp == 0)
        inRange(thresh_output, Scalar(0, 0, 100), Scalar(180, 255, 255), thresh_output); // red
    else{
        inRange(thresh_output, Scalar(0, 0, 100), Scalar(180, 255, 255), thresh_output); // red
    }

    imshow("[FIND_CORNER_AFT]", thresh_output);
    findContours(thresh_output, corner_contours, corner_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < corner_contours.size(); i++)
    {
        RotatedRect rec = minAreaRect(corner_contours[i]);
        double rate = float(rec.size.width) / rec.size.height;
        double area = float(rec.size.width) * rec.size.height;

        if (rate >= param.site_min_rate && rate <= param.site_max_rate && \
            area >= param.site_min_area && area <= param.site_max_area && \
            contourArea(corner_contours[i]) / area <= param.site_area_rate){

            for (const auto & j : corner_contours[i]){
                all_contours.push_back(j);
            }
            valid_contour.push_back(corner_contours[i]);
            anchor_contour.push_back(corner_contours[i][0]);

            if (contourArea(corner_contours[i]) < min_corner_area)
            {
                min_corner_area = contourArea(corner_contours[i]);
                min_corner_index = i;
            }

            corner_cnt++;
        }
    }

    if (!corner_contours.empty()){
        // 得到最小面积角点（标志角点）的面积
        // 使用理论上正方形小角点的外接矩形面积应当小于最小角点（标志角点）的面积
        min_corner_rec = contourArea(corner_contours[min_corner_index]);
        // RotatedRect rec = minAreaRect(corner_contours[min_corner_index]);
        // 提取右上角标志角点的一个点单独储存，用于后续按顺序输出角点座标
        square_contour.push_back(corner_contours[min_corner_index][0]);
        putText(img, "square:" + to_string(corner_contours[min_corner_index][0].x) + "," + to_string(corner_contours[min_corner_index][0].y), Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
        putText(img, "min_aera:"+to_string(min_corner_rec), Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    }

}


void Sitedetector::get_anchor(Mat &img, const vector<Point>& four_station_contours, DebugUI &debug_ui, int index){

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

    anchor_mask = Mat::zeros(thresh_output.size(), CV_8UC1);
    fillPoly(anchor_mask, anchor_poly, Scalar(255, 255, 255));
    thresh_output.copyTo(anchor_mask, anchor_mask);

    findContours(anchor_mask, anchor_contours, anchor_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (auto & contour : anchor_contours)
    {
        RotatedRect rec = minAreaRect(contour);
        double area = float(rec.size.width) * float(rec.size.height);
        if (area < min_corner_rec && area > 50)
        {
            square_contour.clear();
            square_contour.push_back(contour[0]);
        }
    }

    if(!square_contour.empty()){
        cv::circle(img, square_contour[0], 3, cv::Scalar(255, 255, 255), 3);
        for (int i = 0; i < 4; ++i){
            distance[i] = sqrt((anchor_poly[i].x - square_contour[0].x) * (anchor_poly[i].x - square_contour[0].x) + (anchor_poly[i].y - square_contour[0].y) * (anchor_poly[i].y - square_contour[0].y));
        }
        double len_min = distance[0];
        for (int i = 1; i < 4; ++i){
            if (distance[i] < len_min){
                len_min = distance[i];
                min_index = i;
            }
        }
        for (int i = 0; i < 4; ++i){
            anchor_temp.push_back(anchor_poly[(min_index + i) % 4]);
        }
        temp_anchor_point.push_back(anchor_temp);
    }

    RotatedRect res_rect = minAreaRect(anchor_poly);
    double res_area = res_rect.size.width * res_rect.size.height;
    double poly_area = contourArea(anchor_poly);

    polylines(img, anchor_poly, true, Scalar(255, 0, 0), 2, 8, 0);
    putText(img, "match_rate:"+to_string(poly_area / res_area), anchor_poly[min_index] , FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);

    res_rect.points(vertices);
    for (int i = 0; i < 4; ++i){
        line(img, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
    }
    writeImageRaw(index, img);

    if(poly_area / res_area > debug_ui.match_rate){
        logger.warn("match rate:{}", poly_area / res_area);
        anchor_point = temp_anchor_point;
        debug_ui.match_rate = poly_area / res_area;
        debug_ui.poly = anchor_poly;
        debug_ui.area = res_area;
        debug_ui.min_index = min_index;
        debug_ui.small_square_point.clear();
        for (auto & contour : anchor_contours)
        {
            RotatedRect rec = minAreaRect(contour);
            double area = float(rec.size.width) * float(rec.size.height);
            if (area < min_corner_rec && area > 50)
            {
                debug_ui.small_square_area.push_back(area);
                debug_ui.small_square_point.push_back(contour[0]);
            }
        }
        if(debug_ui.match_rate < 0.65){
            anchor_point.clear();
            debug_ui.right_flag = false;
        }else{
            debug_ui.right_flag = true;
        }
    }
}

void Sitedetector::draw_debug_ui(Mat &img, DebugUI &debug_ui){
    for(int i = 0; i < debug_ui.small_square_point.size(); ++i){
        putText(img, "area:"+to_string(debug_ui.small_square_area[i]), debug_ui.small_square_point[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
        cv::circle(img, debug_ui.small_square_point[i], 5, cv::Scalar(0, 255, 255), 5);
    }
    if(!debug_ui.poly.empty()){
        cv::circle(img, debug_ui.poly[debug_ui.min_index], 5, cv::Scalar(255, 0, 255), 5);
        polylines(img, debug_ui.poly, true, Scalar(0, 255, 0), 2, 8, 0);
        putText(img, "poly_area:"+to_string(contourArea(debug_ui.poly)), Point(0, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    }
    putText(img, "area:"+to_string(debug_ui.area), Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    putText(img, "rate:"+to_string(debug_ui.match_rate), Point(0, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    if(debug_ui.right_flag){
        putText(img, "Right!", Point(0, 250), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 2, 5);
    }else{
        putText(img, "Wrong!", Point(0, 250), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2, 5);
    }
}