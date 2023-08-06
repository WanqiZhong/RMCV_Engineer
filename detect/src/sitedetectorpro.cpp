#include "sitedetectorpro.hpp"

void SitedetectorPro::Detector_Run(Mat &img) {
    Mat ori_img = img.clone();
    all_contours.clear();
    station_contours.clear();
    square_contour.clear();
    anchor_contour.clear();
    valid_contour.clear();
    corner_cnt = 0;
    
    find_four_corner(img);
    // find_anchor(img);
    if (corner_cnt == 0){
        logger.warn("ExchangeSite_Run can't find normal corner");
    }else{
        find_anchor(ori_img);
    }
    // if(!anchor_point.empty()){
    //     reverse(anchor_point[0].begin(), anchor_point[0].end());
    // }
    // writeVideoRaw(img);
    imshow("[EXCHANGE_SITE]", ori_img);
    waitKey(1);
}


/// @brief find the L-shaped corner and the square corner of the exchange changesite
/// @param img, the original picture
/// @return void
void SitedetectorPro::find_four_corner(Mat &img)
{
    vector<vector<Point>> corner_contours;
    vector<Vec4i> corner_hierarchy;
    double min_corner_area = 500000; // 记录当前最小的面积
    min_corner_index = 0; 
    Mat morphologyEx_thresh;

    cvtColor(img, thresh_output, COLOR_BGR2HSV);

    if (param.camp == 0)
        inRange(thresh_output, Scalar(0, 0, 100), Scalar(180, 255, 255), thresh_output); // red
    else{
        inRange(thresh_output, Scalar(0, 0, 100), Scalar(180, 255, 255), thresh_output); // red
    }

    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    dilate(thresh_output, morphologyEx_thresh, kernel);
    // kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    // erode(morphologyEx_thresh, morphologyEx_thresh, kernel);

    imshow("[FIND_CORNER_AFT]", morphologyEx_thresh);
    findContours(morphologyEx_thresh, corner_contours, corner_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < corner_contours.size(); i++)
    {
        RotatedRect rec = minAreaRect(corner_contours[i]);
        double rate = float(rec.size.width) / rec.size.height;
        double area = float(rec.size.width) * rec.size.height;
        drawContours(img, corner_contours, i, Scalar(0, 255, 0), 2, 8, corner_hierarchy, 0, Point());
        
        double G_avg = GetMeanValueInsideContour(img, corner_contours[i], 1);
        putText(img, "G_avg:" + to_string(G_avg), Point(corner_contours[i][0].x, corner_contours[i][0].y - 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, 8);
        putText(img, "w/h_rate:" + to_string(rate), corner_contours[i][0], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, 8);
        putText(img, "area:" + to_string(area), Point(corner_contours[i][0].x, corner_contours[i][0].y + 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, 8);
        putText(img, "ratio:" + to_string(contourArea(corner_contours[i]) / area ), Point(corner_contours[i][0].x, corner_contours[i][0].y + 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2, 8);

        if(G_avg >= param.G_avg_max) continue;
        putText(img, "G_avg:" + to_string(G_avg), Point(corner_contours[i][0].x, corner_contours[i][0].y - 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 8);

        if (rate < param.site_min_rate || rate > param.site_max_rate) continue;
        putText(img, "w/h_rate:" + to_string(rate), corner_contours[i][0], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 8);

        if(area < param.site_min_area || area > param.site_max_area) continue;
        putText(img, "area:" + to_string(area), Point(corner_contours[i][0].x, corner_contours[i][0].y + 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 8);

        if(contourArea(corner_contours[i]) / area > param.site_area_rate) continue;
        putText(img, "ratio:" + to_string(contourArea(corner_contours[i]) / area ), Point(corner_contours[i][0].x, corner_contours[i][0].y + 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 8);
        

        for (const auto & j : corner_contours[i]){
            all_contours.push_back(j);
        }
        valid_contour.push_back(corner_contours[i]);

        cv::Moments moments = cv::moments(corner_contours[i]);
        double cX = moments.m10 / moments.m00;
        double cY = moments.m01 / moments.m00;
        anchor_contour.push_back(Point(cX, cY));

            // if (contourArea(corner_contours[i]) < min_corner_area)
            // {
            //     min_corner_area = contourArea(corner_contours[i]);
            //     min_corner_index = i;
            // }
        corner_cnt++;
    }
    imshow("[LOW_DEBUG]", img);

    // if (!corner_contours.empty()){
    //     // 得到最小面积角点（标志角点）的面积
    //     // 使用理论上正方形小角点的外接矩形面积应当小于最小角点（标志角点）的面积
    //     min_corner_rec = contourArea(corner_contours[min_corner_index]);
    //     // RotatedRect rec = minAreaRect(corner_contours[min_corner_index]);
    //     square_contour.push_back(corner_contours[min_corner_index][0]);
    //     putText(img, "square:" + to_string(corner_contours[min_corner_index][0].x) + "," + to_string(corner_contours[min_corner_index][0].y), Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    //     putText(img, "min_aera:"+to_string(min_corner_rec), Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    // }

}




/// @brief get coordinates of exchange station
/// @param img, the original picture
/// @param corner_contour, points of corners' contours;
/// @param square_corner_contour, points of square corner's contour;
void SitedetectorPro::find_anchor(Mat &img)
{
    DebugUI debug_ui;
    debug_ui.right_flag = false;
    debug_ui.area = 0;
    debug_ui.match_rate = 0;
    debug_ui.min_area = 100000;
    debug_ui.min_index = 0;
    debug_ui.min_area_point = Point(-1,-1);
    debug_ui.small_square_point.clear();
    debug_ui.small_square_area.clear();
    debug_ui.poly.clear();

    logger.info("corner_cnt:{}", corner_cnt);

    if(corner_cnt < 4){
        logger.warn("Wrong number of anchor_poly:{}", corner_cnt);
        if(!anchor_contour.empty())
            anchor_point.push_back(anchor_contour);
        return;
    }else if(corner_cnt >= 4){
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
                        find_best_match(canvas, temp_station_contours, temp, debug_ui, valid_index++);
                    }
                }
            }
        }
    }
    if(debug_ui.right_flag)
        get_anchor(img, debug_ui, 0);
    draw_debug_ui(img, debug_ui);
}


void SitedetectorPro::find_best_match(Mat &img, const vector<Point>& four_station_contours, const vector<vector<Point>>& four_station_contour, DebugUI &debug_ui, int index){
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
    double min_corner_area = 100000;
    bool min_corner_flag = false;

    convexHull(Mat(four_station_contours), anchor_hull, false);
    approxPolyDP(anchor_hull, anchor_poly, 25, true);
    

    anchor_mask = Mat::zeros(thresh_output.size(), CV_8UC1);
    fillPoly(anchor_mask, anchor_poly, Scalar(255, 255, 255));
    thresh_output.copyTo(anchor_mask, anchor_mask);

    if(anchor_poly.size() != 4){
        logger.warn("anchor_poly.size():{}", anchor_poly.size());
        return;
    }

    findContours(anchor_mask, anchor_contours, anchor_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < anchor_contours.size(); i++)
    {
        RotatedRect rec = minAreaRect(anchor_contours[i]);
        double rate = float(rec.size.width) / rec.size.height;
        double area = float(rec.size.width) * rec.size.height;
        
        if (rate < param.site_min_rate || rate > param.site_max_rate) continue;
        if(area < param.site_min_area || area > param.site_max_area) continue;
        if(contourArea(anchor_contours[i]) / area > param.site_area_rate) continue;
        
        if (contourArea(anchor_contours[i]) < min_corner_area)
        {
            min_corner_area = contourArea(anchor_contours[i]);
            min_corner_index = i;
        }
    }

    for (int i = 0; i < anchor_contours.size(); i++)
    {
        if (contourArea(anchor_contours[i]) < min_corner_area)
        {
            min_corner_flag = true;
            break;
        }
    }

    if(!min_corner_flag){
        logger.warn("min_corner_flag:{}", min_corner_flag);
        return;
    }
    
    RotatedRect res_rect = minAreaRect(anchor_poly);
    double res_area = res_rect.size.width * res_rect.size.height;
    double poly_area = contourArea(anchor_poly);

    // res_rect.points(vertices);
    // for (int i = 0; i < 4; ++i){
    //     line(img, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
    // }
    // writeImageRaw(index, img);

    if(poly_area / res_area > debug_ui.match_rate){
        // logger.warn("match rate:{}", poly_area / res_area);
        debug_ui.match_rate = poly_area / res_area;
        debug_ui.poly = anchor_poly;
        debug_ui.area = res_area;
        for (auto & contour : four_station_contour)
        {
            double area = contourArea(contour);
            if(area < debug_ui.min_area){
                debug_ui.min_area = area;
            }   
        }
        if(debug_ui.match_rate < 0.65){
            debug_ui.right_flag = false;
        }else{
            debug_ui.right_flag = true;
        }
    }
}




void SitedetectorPro::get_anchor(Mat &img, DebugUI &debug_ui, int index){

    vector<vector<Point>> temp_anchor_point;
    vector<double> distance = vector<double>(4, 0);
    vector<Point> anchor_temp;
    vector<Point> anchor_hull;
    vector<Point> anchor_poly;
    vector<vector<Point>> anchor_contours;
    vector<Point> anchor_contour;
    vector<Vec4i> anchor_hierarchy;
    int min_index = 0;
    cv::Point2f vertices[4];
    Mat anchor_mask;

    square_contour.clear();
    anchor_mask = Mat::zeros(thresh_output.size(), CV_8UC1);
    fillPoly(anchor_mask, debug_ui.poly, Scalar(255, 255, 255));
    thresh_output.copyTo(anchor_mask, anchor_mask);

    imshow("[LAST]",anchor_mask);
    waitKey(1);

    if(anchor_poly.size() != 4){
        logger.warn("anchor_poly.size():{}", anchor_poly.size());
        return;
    }

    findContours(anchor_mask, anchor_contours, anchor_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (auto & contour : anchor_contours)
    {
        for(auto & p : contour){
            anchor_contour.push_back(p);
        }
        RotatedRect rec = minAreaRect(contour);
        double area = float(rec.size.width) * float(rec.size.height);
        // logger.info("area:{}", area);
        if (area < debug_ui.min_area && area > 50)
        {
            cv::Moments moments = cv::moments(contour);
            double cX = moments.m10 / moments.m00;
            double cY = moments.m01 / moments.m00;
            square_contour.push_back(Point(cX, cY));
            debug_ui.small_square_area.push_back(area);
            debug_ui.small_square_point.push_back(Point(cX, cY));
        }
    }

    convexHull(Mat(anchor_contour), anchor_hull, false);
    approxPolyDP(anchor_hull, anchor_poly, 25, true);
    polylines(img, anchor_poly, true, Scalar(255, 255, 255), 2, 8, 0);

    if(!square_contour.empty()){
        cv::circle(img, square_contour[0], 3, cv::Scalar(255, 255, 255), 3);
        Point square_point;
        double square_x = 0;
        double square_y = 0;
        for(auto & t : square_contour) {
            square_x += t.x;
            square_y += t.y;
        }
        square_point.x = square_x / square_contour.size();
        square_point.y = square_y / square_contour.size();
        debug_ui.min_area_point = square_point;
        
        for (int i = 0; i < 4; ++i){
            distance[i] = sqrt(pow(square_point.x - anchor_poly[i].x, 2) + pow(square_point.y - anchor_poly[i].y, 2));
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
        
        anchor_point.push_back(anchor_temp);
        debug_ui.poly = anchor_poly;
        debug_ui.min_index = min_index;
        
    }
    
}

void SitedetectorPro::draw_debug_ui(Mat &img, DebugUI &debug_ui){
    for(int i = 0; i < debug_ui.small_square_point.size(); ++i){
        putText(img, "area:"+to_string(debug_ui.small_square_area[i]), debug_ui.small_square_point[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 5);
        cv::circle(img, debug_ui.small_square_point[i], 5, cv::Scalar(0, 255, 255), 5);
    }
    if(!debug_ui.poly.empty()){
        cv::circle(img, debug_ui.poly[debug_ui.min_index], 5, cv::Scalar(255, 0, 255), 5);
        polylines(img, debug_ui.poly, true, Scalar(0, 255, 0), 2, 8, 0);
        putText(img, "poly_area:"+to_string(contourArea(debug_ui.poly)), Point(0, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    }
    putText(img, "area:"+to_string(debug_ui.min_area), Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    putText(img, "rate:"+to_string(debug_ui.match_rate), Point(0, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    if(debug_ui.right_flag){
        putText(img, "Right!", Point(0, 250), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 2, 5);
    }else{
        putText(img, "Wrong!", Point(0, 250), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2, 5);
    }
}