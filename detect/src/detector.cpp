#include "detector.hpp"


void Detector::Run()
{
    logger.info("Detector Run");
    initVideoRaw();
    Detector_thread = thread(&Detector::Detect_Run,this);
}

void Detector::Join() {
    logger.info("Waiting for [Detector]");
    Detector_thread.join();
    logger.info("[Detector] joined");
}

void Detector::Detect_Run()
{
    cv::Mat img;
    umt::Subscriber<cv::Mat> sub("channel0");
    umt::Publisher<MINE_POSITION_MSG> mine_sub("anchor_point_data");
    while(param.get_run_mode()!=HALT)
    {
        try{
            img = sub.pop();
            if(!img.empty()){
                // imshow("pop_img",img);
                // waitKey(1);
            }
            else{
                logger.warn("Sub pop empty img");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(int(1000. /30)));
        }
        catch(const HaltEvent&){
            break;
        }
        if(!img.empty())
        {
            if(param.get_run_mode()==GoldMode){
                GoldMineDetect_Run2(img);
                if(anchor_point.empty()){
                     logger.info("No anchor point");
                }
                else{
                    mine_sub.push(MINE_POSITION_MSG{.goal=anchor_point});
                    logger.info("GoldMineDetect_Run --> Control");
                }
            }
            else if(param.get_run_mode()==SilverMode){
                SilverMineDetect_Run(img);
                // mine_sub.push(MINE_POSITION_MSG(silver_mine_rect));
                // logger.info("SilverMineDetect_Run");
            }
            else if(param.get_run_mode()==ExchangeSiteMode){
                ExchangeSite_Run(img);
                if(!anchor_point.empty()){
                    std::reverse(anchor_point[0].begin(), anchor_point[0].end());
                }
                mine_sub.push(MINE_POSITION_MSG{.goal=anchor_point});
                // umt::Publisher<ANGLE_DATA_MSG> angle_pub("robot_data");
                //             ANGLE_DATA_MSG angle_msg;
                // angle_msg.is_valid = true;
                // angle_msg.ratation_right = true;
                // // change to float
                // angle_msg.roll = param.cali_roll;
                // angle_msg.pitch = param.cali_pitch;
                // angle_msg.yaw = param.cali_yaw;
                // angle_msg.x = param.cali_x;
                // angle_msg.y = param.cali_y;
                // angle_msg.z = param.cali_z;
                // logger.warn("angle_msg: x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",angle_msg.x,angle_msg.y,angle_msg.z,\
                // angle_msg.roll,angle_msg.pitch,angle_msg.yaw);
                // angle_pub.push(angle_msg);
            }
            else{
                 logger.critical("Detector mode error");
            }
            if(param.detector_need){
                writeVideoRaw(img);
                imshow("res_pic", img);
                // waitKey(1);
                if (waitKey(1) == 's'){
                    writeImageRaw(shot_index++, img);
                }

            }
        }
        
    }
}

void Detector::img_light_enhance(Mat &img, Mat &img_hsv){
    if(!img.empty()){
        int hmin = 0;
        int hmax = 50;
        int smin = 87;
        int smax = 255;
        int vmin = 175;
        int vmax = 255;

        Mat hsv;
        Mat color_mask;

        int rows = img.rows;
        int cols = img.cols;
        int max_val = 255;


        cvtColor(img, img, COLOR_BGR2HSV);  // 将BGR颜色空间转换为HSV颜色空间
        // 三通道拆分
        vector<Mat> hsv_channels(3);
        split(img, hsv_channels);
        // 对V通道进行自适应直方图均衡化

        // Ptr<CLAHE> clahe = createCLAHE(2.0,tileGridSize=(8,8)).apply(hsv_channels[2],hsv_channels[2]);
        createCLAHE(2.0,Size(8,8))->apply(hsv_channels[2],hsv_channels[2]);
        // 三通道合并
        merge(hsv_channels, img);
        cvtColor(img, img, COLOR_HSV2BGR);  // 将HSV颜色空间转换为BGR颜色空间
        imshow("enhance_before", img);

        cvtColor(img, img, COLOR_BGR2HSV);  // 将BGR颜色空间转换为HSV颜色空间
        inRange(img,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),color_mask);

        int v_sum = 0;
        int count = 1;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (color_mask.at<uchar>(i, j) != 0) {  // 判断像素是否在区域内
                    int v = img.at<Vec3b>(i, j)[2];
                    v_sum += v;
                    count++;
                }
            }
        }
        
        int avg_v = v_sum / count;  // 计算区域内平均亮度值
        v_sum = v_sum / 10000;
        cout<<"avg_v:"<<avg_v<<" "<<v_sum<<endl;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                int h = img.at<Vec3b>(i, j)[0];
                int s = img.at<Vec3b>(i, j)[1];
                int v = img.at<Vec3b>(i, j)[2];
                // if(v_sum < 500){
                //     v *= 1.5;
                //     v += 50;
                // }
                if(v_sum < 1500){
                    v *= 1.4;  
                    v += 30;                
                 }
                else if(v_sum < 3000){
                    v *= 1.3;
                    v += 30;
                }
                else{
                    v *= 1.2;
                    v += 20;
                }
                v = std::min(v, max_val);  // 取max(255,提亮后)
                img.at<Vec3b>(i, j) = Vec3b(h, s, v);
            }
        }

        inRange(img,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),color_mask);
        v_sum = 0;
        count = 1;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (color_mask.at<uchar>(i, j) != 0) {  // 判断像素是否在区域内
                    int v = img.at<Vec3b>(i, j)[2];
                    v_sum += v;
                    count++;
                }
            }
        }
        avg_v = v_sum / count;  // 计算区域内平均亮度值
        cout<<"aft_avg_v:"<<avg_v<<" "<<v_sum/10000<<endl;

        cvtColor(img, img, COLOR_HSV2BGR);  // 将HSV颜色空间转换为BGR颜色空间
        imshow("enhance_after", img);


        cvtColor(img, img_hsv, COLOR_BGR2HSV);
        // namedWindow("dst", WINDOW_NORMAL);
        inRange(img_hsv,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),img_hsv);

	    Mat kernel = getStructuringElement(MORPH_RECT,Size(2,2)); 
        erode(img_hsv, img_hsv, kernel);
        kernel = getStructuringElement(MORPH_ELLIPSE,Size(2,2));
        morphologyEx(img_hsv, img_hsv,MORPH_CLOSE, kernel); 
        kernel = getStructuringElement(MORPH_ELLIPSE,Size(3,3));
        morphologyEx(img_hsv, img_hsv,MORPH_OPEN, kernel); 
        medianBlur(img_hsv,img_hsv,1);
        
    }
}


void Detector::GoldMineDetect_Run(Mat &img)
{
    find_gold_mine(img);
    get_gold_mine(img);
    imshow("mine",img);
}

void Detector::GoldMineDetect_Run2(Mat &img)
{
    anchor_point.clear();
    gold_mine_contours_2.clear();
    RotatedRect gold_mine_rec;
    // polycontours.clear();
    // find_gold_mine_2(img);

    // if(polycontours.empty())
        // return;

    Mat enhance_img = img.clone();
    Mat img_hsv = img.clone();
    if(!img.empty()){

        img_light_enhance(enhance_img, img_hsv);
        

        Mat output = get_gold_mine_2(enhance_img, img_hsv);
    
        Mat process = process_img_corner(output, 155, 255);

        vector<vector<Point>> logo_R;
        int side_number = find_R(logo_R, process);

        vector<vector<Point>> side;
        vector<Point> square;
        cout<<"Store side"<<endl;
        side = store_side(logo_R, square, corner_number);
        cout<<"Store side"<<endl;
        
        for (int i = 0; i < side_number; i++)
        {
            if (side.size() == 0)
            {
                cout << "cannot find side" << endl;
            }
            else if (square.size() == 0)
            {
                cout << "cannot find square" << endl;
            }
            else
            {
                draw_side(img, side[i], square[i], corner_number[i]);
            }
        }
            
    }

}


void Detector::SilverMineDetect_Run(Mat &img)
{
    find_white_mineral(img,silver_mine_rect);
    enhance_img(img);
    // vector<Mat> imgs;
    for(int i = 0;i<silver_mine_rect.size();i++)
    {
        Mat output = get_white_mineral(img,silver_mine_rect[i]);
        process_white_corner(output, 95, 255,silver_mine_contours);
        get_white_corner(img,silver_mine_contours);
        //imgs.push_back(output);
    }
}



void Detector::ExchangeSite_Run(Mat &img) {
    all_contours.clear();
    station_contours.clear();
    square_contour.clear();
    anchor_point.clear();
    anchor_contour.clear();
    valid_contour.clear();
    find_site_corner(img);
    if (all_contours.size() == 0)
    {
        logger.warn("ExchangeSite_Run can't find normal corner");
    }
    else if (sizeof(square_contour) == 0)
    {
        logger.warn("ExchangeSite_Run can't find square corner");
    }
    else
    {
        get_station_side(img);
    }
}







/// @brief process the RGB image to get the contours(include inRange, morphologyEx, GaussianBlur, Canny, findContours)
/// @param img current RGB frame image
/// @param lower  inRange lower bound
/// @param upper  inRange upper bound
void Detector::process_gold_mine(const Mat& img,Scalar lower=Scalar(gold_hmin,gold_smin,gold_vmin),Scalar upper=Scalar(gold_hmax,gold_smax,gold_vmax))
{
    Mat img_hsv;
    vector<Mat> img_split ;
    cvtColor(img,img_hsv,COLOR_BGR2HSV);
    split(img_hsv,img_split);
    equalizeHist(img_split[2],img_split[2]);
    merge(img_split,img_hsv);

    GaussianBlur(img_hsv,img_hsv,Size(3,3),2,2);
    inRange(img_hsv,lower,upper,img_hsv); 


    Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5)); 
    erode(img_hsv, img_hsv, kernel);
    medianBlur(img_hsv,img_hsv,1);
    
    Mat img_Canny;
    imshow("aft",img_hsv);
    Canny(img_hsv,img_Canny,20,120);

    findContours(img_Canny,gold_mine_contours,gold_mine_hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
}

/// @brief process the RGB->gray image to get the contours(GaussianBlur, Canny, findContours)
/// @param img current RGB frame image
/// @param thresh the threshold value
/// @param maxval the threshold max value
/// @param type threshold type
void Detector::process_gold_mine(const Mat& img,int thresh,int maxval=255,int type=THRESH_BINARY)
{   
    Mat img_blur;
    Mat img_gray;
    GaussianBlur(img,img_blur,Size(5,5),3,3);
    cvtColor(img_blur,img_gray,COLOR_BGR2GRAY);
    threshold(img_gray,img_gray,thresh,maxval,THRESH_BINARY);

    GaussianBlur(img_gray,img_gray,Size(5,5),0);
    Mat img_Canny;
    Canny(img_gray,img_Canny,150,100);
    findContours(img_Canny,gold_mine_contours,gold_mine_hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
    // findContours(img_Canny,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
}


void Detector::enhance_img(Mat &img)
{
    double a, b;
    double delta=0;
    delta = 127. * contrast  / 100;
    a = 255. / (255. - delta * 2);
    b = a * (bright - delta);
    img.convertTo(img, CV_8U, a, b);
}

/* ====================================================================================== */
/* ====================================== 金矿识别方案1 =================================== */
/* ====================================================================================== */

/// @brief find the anchor point of mine
/// @param img current frame image
void Detector::find_gold_mine(Mat& img)
{
    enhance_img(img);
    process_gold_mine(img);
    double whole_area = 0;
    for(int i = 0; i < gold_mine_contours.size(); ++i){
        int k = i;
        if(gold_mine_hierarchy[i][2]!=-1) {
            k = gold_mine_hierarchy[i][2];
        }   
        Rect rect_res = boundingRect(gold_mine_contours[k]);
        double area = contourArea(gold_mine_contours[k]);
        double rect_area = rect_res.width * rect_res.height;
        double rate = area / rect_area;
        Point p(rect_res.x + rect_res.width / 2+25, rect_res.y + rect_res.height / 2+10);
        if(rate>=0.65){  
            if(rect_area>400&&(double)rect_res.width/(double)rect_res.height>0.3&&(double)rect_res.height/(double)rect_res.width>0.3)
            {
                gold_mine_whole_rect.push_back(rect_res);
                whole_area += rect_area;
                #ifndef LABEL
                string s = to_string(rect_res.x) + "," + to_string(rect_res.y); 
                string s1 = to_string(rect_res.width) + "," + to_string(rect_res.height);
                // putText(img, s, p, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 128, 255), 2, 8, 0);
                // putText(img, s1, p+Point(0,20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2, 8, 0);
                #endif
            }
        } 
        else if (rate>=0.35){
            if(rect_area>400&&(double)rect_res.width/(double)rect_res.height>0.2&&(double)rect_res.height/(double)rect_res.width>0.2)
            {
                gold_mine_half_rect.push_back(rect_res);
                #ifndef LABEL
                // drawContours(img,gold_mine_contours,k,Scalar(255, 128, 255),2);          
                rectangle(img,rect_res,Scalar(255,128,255),2);
                string s = to_string(rect_res.x) + "," + to_string(rect_res.y); 
                string s1 = to_string(rect_res.width) + "," + to_string(rect_res.height);
                // putText(img, s, p, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 128, 255), 2, 8, 0);
                // putText(img, s1, p+Point(0,20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2, 8, 0);
                #endif
            }
        }
    }
    double single_area = whole_area/gold_mine_whole_rect.size();
    for(int i = 0; i < gold_mine_whole_rect.size(); ++i){
        if(gold_mine_whole_rect[i].area() > single_area*1.5){
            gold_mine_whole_rect.erase(gold_mine_whole_rect.begin()+i);
            whole_area -= single_area;
        }
        else{  
            #ifndef LABEL
            // rectangle(img,gold_mine_whole_rect[i],Scalar(0,128,255),2); 
            #endif
        }
    }
}

void Detector::get_gold_mine(Mat &img)
{
    Point center;
    int lower_bound_y[3] = {0};
    int upper_bound_y[3] = {0};
    int lower_bound_x[3] = {0};      
    int upper_bound_x[3] = {0};
    int lower_bound_wid = 0;
    int upper_bound_wid = 0;
    int lower_bound_hei = 0;
    int upper_bound_hei = 0;
    int bound_wid = 0;
    int bound_hei = 0;
    vector<Rect> rect_res;
    cout<<"gold_mine_whole_rect.size():"<<gold_mine_whole_rect.size()<<endl;
    anchor_point.clear();
    for(int i = 0; i < gold_mine_whole_rect.size(); ++i){
        vector<Point> anchor_point_single;
        vector<vector<int>> decision_tree = vector<vector<int>>(4,vector<int>(3,0));
        vector<vector<Rect>> rect_tree = vector<vector<Rect>>(4,vector<Rect>(3));
        lower_bound_wid = gold_mine_whole_rect[i].width*0.4;
        upper_bound_wid = gold_mine_whole_rect[i].width*1.6;
        lower_bound_hei = gold_mine_whole_rect[i].height*0.6;
        upper_bound_hei = gold_mine_whole_rect[i].height*1.4;
        bound_wid = gold_mine_whole_rect[i].width*1.3;
        bound_hei = gold_mine_whole_rect[i].height*1.3;
        for(int k=0;k<3;k++){
            if(k==0)
            {
                lower_bound_y[k] = gold_mine_whole_rect[i].y - 5 * gold_mine_whole_rect[i].height;
                upper_bound_y[k] = gold_mine_whole_rect[i].y - 2 * gold_mine_whole_rect[i].height;    
                lower_bound_x[k] = gold_mine_whole_rect[i].x - 5 * gold_mine_whole_rect[i].width;
                upper_bound_x[k] = gold_mine_whole_rect[i].x - 2 * gold_mine_whole_rect[i].width;
            }
            else if(k==1)
            {
                lower_bound_y[k] = gold_mine_whole_rect[i].y + 2.5 * gold_mine_whole_rect[i].height;
                upper_bound_y[k] = gold_mine_whole_rect[i].y + 5 * gold_mine_whole_rect[i].height;    
                lower_bound_x[k] = gold_mine_whole_rect[i].x + 2.5 * gold_mine_whole_rect[i].width;
                upper_bound_x[k] = gold_mine_whole_rect[i].x + 5 * gold_mine_whole_rect[i].width;
            }
            else if(k==2)
            {
                lower_bound_y[k] = gold_mine_whole_rect[i].y - 1.5 * gold_mine_whole_rect[i].height;
                upper_bound_y[k] = gold_mine_whole_rect[i].y + 1.5 * gold_mine_whole_rect[i].height;    
                lower_bound_x[k] = gold_mine_whole_rect[i].x - 1.5 * gold_mine_whole_rect[i].width;
                upper_bound_x[k] = gold_mine_whole_rect[i].x + 1.5 * gold_mine_whole_rect[i].width;
            }
        }
        // cout<<"lower_bound_y:"<<lower_bound_y[0]<<","<<lower_bound_y[1]<<","<<lower_bound_y[2]<<endl;
        // cout<<"upper_bound_y:"<<upper_bound_y[0]<<","<<upper_bound_y[1]<<","<<upper_bound_y[2]<<endl;
        // cout<<"lower_bound_x:"<<lower_bound_x[0]<<","<<lower_bound_x[1]<<","<<lower_bound_x[2]<<endl;
        // cout<<"upper_bound_x:"<<upper_bound_x[0]<<","<<upper_bound_x[1]<<","<<upper_bound_x[2]<<endl;
        // cout<<"gold_mine_whole_rect[i].x:"<<gold_mine_whole_rect[i].x<<" gold_mine_whole_rect[i].y:"<<gold_mine_whole_rect[i].y<<endl;
        // cout<<"gold_mine_whole_rect[i].width:"<<gold_mine_whole_rect[i].width<<" gold_mine_whole_rect[i].height:"<<gold_mine_whole_rect[i].height<<endl;
        for(int j = 0; j < gold_mine_half_rect.size(); ++j){
            if(gold_mine_half_rect[j].width>lower_bound_wid&&gold_mine_half_rect[j].width<upper_bound_wid&&gold_mine_half_rect[j].height>lower_bound_hei&&gold_mine_half_rect[j].height<upper_bound_hei)
            {
                if(gold_mine_half_rect[j].x>lower_bound_x[1]&&gold_mine_half_rect[j].x<upper_bound_x[1]&&gold_mine_half_rect[j].y>lower_bound_y[0]&&gold_mine_half_rect[j].y<upper_bound_y[0])
                {
                    decision_tree[0][1] = 1;
                    rect_tree[0][1] = gold_mine_half_rect[j];
                }
                else if(gold_mine_half_rect[j].x>lower_bound_x[0]&&gold_mine_half_rect[j].x<upper_bound_x[0]&&gold_mine_half_rect[j].y>lower_bound_y[0]&&gold_mine_half_rect[j].y<upper_bound_y[0])
                {
                    decision_tree[1][1] = 1;
                    rect_tree[1][1] = gold_mine_half_rect[j];
                }
                else if(gold_mine_half_rect[j].x>lower_bound_x[0]&&gold_mine_half_rect[j].x<upper_bound_x[0]&&gold_mine_half_rect[j].y>lower_bound_y[1]&&gold_mine_half_rect[j].y<upper_bound_y[1])
                {
                    decision_tree[2][1] = 1;
                    rect_tree[2][1] = gold_mine_half_rect[j];
                }
                else if(gold_mine_half_rect[j].x>lower_bound_x[1]&&gold_mine_half_rect[j].x<upper_bound_x[1]&&gold_mine_half_rect[j].y>lower_bound_y[1]&&gold_mine_half_rect[j].y<upper_bound_y[1])
                {
                    decision_tree[3][1] = 1;
                    rect_tree[3][1] = gold_mine_half_rect[j];
                }
                else if(abs(gold_mine_half_rect[j].x-gold_mine_whole_rect[i].x)<bound_wid && gold_mine_half_rect[j].y>lower_bound_y[0]&&gold_mine_half_rect[j].y<upper_bound_y[0])
                {
                    decision_tree[0][0] = 1;
                    decision_tree[1][0] = 1;
                    rect_tree[0][0] = gold_mine_half_rect[j];
                    rect_tree[1][0] = gold_mine_half_rect[j];
                }
                else if(abs(gold_mine_half_rect[j].x-gold_mine_whole_rect[i].x)<bound_wid && gold_mine_half_rect[j].y>lower_bound_y[1]&&gold_mine_half_rect[j].y<upper_bound_y[1])
                {
                    decision_tree[2][0] = 1;
                    decision_tree[3][0] = 1;
                    rect_tree[2][0] = gold_mine_half_rect[j];
                    rect_tree[3][0] = gold_mine_half_rect[j];
                }
                else if(abs(gold_mine_half_rect[j].y-gold_mine_whole_rect[i].y)<bound_hei && gold_mine_half_rect[j].x>lower_bound_x[0]&&gold_mine_half_rect[j].x<upper_bound_x[0])
                {
                    decision_tree[1][2] = 1;
                    decision_tree[2][2] = 1;
                    rect_tree[1][2] = gold_mine_half_rect[j];
                    rect_tree[2][2] = gold_mine_half_rect[j];
                    // cout<<"[2] gold_mine_whole_rect[i].x:"<<gold_mine_whole_rect[i].x;
                    // cout<<" gold_mine_whole_rect[i].y:"<<gold_mine_whole_rect[i].y<<endl;
                }
                else if(abs(gold_mine_half_rect[j].y-gold_mine_whole_rect[i].y)<bound_hei && gold_mine_half_rect[j].x>lower_bound_x[1]&&gold_mine_half_rect[j].x<upper_bound_x[1])
                {
                    decision_tree[0][2] = 1;
                    decision_tree[3][2] = 1;
                    rect_tree[0][2] = gold_mine_half_rect[j];
                    rect_tree[3][2] = gold_mine_half_rect[j];
                }
            }
        }
        int sum[4] = {0};
        int max = 0;
        int max_k = 0;
        vector<string> res={"<-left","bottom","->right","^ top"};
        for(int k=0;k<4;++k){
            sum[k] = decision_tree[k][0]+decision_tree[k][1]+decision_tree[k][2];
            if(sum[k]==3)
            {
                // cout<<"sum["<<k<<"]:"<<sum[k]<<endl;
                max = sum[k];
                max_k = k;
                rect_res.push_back(gold_mine_whole_rect[i]);
                if(k==0)
                {
                    center = Point((gold_mine_whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2,(gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height+rect_tree[k][1].y)/2);
                    anchor_point_single.push_back(Point(rect_tree[k][0].x,rect_tree[k][0].y));
                    anchor_point_single.push_back(Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y));
                    anchor_point_single.push_back(Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y+rect_tree[k][2].height));
                    anchor_point_single.push_back(Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height));
                    anchor_point.push_back(anchor_point_single);
                    line(img, Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height), Point(rect_tree[k][0].x,rect_tree[k][0].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x,rect_tree[k][0].y), Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y), Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y+rect_tree[k][2].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y+rect_tree[k][2].height), Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((gold_mine_whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
                else if(k==1)
                {
                    anchor_point_single.push_back(Point(rect_tree[k][1].x,rect_tree[k][1].y));
                    anchor_point_single.push_back(Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y));
                    anchor_point_single.push_back(Point(gold_mine_whole_rect[i].x+gold_mine_whole_rect[i].width,gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height));
                    anchor_point_single.push_back(Point(rect_tree[k][2].x,rect_tree[k][2].y+rect_tree[k][2].height));
                    anchor_point.push_back(anchor_point_single);
                    line(img, Point(gold_mine_whole_rect[i].x+gold_mine_whole_rect[i].width,gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height), Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y), Point(rect_tree[k][1].x,rect_tree[k][1].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x,rect_tree[k][1].y), Point(rect_tree[k][2].x,rect_tree[k][2].y+rect_tree[k][2].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x,rect_tree[k][2].y+rect_tree[k][2].height), Point(gold_mine_whole_rect[i].x+gold_mine_whole_rect[i].width,gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((gold_mine_whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
                else if(k==2)
                {
                    anchor_point_single.push_back(Point(rect_tree[k][2].x,rect_tree[k][2].y));
                    anchor_point_single.push_back(Point(gold_mine_whole_rect[i].x+gold_mine_whole_rect[i].width,gold_mine_whole_rect[i].y));
                    anchor_point_single.push_back(Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y+rect_tree[k][0].height));
                    anchor_point_single.push_back(Point(rect_tree[k][1].x,rect_tree[k][1].y+rect_tree[k][1].height));
                    anchor_point.push_back(anchor_point_single);
                    line(img, Point(gold_mine_whole_rect[i].x+gold_mine_whole_rect[i].width,gold_mine_whole_rect[i].y), Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y+rect_tree[k][0].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y+rect_tree[k][0].height), Point(rect_tree[k][1].x,rect_tree[k][1].y+rect_tree[k][1].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x,rect_tree[k][1].y+rect_tree[k][1].height), Point(rect_tree[k][2].x,rect_tree[k][2].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x,rect_tree[k][2].y), Point(gold_mine_whole_rect[i].x+gold_mine_whole_rect[i].width,gold_mine_whole_rect[i].y), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((gold_mine_whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
                else if(k==3)
                {
                    anchor_point_single.push_back(Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y));
                    anchor_point_single.push_back(Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y));
                    anchor_point_single.push_back(Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y+rect_tree[k][1].height));
                    anchor_point_single.push_back(Point(rect_tree[k][0].x,rect_tree[k][0].y+rect_tree[k][0].height));
                    anchor_point.push_back(anchor_point_single);
                    line(img, Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y), Point(rect_tree[k][0].x,rect_tree[k][0].y+rect_tree[k][0].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x,rect_tree[k][0].y+rect_tree[k][0].height), Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y+rect_tree[k][1].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y+rect_tree[k][1].height), Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y), Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((gold_mine_whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(gold_mine_whole_rect[i].y+gold_mine_whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
            }
            else
            {
                max = sum[k] > max ? sum[k] : max;
                max_k = k;
                // cout<<max<<endl;
            }
        }
        if(max!=3&&max>=2)
        {
            rect_res.push_back(gold_mine_whole_rect[i]);
            putText(img, res[max_k], Point(gold_mine_whole_rect[i].x,gold_mine_whole_rect[i].y), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,128,255), 2, 8, 0);
        }
    }
}






/* ====================================================================================== */
/* ====================================== 金矿识别方案2 =================================== */
/* ====================================================================================== */


// void Detector::find_gold_mine_2(Mat &img)
// {
//     Mat output;
//     cvtColor(img, output, COLOR_BGR2HSV);
//     inRange(output, Scalar(0, 76, 169), Scalar(48, 255, 255), output);
//     Mat kernel_middle = getStructuringElement(MORPH_RECT, Size(5, 5));
//     Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
//     // erode(output, output, kernel_middle);
//     // dilate(output, output, kernel_middle);
//     // dilate(output, output, kernel, Point(-1, -1), 3);
//     // erode(output, output, kernel, Point(-1, -1), 5);
//     medianBlur(output, output, 3);
//     vector<vector<Point>> side_contours;
//     vector<Vec4i> side_hierarchy;
//     findContours(output, side_contours, side_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//     for (int i = 0; i < side_contours.size(); i++)
//     {
//         RotatedRect rec_temp = minAreaRect(side_contours[i]);
//         double rate = float(rec.size.width) / float(rec.size.height);
//         double area = float(rec.size.width) * float(rec.size.height);
//         if (rate >= 0.5 && rate <= 2.5 && area >= 8000 && area <= 20000)
//         {
//             // Point2f p[4];
//             // rec_temp.points(p);
//             // for (int i = 0; i < 4; i++)
//             // {
//             //     line(img, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
//             // }
//             rec = rec_temp;
//         }
//     }
//     // namedWindow("mine",WINDOW_NORMAL);
//     // imshow("mine",output);
// }


Mat Detector::get_gold_mine_2(Mat &img, Mat &colorhist)
{
    // imshow("get",img);
    vector<vector<Point>> side_contours;
    vector<Vec4i> side_hierarchy;
    vector<vector<Point>> hull;

    //正式代码中注释掉
    // cvtColor(colorhist, colorhist, COLOR_BGR2GRAY);
    

    findContours(colorhist, side_contours, side_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < side_contours.size(); i++)
    {
        RotatedRect rec_temp = minAreaRect(side_contours[i]);
        double rate = float(rec_temp.size.width) / float(rec_temp.size.height);
        double area = float(rec_temp.size.width) * float(rec_temp.size.height);
        if (rate >= 0.5 && rate <= 2.5 && area >= 80000)
        {
            // Point2f p[4];
            // rec_temp.points(p);
            // for (int i = 0; i < 4; i++)
            // {
            //     line(img, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
            // }

            vector<Point> hull_temp;
            convexHull(Mat(side_contours[i]), hull_temp, false); // 寻找金矿的凸包
            hull.push_back(hull_temp);
        }
    }

    // 抠图
    Mat mask;
    mask = Mat::zeros(img.size(), CV_8UC1); // 设置蒙版
    for (int i = 0; i < hull.size(); i++)
    {
        fillPoly(mask, hull[i], Scalar(255, 255, 255)); // 将凸包区域设置为白色
        
    }

    // mask(rec).setTo(255);
    img.copyTo(mask, mask);

    // 背景变为白色
    for (int i = 0; i < mask.rows; i++)
    {
        for (int j = 0; j < mask.cols; j++)
        {
            if (mask.at<Vec3b>(i, j)[0] == 0 && mask.at<Vec3b>(i, j)[1] == 0 && mask.at<Vec3b>(i, j)[2] == 0)
            {
                mask.at<Vec3b>(i, j)[0] = 255;
                mask.at<Vec3b>(i, j)[1] = 255;
                mask.at<Vec3b>(i, j)[2] = 255;
            }
        }
    }
    return mask;
}

/// @brief process the RGB image to get the contours of corner(include cvtColor, threshold, GaussianBlur, findContours)
/// @param img current RGB frame image
/// @param thresh the threshold of function "threshold"
/// @param maxval the max value of function "threshold"
Mat Detector::process_img_corner(const Mat &img, int thresh, int maxval)
{
    Mat dst;
    cvtColor(img, dst, COLOR_BGR2GRAY);
    // GaussianBlur(dst, dst, Size(3, 3), 2, 2);
    threshold(dst, dst, thresh, maxval, THRESH_BINARY_INV);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(dst, dst, MORPH_CLOSE, kernel, Point(-1, -1), 3);
    morphologyEx(dst, dst, MORPH_OPEN, kernel, Point(-1, -1), 5);
    dilate(dst, dst, kernel, Point(-1, -1), 4);
    erode(dst, dst, kernel, Point(-1, -1), 2);
    medianBlur(dst, dst, 1);
    vector<vector<Point>> contour_temp;
    findContours(dst, contour_temp, gold_mine_hierarchy_2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // 寻找角点轮廓
    for (int i = 0; i < contour_temp.size(); i++)
    {
        RotatedRect rec = minAreaRect(contour_temp[i]);
        double area = rec.size.height * rec.size.width;
        double rate = double(rec.size.height) / double(rec.size.width);
        // cout<<rate<<endl;
        
        // 筛选角点轮廓
        if (contourArea(contour_temp[i]) >= 1000 && contourArea(contour_temp[i]) <= 15000 && area >= 4000 && area <= 18000 && rate >= 0.3 && rate <= 3)
        {
            Point2f p[4];
            rec.points(p);
            cout<<area<<endl;
            for (int i = 0; i < 4; i++)
            {
                line(img, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
            }
            gold_mine_contours_2.push_back(contour_temp[i]);
        }
    }

    // cout<<"contours.size():"<<gold_mine_contours_2.size()<<endl;

    namedWindow("corner", WINDOW_NORMAL);
    imshow("corner", dst);

    // for (int i = 0; i < gold_mine_contours_2.size(); i++)
    // {
    //     Rect rec = boundingRect(gold_mine_contours_2[i]);
    //
    // }
    return dst;
}
/// @brief find contours of logo "R" (include findContours)
/// @param logo_R to store the contours of "R"
/// @param process the image which have been processed
/// @return int side_number
int Detector::find_R(vector<vector<Point>> &logo_R, Mat process)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(process, process, MORPH_CLOSE, kernel, Point(-1, -1), 2);

    findContours(process, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++)
    {
        double len = arcLength(contours[i], true); // 获取轮廓周长
        // cout << "contour len:" << len << endl;
        RotatedRect rec = minAreaRect(contours[i]);
        double rec_len = (rec.size.height + rec.size.width) * 2; // 获取外接旋转矩形周长
        // cout << "rec len:" << rec_len << endl;
        if (hierarchy[i][2] >= 0 && contourArea(contours[i]) >= 20000 && contourArea(contours[i]) <= 50000 && len > rec_len)
        {
            logo_R.push_back(contours[i]);

            // Point2f p[4];
            // rec.points(p);
            // for (int i = 0; i < 4; i++)
            // {
            //     line(process, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
            // }
        }
    }
    // namedWindow("corner", WINDOW_NORMAL);
    // imshow("corner", process);
    int side_number = logo_R.size();
    return side_number;
}
/// @brief find the four corners closest to "R"
/// @param p the center of "R"
/// @return vector<int> min
vector<int> Detector::sort_length(Point p)
{
    double len = 0;
    Point corner_center;
    vector<double> len_temp;
    // 找到各轮廓的中心点与R的中心点的距离并排序
    for (int i = 0; i < gold_mine_contours_2.size(); i++)
    {
        corner_center = Point(0, 0);
        for (int j = 0; j < gold_mine_contours_2[i].size(); j++)
        {
            corner_center.x += gold_mine_contours_2[i][j].x;
            corner_center.y += gold_mine_contours_2[i][j].y;
        }
        corner_center.x /= gold_mine_contours_2[i].size();
        corner_center.y /= gold_mine_contours_2[i].size();
        len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));

        len_temp.push_back(len);
    }
    for (int i = 0; i < gold_mine_contours_2.size(); i++)
    {
        for (int j = 0; j < gold_mine_contours_2.size() - i - 1; j++)
        {
            if (len_temp[j] > len_temp[j + 1])
            {
                double temp = len_temp[j];
                len_temp[j] = len_temp[j + 1];
                len_temp[j + 1] = temp;
            }
        }
    }
    vector<int> min;
    for (int j = 0; j <len_temp.size(); j++)
    {
        for (int i = 0; i < gold_mine_contours_2.size(); i++)
        {
            corner_center = Point(0, 0);
            for (int k = 0; k < gold_mine_contours_2[i].size(); k++)//计算轮廓中心点
            {
                corner_center.x += gold_mine_contours_2[i][k].x;
                corner_center.y += gold_mine_contours_2[i][k].y;
            }
            corner_center.x /= gold_mine_contours_2[i].size();
            corner_center.y /= gold_mine_contours_2[i].size();

            //计算轮廓中心点与R标中心点的距离
            len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));
            //cout<<len<<endl;

            if (len == len_temp[j] && len >= 120 && len <= 350)
            {
                min.push_back(i);
                // cout<<len<<endl;
                break;
            }
        }
       
    } 
    //cout<<"min.size()"<<min.size()<<endl;
    return min;
}
/// @brief store all contours'points in one side
/// @param logo_R the contours of R
/// @param square to store points of square corners
/// @return vector<Point> side
vector<vector<Point>> Detector::store_side(vector<vector<Point>> logo_R, vector<Point> &square, vector<int> &corner_number)
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
    // 找到角点的索引
    vector<vector<int>> len_min;
    for (int i = 0; i < logo_R.size(); i++)
    {
        len_min.push_back(sort_length(p_R[i]));
    }
    // 找到正方形角点
    for (int i = 0; i < logo_R.size(); i++)
    {
        for (int j = 0; j < len_min[i].size(); j++)

        {
            RotatedRect rec = minAreaRect(gold_mine_contours_2[len_min[i][j]]);
            double area = float(rec.size.width) * float(rec.size.height);
            double rate = contourArea(gold_mine_contours_2[len_min[i][j]]) / area;
            //cout<<"rate:"<<rate<<"   area:"<<area<<endl;

            if (rate >= 0.6 && area >= 5000 && area <= 15000)
            {
                square.push_back(gold_mine_contours_2[len_min[i][j]][0]);
                cout << square[0] << endl;
            }
        }
    }
    cout << "square.size():" << square.size() << endl;
    cout << "logo_R.size():" << logo_R.size() << endl;
    // cout<<logo_R[0][0]<<endl;

    //  将4个轮廓的点放入一个容器
    vector<vector<Point>> side;
    vector<Point> side_;
    for (int i = 0; i < logo_R.size(); i++)
    {
        vector<Point> side_temp;
        side_ = side_temp;
        if (len_min[i].size() >= 3)
        {
            for (int j = 0; j < len_min[i].size(); j++)
            {
                for (int k = 0; k < gold_mine_contours_2[len_min[i][j]].size(); k++)
                {
                    side_.push_back(gold_mine_contours_2[len_min[i][j]][k]);
                }
            }
            side.push_back(side_);
        }
        else
            cout << "cannot find enough corner" << endl;

        corner_number.push_back(len_min[i].size());
    }
    return side;
}
/// @brief draw sides
/// @param img, the original picture
/// @param side, all contours'points in each side
/// @param squre, points of square corners
void Detector::draw_side(Mat img, vector<Point> side, Point square, int corner_number)
{
    vector<Point> hull;
    vector<Point> poly;
    double len[4] = {0};
    vector<vector<Point>> anchor_point;
    anchor_point.clear();
    vector<Point> anchor_temp;

    if (corner_number == 4)
    {
        convexHull(Mat(side), hull, false);
        approxPolyDP(hull, poly, 25, true);

        cout << "poly.size():" << poly.size() << endl;
    }
    else if (corner_number == 3)
    {
        
        vector<Point> poly_temp;
        vector<Point> hull_temp;
        convexHull(Mat(side), hull_temp, false);
        approxPolyDP(hull_temp, poly_temp, 5, true);
        // cout<<poly_temp.size()<<endl;
        // polylines(img, poly_temp, true, Scalar(255, 0, 0), 2, 8, 0);
        if (poly_temp.size() != 5)
        {
            cout << "wrong number of poly_temp,number:" << poly_temp.size() << endl;
        }
        else
        {

            // 计算相邻两点间距，找到最短的两条线段
            double len[5] = {0};
            for (int i = 0; i < 5; i++)
            {
                len[i] = sqrt((poly_temp[i].x - poly_temp[(i + 1) % 5].x) * (poly_temp[i].x - poly_temp[(i + 1) % 5].x) + (poly_temp[i].y - poly_temp[(i + 1) % 5].y) * (poly_temp[i].y - poly_temp[(i + 1) % 5].y));
            }
            int index[5] = {0, 1, 2, 3, 4};

            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 4 - i; j++)
                {
                    if (len[j] > len[j + 1])
                    {
                        double temp = len[j];
                        len[j] = len[j + 1];
                        len[j + 1] = temp;
                        int temp_index = index[j];
                        index[j] = index[j + 1];
                        index[j + 1] = temp_index;
                    }
                }
            }
            Vec4i LineA, LineB;
            LineA = {poly_temp[index[0]].x, poly_temp[index[0]].y, poly_temp[(index[0] + 1) % 5].x, poly_temp[(index[0] + 1) % 5].y};
            LineB = {poly_temp[index[1]].x, poly_temp[index[1]].y, poly_temp[(index[1] + 1) % 5].x, poly_temp[(index[1] + 1) % 5].y};

            // 求两直线的交点
            double ka, kb;
            ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); // 求出LineA斜率
            kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); // 求出LineB斜率

            Point2f crosspoint_temp;
            crosspoint_temp.x = (ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kb);
            crosspoint_temp.y = (ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb);
            Point crosspoint;
            crosspoint.x = int(crosspoint_temp.x);
            crosspoint.y = int(crosspoint_temp.y);
            poly_temp.push_back(crosspoint);

            // 对加入交点的点集再次凸包和多边形拟合
            convexHull(Mat(poly_temp), hull, false);
            approxPolyDP(hull, poly, 25, true);

            if (poly.size() != 4)
            {
                cout << "wrong number of poly,number:" << poly.size() << endl;
            }
            else
            {
                double k1 = (double)(poly[0].y - poly[1].y) / (double)(poly[0].x - poly[1].x);
                double k2 = (double)(poly[2].y - poly[3].y) / (double)(poly[2].x - poly[3].x);
                double k3 = (double)(poly[1].y - poly[2].y) / (double)(poly[1].x - poly[2].x);
                double k4 = (double)(poly[3].y - poly[0].y) / (double)(poly[3].x - poly[0].x);

                // cout<<"k1:"<<k1<<endl;
                // cout<<"k2:"<<k2<<endl;
                // cout<<"k3:"<<k3<<endl;
                // cout<<"k4:"<<k4<<endl;
                // polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
                if (abs(k1 - k2) >= 0.4 || abs(k3 - k4) >= 0.4)
                {
                    cout << "cannot detect 3 clear corners" << endl;
                    poly.clear();
                }
            }
        }
    }
    else
    {
        cout<<"wrong corner number:"<<corner_number<<endl;
    }

    if (poly.size() == 4)
    {
        for (int i = 0; i < 4; i++)
        {
            len[i] = sqrt((poly[i].x - square.x) * (poly[i].x - square.x) + (poly[i].y - square.y) * (poly[i].y - square.y));
        }
        double len_min = len[0];
        int min_index = 0;
        for (int i = 1; i < 4; i++)
        {
            if (len[i] < len_min)
            {
                len_min = len[i];
                min_index = i;
            }
        }
        min_index = (min_index + 2) % 4;
        for (int i = 0; i < 4; i++)
        {
            // cout << poly[(min_index + i) % 4] << endl;
            anchor_temp.push_back(poly[(min_index + i) % 4]);
        }
        anchor_point.push_back(anchor_temp);
        polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
    }
    if (img.empty())
    {
        cout << "img is empty" << endl;
    }
    else
        imshow("img_poly", img);
}

/* ====================================================================================== */
/* ====================================== 银矿识别部分 =================================== */
/* ====================================================================================== */

void Detector::find_white_mineral(Mat &img, vector<Rect> &side_rect)
{
    Mat output;
    cvtColor(img, output, COLOR_BGR2HSV);
    inRange(output, Scalar(0, 0, 101), Scalar(180, 17, 159), output);

    Mat kernel_middle = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(output, output, kernel_middle);
    erode(output, output, kernel_middle);
    dilate(output, output, kernel, Point(-1, -1), 3);
    erode(output, output, kernel, Point(-1, -1), 5);
    medianBlur(output, output, 3);

    vector<vector<Point>> side_contours;
    vector<Vec4i> side_hierarchy;
    findContours(output, side_contours, side_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < side_contours.size(); i++)
    {
        Rect rec = boundingRect(side_contours[i]);
        double rate = float(rec.width) / float(rec.height);
        double area = float(rec.width) * float(rec.height);

        if (rate >= 1 && rate <= 2 && area >= 80000 && area <= 150000)
        {
            // rectangle(img, rec, Scalar(255, 0, 0), 2, 8);
            side_rect.push_back(rec);
        }
    }
}

Mat Detector::get_white_mineral(Mat &img, Rect rec)
{
    // 抠图
    Mat mask;
    mask = Mat::zeros(img.size(), CV_8UC1);
    mask(rec).setTo(255);
    img.copyTo(mask, mask);

    // 背景变为白色
    for (int i = 0; i < mask.rows; i++)
    {
        for (int j = 0; j < mask.cols; j++)
        {
            if (mask.at<Vec3b>(i, j)[0] == 0 && mask.at<Vec3b>(i, j)[1] == 0 && mask.at<Vec3b>(i, j)[2] == 0)
            {
                mask.at<Vec3b>(i, j)[0] = 255;
                mask.at<Vec3b>(i, j)[1] = 255;
                mask.at<Vec3b>(i, j)[2] = 255;
            }
        }
    }

    return mask;
}

void Detector::process_white_corner(Mat &img, int thresh, int maxual, vector<vector<Point>> &corner_contour)
{
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    // Mat gaussian;
    // GaussianBlur(gray, gaussian, Size(3, 3), 2, 2);
    Mat thre;
    threshold(gray, thre, thresh, maxual, THRESH_BINARY_INV);
    Mat dst;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(thre, thre, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    dilate(thre, dst, kernel, Point(-1, -1), 4);
    medianBlur(dst, dst, 1);
    findContours(dst, gold_mine_contours, gold_mine_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < gold_mine_contours.size(); i++)
    {
        Rect rec = boundingRect(gold_mine_contours[i]);
        double area = rec.height * rec.width;
        if (contourArea(gold_mine_contours[i]) / area <= 0.7)
        {
            corner_contour.push_back(gold_mine_contours[i]);
            rectangle(img, rec, Scalar(255, 0, 0), 2, 8);
        }
    }
    // namedWindow("img",WINDOW_NORMAL);
    // imshow("img",img);
    // waitKey(0);
}


void Detector::get_white_corner(Mat &img, vector<vector<Point>> corner_contour)
{
    for (int i = 0; i < corner_contour.size(); i++)
    {
        vector<Point> poly;
        approxPolyDP(corner_contour[i], poly, 25, true);
        if (poly.size() == 3)
        {
            polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
            double max = poly[0].y;
            int max_index = 0;
            for(int i = 1;i<3;i++)
            {
                if(poly[i].y > max)
                {
                    max = poly[i].y;
                    max_index = i;
                }
            }
            int count = 0;
            int flag = 0;
            for(int i = 0; ;i++)
            {
                if(poly[i].y == max)
                {
                    flag = 1;
                }
                if(flag == 1)
                {
                    cout<<poly[i%3]<<endl;//可以把输出点座标注释掉
                    count++;
                }
                if(count == 3)
                {
                    break;
                }
            }
        }
        
    }
}


/// @brief find red corners of exchange station
/// @param img, the original picture
/// @param corner_contour, to store points of corners' contours;
/// @param square_corner_contour, to store points of square corner's contour;
void Detector::find_site_corner(Mat &img)
{
    Mat output;
    corner_cnt = 0;

    // 图像预处理，得到角点
    cvtColor(img, output, COLOR_BGR2HSV);
    if (param.camp == 0)
        inRange(output, Scalar(0, 0, 150), Scalar(255, 255, 255), output); // red
    else
        inRange(output, Scalar(78, 72, 147), Scalar(122, 255, 255), output); // blue

    // Mat kernel_middle = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    //  erode(output, output, kernel, Point(-1, -1), 3);
    // morphologyEx(output, output, MORPH_CLOSE, kernel, Point(-1, -1), 1);
    //  dilate(output, output, kernel, Point(-1, -1), 3);
    // morphologyEx(output, output, MORPH_OPEN, kernel, Point(-1, -1), 1);
    medianBlur(output, output, 3);

    imshow("output", output);
    thresh = output;//得到处理后的图片，用于后续再次寻找轮廓
    // 寻找角点轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    min_corner_index = 0;
    double min_corner_area = 10000;
    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect rec = minAreaRect(contours[i]);
        double rate = float(rec.size.width) / float(rec.size.height);
        double area = float(rec.size.width) * float(rec.size.height);
        // cout << "area:" << area << endl;
        // cout << "rate:" << rate << endl;

        // 通过旋转矩形面积、长宽比、矩形与角点轮廓的面积比来筛选角点
        if (rate >= param.site_min_rate && rate <= param.site_max_rate && area >= param.site_min_area && area <= param.site_max_area && contourArea(contours[i]) / area <= param.site_area_rate)
        {
            // 将四个角点座标放入同一个容器中
            for (int j = 0; j < contours[i].size(); ++j){
                all_contours.push_back(contours[i][j]);
            }
            valid_contour.push_back(contours[i]);
            // 选出面积最小的角点（即右上角角点）
            anchor_contour.push_back(contours[i][0]);
            if (contourArea(contours[i]) < min_corner_area)
            {
                min_corner_area = contourArea(contours[i]);
                min_corner_index = i;
            }
            corner_cnt++;
            // putText(img, "area:"+to_string(contourArea(contours[i])), contours[i][0], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2, 5);
            // putText(img, "rate:"+to_string(contourArea(contours[i]) / area), Point(contours[i][0].x, contours[i][0].y + 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2, 5);
            // cout << "area:" << area << endl;
            // cout << "contour/rec rate:" << contourArea(contours[i]) / area << endl;
            // Point2f p[4];
            // rec.points(p);
            // for (int i = 0; i < 4; i++)
            // {
            //     line(img, p[i], p[(i + 1) % 4], Scalar(255, 0, 0));
            // }
        }
    }
   

    if (contours.size() > 0){
        // 找到最小面积角点的外接旋转矩形面积
        // RotatedRect rec = minAreaRect(contours[min_corner_index]);
        min_corner_rec = contourArea(contours[min_corner_index]);
        // 提取右上角角点的一个点单独储存，用于后续按顺序输出角点座标
        square_contour.push_back(contours[min_corner_index][0]);
        putText(img, "square:"+to_string(contours[min_corner_index][0].x)+","+to_string(contours[min_corner_index][0].y), Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
        putText(img, "min_aera:"+to_string(min_corner_rec), Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2, 5);
    }

}

/// @brief get coordinates of exchange station
/// @param img, the original picture
/// @param corner_contour, points of corners' contours;
/// @param square_corner_contour, points of square corner's contour;
void Detector::get_station_side(Mat &img)
{
    DebugUI debug_ui;
    debug_ui.right_flag = false;
    debug_ui.area = 0;
    debug_ui.match_rate = 0;
    debug_ui.min_index = 0;
    debug_ui.small_square_point.clear();
    debug_ui.small_square_area.clear();
    debug_ui.poly.clear();

    vector<Point> hull;
    vector<Point> poly;
    convexHull(Mat(all_contours), hull, false);
    approxPolyDP(hull, poly, 25, true);
    polylines(img, poly, true, Scalar(255, 255, 255), 2, 8, 0);
    logger.info("corner_cnt:{}", corner_cnt);
    if (corner_cnt == 4){
        for(int i = 0; i < valid_contour.size(); ++i){
            for (int j = 0; j < valid_contour[i].size(); ++j){
                station_contours.push_back(valid_contour[i][j]);
            }
        }
        get_station_corner(img, station_contours, debug_ui, 0);
    }else if(corner_cnt < 4){
        logger.warn("Wrong number of poly:{}", corner_cnt);
            if(!anchor_contour.empty())
                anchor_point.push_back(anchor_contour);
    }else if(corner_cnt > 4){
        int index = 0;
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
                        temp.push_back(valid_contour[i]);
                        temp.push_back(valid_contour[j]);
                        temp.push_back(valid_contour[k]);
                        temp.push_back(valid_contour[l]);
                        vector<Point> temp_station_contours;
                        for(int i = 0; i < temp.size(); ++i){
                            for (int j = 0; j < temp[i].size(); ++j){
                                temp_station_contours.push_back(temp[i][j]);
                            }
                        }
                        Mat canvas = img.clone();
                        get_station_corner(canvas, temp_station_contours, debug_ui, index++);
                    }
                }
            }
        }
    }
    draw_debug_ui(img, debug_ui);
}

void Detector::draw_debug_ui(Mat &img, DebugUI &debug_ui){
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

void Detector::get_station_corner(Mat &img, vector<Point> four_station_contours, DebugUI &debug_ui, int index){
        
        vector<vector<Point>> temp_anchor_point;
        double len[4] = {0};
        int min_index = 0;
        vector<Point> anchor_temp;
        vector<Point> hull;
        vector<Point> poly;
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        // 通过凸包和多边形拟合框出四个角点
        convexHull(Mat(four_station_contours), hull, false);
        approxPolyDP(hull, poly, 25, true);

        Mat mask;
        mask = Mat::zeros(thresh.size(), CV_8UC1); // 设置蒙版
        fillPoly(mask, poly, Scalar(255, 255, 255)); // 将凸包区域设置为白色

        // mask(rec).setTo(255);
        thresh.copyTo(mask, mask);

        // 找到区域中所有轮廓，并挑选出比最小角点面积还小的轮廓即为正方形点
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++)
        {
            RotatedRect rec = minAreaRect(contours[i]);
            double area = float(rec.size.width) * float(rec.size.height);
            if (area < min_corner_rec && area > 50)
            {
                square_contour.clear();
                square_contour.push_back(contours[i][0]);
            }
        }

        if(square_contour.size() >= 0){
            cv::circle(img, square_contour[0], 3, cv::Scalar(255, 255, 255), 3);
            for (int i = 0; i < 4; i++) // 计算各角点到正方形点的距离
            {
                len[i] = sqrt((poly[i].x - square_contour[0].x) * (poly[i].x - square_contour[0].x) + (poly[i].y - square_contour[0].y) * (poly[i].y - square_contour[0].y));
            }
            double len_min = len[0];
            for (int i = 1; i < 4; i++) // 找到最小距离
            {
                if (len[i] < len_min)
                {
                    len_min = len[i];
                    min_index = i;
                }
            }
            // 从右上角开始输出角点
            for (int i = 0; i < 4; i++)
            {
                // cout << poly[(min_index + i) % 4] << endl;
                anchor_temp.push_back(poly[(min_index + i) % 4]);
            }
            temp_anchor_point.push_back(anchor_temp);
        }

        RotatedRect resRec = minAreaRect(poly);
        float area = resRec.size.width * resRec.size.height;
        // get poly area
        float poly_area = contourArea(poly);
        polylines(img, poly, true, Scalar(255, 0, 0), 2, 8, 0);
        putText(img, "match_rate:"+to_string(poly_area / area), poly[min_index] , FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2, 5);
        cv::Point2f vertices[4];
        resRec.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            line(img, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
        }
        writeImageRaw(index, img);

        if(poly_area / area > debug_ui.match_rate){
            logger.warn("match rate:{}", poly_area / area);
            anchor_point = temp_anchor_point;
            debug_ui.match_rate = poly_area / area;
            debug_ui.poly = poly;
            debug_ui.area = area;
            debug_ui.min_index = min_index;
            debug_ui.small_square_point.clear();
            for (int i = 0; i < contours.size(); i++)
            {
                RotatedRect rec = minAreaRect(contours[i]);
                double area = float(rec.size.width) * float(rec.size.height);
                if (area < min_corner_rec && area > 50)
                {
                    debug_ui.small_square_area.push_back(area);
                    debug_ui.small_square_point.push_back(contours[i][0]);
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


void Detector::initVideoRaw() {

    for(auto num: param.detector_writer_set){
        VideoWriter videoWriter(param.get_log_path(num, param.detector_prefix) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
        detector_writer_map.insert({num,videoWriter});
        logger.critical("write video:{}", param.get_log_path(num, param.detector_prefix));
    }
}

void Detector::writeVideoRaw(cv::Mat &img) {

    if(detector_writer_map.empty()){
        return;
    }
    for(auto num: param.detector_writer_set){
        VideoWriter videoWriter = detector_writer_map.at(num);
        videoWriter.write(img);
        // logger.info("write video:{}", param.get_log_path(num, param.detector_prefix));
        if (frame_index++ >= 200) {
            frame_index = 0;
            videoWriter.release();
            detector_writer_map.at(num) = VideoWriter(param.get_log_path(num, param.detector_prefix) + ".mp4", param.codec, param.sensor_fps, Size(param.frame_width, param.frame_height));
        }
    }
}

void Detector::writeImageRaw(int index, Mat& img){
    for(auto num: param.writer_set){
        imwrite(param.get_log_path(num, param.shot_prefix, index) + ".jpg", img);
    }
}