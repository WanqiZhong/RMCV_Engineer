#include "detector.hpp"

Detector::Detector(){};
Detector::~Detector(){};

void Detector::Run()
{
    logger.info("Detector Run");
    Detector_thread = thread(&Detector::Detect_Run,this);
}

void Detector::Join()
{
    logger.info("Waiting for [Detector]");
    Detector_thread.join();
    logger.sinfo("[Detector] joined");
}

void Detector::Detect_Run()
{
    cv:Mat img;
    umt::Subscriber<cv::Mat> pub("channel1");
    umt::Publisher<MINE_POSITION_MSG> mine_sub("anchor_point_data");
    while((mode=param.get_run_mode())!=HALT)
    {
        try{
            img = pub.pop();
        }
        catch(const HaltEvent&){
            break;
        }
        if(mode==GoldMode)
        {
            GoldMineDetect_Run(img);
            // GoldMineDetect_Run2(img);
            mine_sub.push(MINE_POSITION_MSG(anchor_point));
            logger.info("GoldMineDetect_Run");
        }
        else if(mode==SilverMode)
        {
            SilverMineDetect_Run(img);
            // mine_sub.push(MINE_POSITION_MSG(silver_mine_rect));
            logger.info("SilverMineDetect_Run");
        }
        else if(mode==ChangeSiteMode)
        {
            ChangeSiteDetect_Run(img);
        }
        else
        {
            logger.info("Detector mode error");
        }
    }
}

void Detector::GoldMineDetect_Run(Mat &img)
{
    find_gold_mine(img);
    get_gold_mine(img);
}

void Detector::GoldMineDetect_Run2(Mat &img)
{
    process_img_corner(img, gold_thresh, gold_maxual);
    logo_R = find_R(side_num);
    gold_mine_side = store_side(logo_R, img);
    for (int i = 0; i < side_num; i++)
    {
        draw_side(img, gold_mine_side[i]);
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
    // imshow("aft",img_hsv);
    Canny(img_hsv,img_Canny,20,120);

    findContours(img_Canny,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
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
    findContours(img_Canny,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
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
                putText(img, s, p, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 128, 255), 2, 8, 0);
                putText(img, s1, p+Point(0,20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2, 8, 0);
                #endif
            }
        } 
        else if (rate>=0.35){
            if(rect_area>400&&(double)rect_res.width/(double)rect_res.height>0.2&&(double)rect_res.height/(double)rect_res.width>0.2)
            {
                gold_mine_half_rect.push_back(rect_res);
                #ifndef LABEL
                drawContours(img,contours,k,Scalar(255, 128, 255),2);          
                rectangle(img,rect_res,Scalar(255,128,255),2);
                string s = to_string(rect_res.x) + "," + to_string(rect_res.y); 
                string s1 = to_string(rect_res.width) + "," + to_string(rect_res.height);
                putText(img, s, p, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 128, 255), 2, 8, 0);
                putText(img, s1, p+Point(0,20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2, 8, 0);
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
            rectangle(img,gold_mine_whole_rect[i],Scalar(0,128,255),2); 
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

/// @brief process the RGB image to get the contours of corner(include cvtColor, threshold, GaussianBlur, findContours)
/// @param img current RGB frame image

void Detector::process_img_corner(const Mat &img, int thresh, int maxual)
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

    findContours(dst, gold_mine_contours, gold_mine_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}

/// @brief find contours of logo "R" (include contourArea)
/// @param side_number the number of sides
/// @return vector<vector<Point>> logo_R
vector<vector<Point>> Detector::find_R(int side_number)
{
    vector<double> contour_area;
    vector<vector<Point>> logo_R;

    for (int i = 0; i < gold_mine_contours.size(); i++)
    {
        contour_area.push_back(contourArea(gold_mine_contours[i]));
    }
    for (int i = 0; i < gold_mine_contours.size(); i++)
    {
        for (int j = 0; j < gold_mine_contours.size() - i - 1; j++)
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
        for (int j = 0; j < gold_mine_contours.size(); j++)
        {
            if (contour_area[i] == contourArea(gold_mine_contours[j]))
            {
                logo_R.push_back(gold_mine_contours[j]);
                break;
            }
        }
    }
    return logo_R;
}

/// @brief find the four corners closet to "R"
/// @param p the center of "R"
/// @return vector<vector<Point>> logo_R
vector<int> Detector::sort_length(Point p)
{
    double len = 0;

    Point corner_center;

    vector<double> len_temp;
    for (int i = 0; i < contours.size(); i++)
    {
        corner_center = Point(0, 0);
        for (int j = 0; j < contours[i].size(); j++)
        {
            corner_center.x += gold_mine_contours[i][j].x;
            corner_center.y += gold_mine_contours[i][j].y;
        }
        corner_center.x /= gold_mine_contours[i].size();
        corner_center.y /= gold_mine_contours[i].size();

        len = sqrt(((double)p.x - (double)corner_center.x) * ((double)p.x - (double)corner_center.x) + ((double)p.y - (double)corner_center.y) * ((double)p.y - (double)corner_center.y));
        len_temp.push_back(len);
    }
    for (int i = 0; i < gold_mine_contours.size(); i++)
    {
        for (int j = 0; j < gold_mine_contours.size() - i - 1; j++)
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
        for (int i = 0; i < gold_mine_contours.size(); i++)
        {

            corner_center = Point(0, 0);
            for (int j = 0; j < gold_mine_contours[i].size(); j++)
            {
                corner_center.x += gold_mine_contours[i][j].x;
                corner_center.y += gold_mine_contours[i][j].y;
            }
            corner_center.x /= gold_mine_contours[i].size();
            corner_center.y /= gold_mine_contours[i].size();

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
vector<vector<Point>> Detector::store_side(vector<vector<Point>> logo_R, Mat img)
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
            for (int k = 0; k < gold_mine_contours[len_min[i][j]].size(); k++)
            {
                side_.push_back(gold_mine_contours[len_min[i][j]][k]);
            }
        }
        side.push_back(side_);
    }
    return side;
}

/// @brief draw sides
/// @param img, the original picture
/// @param side, all contours'points in each side

void Detector::draw_side(Mat img, vector<Point> side)
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
    findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        Rect rec = boundingRect(contours[i]);
        double area = rec.height * rec.width;
        if (contourArea(contours[i]) / area <= 0.7)
        {
            corner_contour.push_back(contours[i]);
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
                    cout<<poly[i%3]<<endl;
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

