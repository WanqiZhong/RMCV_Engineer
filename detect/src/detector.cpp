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
    Mat img;
    while(1)
    {
        img = ImgUpdate(img);
        if(!img.empty())
        {
            PreProcessMine(img);
            FindSide(img);
        }
        else
        {
            logger.info("Detector img is empty");
        }
        // this_thread::sleep_for(chrono::milliseconds(10));
    }
}

Mat Detector::ImgUpdate()
{
    Mat img;
    return img;
}


/// @brief process the RGB image to get the contours(include inRange, morphologyEx, GaussianBlur, Canny, findContours)
/// @param img current RGB frame image
/// @param lower  inRange lower bound
/// @param upper  inRange upper bound
void DemarMine::ImgProcess(const Mat& img,Scalar lower=Scalar(hmin,smin,vmin),Scalar upper=Scalar(hmax,smax,vmax))
{
    Mat img_hsv;
    vector<Mat> img_split ;
    cvtColor(img,img_hsv,COLOR_BGR2HSV);
    split(img_hsv,img_split);
    equalizeHist(img_split[2],img_split[2]);
    merge(img_split,img_hsv);

    GaussianBlur(img_hsv,img_hsv,Size(3,3),2,2);
    inRange(img_hsv,lower,upper,img_hsv); 
    // imshow("inRange",img_hsv);


    Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5)); 
    // morphologyEx(img_hsv, img_hsv,MORPH_OPEN, kernel); 
    erode(img_hsv, img_hsv, kernel);
    // morphologyEx(img_hsv, img_hsv,MORPH_OPEN, kernel); 
    // kernel = getStructuringElement(MORPH_RECT,Size(3,3));
    // kernel = getStructuringElement(MORPH_RECT,Size(5,5)); 
    // morphologyEx(img_hsv, img_hsv,MORPH_OPEN, kernel); 
    medianBlur(img_hsv,img_hsv,1);
    
    Mat img_Canny;
    imshow("aft",img_hsv);
    Canny(img_hsv,img_Canny,20,120);

    // imshow("Canny",img_Canny);

    findContours(img_Canny,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
}

/// @brief process the RGB->gray image to get the contours(GaussianBlur, Canny, findContours)
/// @param img current RGB frame image
/// @param thresh the threshold value
/// @param maxval the threshold max value
/// @param type threshold type
void DemarMine::ImgProcess(const Mat& img,int thresh,int maxval=255,int type=THRESH_BINARY)
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


void DemarMine::ImgEnhance(Mat &img)
{
    double a, b;
    double delta=0;
    delta = 127. * contrast  / 100;
    a = 255. / (255. - delta * 2);
    b = a * (bright - delta);
    img.convertTo(img, CV_8U, a, b);
}



/// @brief find the anchor point of mine
/// @param img current frame image
void DemarMine::PreProcessMine(Mat& img)
{
    ImgEnhance(img);
    ImgProcess(img);
    double whole_area = 0;
    for(int i = 0; i < contours.size(); ++i){
        int k = i;
        if(hierarchy[i][2]!=-1) {
            k = hierarchy[i][2];
        }   
        Rect rect_res = boundingRect(contours[k]);
        double area = contourArea(contours[k]);
        double rect_area = rect_res.width * rect_res.height;
        double rate = area / rect_area;
        Point p(rect_res.x + rect_res.width / 2+25, rect_res.y + rect_res.height / 2+10);
        if(rate>=0.65){  
            if(rect_area>400&&(double)rect_res.width/(double)rect_res.height>0.3&&(double)rect_res.height/(double)rect_res.width>0.3)
            {
                whole_rect.push_back(rect_res);
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
                half_rect.push_back(rect_res);
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
    double single_area = whole_area/whole_rect.size();
    for(int i = 0; i < whole_rect.size(); ++i){
        if(whole_rect[i].area() > single_area*1.5){
            whole_rect.erase(whole_rect.begin()+i);
            whole_area -= single_area;
        }
        else{  
            #ifndef LABEL
            rectangle(img,whole_rect[i],Scalar(0,128,255),2); 
            #endif
        }
    }
}

void DemarMine::FindSide(Mat &img)
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
    cout<<"whole_rect.size():"<<whole_rect.size()<<endl;
    for(int i = 0; i < whole_rect.size(); ++i){
        vector<vector<int>> decision_tree = vector<vector<int>>(4,vector<int>(3,0));
        vector<vector<Rect>> rect_tree = vector<vector<Rect>>(4,vector<Rect>(3));
        lower_bound_wid = whole_rect[i].width*0.4;
        upper_bound_wid = whole_rect[i].width*1.6;
        lower_bound_hei = whole_rect[i].height*0.6;
        upper_bound_hei = whole_rect[i].height*1.4;
        bound_wid = whole_rect[i].width*1.3;
        bound_hei = whole_rect[i].height*1.3;
        for(int k=0;k<3;k++){
            if(k==0)
            {
                lower_bound_y[k] = whole_rect[i].y - 5 * whole_rect[i].height;
                upper_bound_y[k] = whole_rect[i].y - 2 * whole_rect[i].height;    
                lower_bound_x[k] = whole_rect[i].x - 5 * whole_rect[i].width;
                upper_bound_x[k] = whole_rect[i].x - 2 * whole_rect[i].width;
            }
            else if(k==1)
            {
                lower_bound_y[k] = whole_rect[i].y + 2.5 * whole_rect[i].height;
                upper_bound_y[k] = whole_rect[i].y + 5 * whole_rect[i].height;    
                lower_bound_x[k] = whole_rect[i].x + 2.5 * whole_rect[i].width;
                upper_bound_x[k] = whole_rect[i].x + 5 * whole_rect[i].width;
            }
            else if(k==2)
            {
                lower_bound_y[k] = whole_rect[i].y - 1.5 * whole_rect[i].height;
                upper_bound_y[k] = whole_rect[i].y + 1.5 * whole_rect[i].height;    
                lower_bound_x[k] = whole_rect[i].x - 1.5 * whole_rect[i].width;
                upper_bound_x[k] = whole_rect[i].x + 1.5 * whole_rect[i].width;
            }
        }
        // cout<<"lower_bound_y:"<<lower_bound_y[0]<<","<<lower_bound_y[1]<<","<<lower_bound_y[2]<<endl;
        // cout<<"upper_bound_y:"<<upper_bound_y[0]<<","<<upper_bound_y[1]<<","<<upper_bound_y[2]<<endl;
        // cout<<"lower_bound_x:"<<lower_bound_x[0]<<","<<lower_bound_x[1]<<","<<lower_bound_x[2]<<endl;
        // cout<<"upper_bound_x:"<<upper_bound_x[0]<<","<<upper_bound_x[1]<<","<<upper_bound_x[2]<<endl;
        // cout<<"whole_rect[i].x:"<<whole_rect[i].x<<" whole_rect[i].y:"<<whole_rect[i].y<<endl;
        // cout<<"whole_rect[i].width:"<<whole_rect[i].width<<" whole_rect[i].height:"<<whole_rect[i].height<<endl;
        for(int j = 0; j < half_rect.size(); ++j){
            if(half_rect[j].width>lower_bound_wid&&half_rect[j].width<upper_bound_wid&&half_rect[j].height>lower_bound_hei&&half_rect[j].height<upper_bound_hei)
            {
                if(half_rect[j].x>lower_bound_x[1]&&half_rect[j].x<upper_bound_x[1]&&half_rect[j].y>lower_bound_y[0]&&half_rect[j].y<upper_bound_y[0])
                {
                    decision_tree[0][1] = 1;
                    rect_tree[0][1] = half_rect[j];
                }
                else if(half_rect[j].x>lower_bound_x[0]&&half_rect[j].x<upper_bound_x[0]&&half_rect[j].y>lower_bound_y[0]&&half_rect[j].y<upper_bound_y[0])
                {
                    decision_tree[1][1] = 1;
                    rect_tree[1][1] = half_rect[j];
                }
                else if(half_rect[j].x>lower_bound_x[0]&&half_rect[j].x<upper_bound_x[0]&&half_rect[j].y>lower_bound_y[1]&&half_rect[j].y<upper_bound_y[1])
                {
                    decision_tree[2][1] = 1;
                    rect_tree[2][1] = half_rect[j];
                }
                else if(half_rect[j].x>lower_bound_x[1]&&half_rect[j].x<upper_bound_x[1]&&half_rect[j].y>lower_bound_y[1]&&half_rect[j].y<upper_bound_y[1])
                {
                    decision_tree[3][1] = 1;
                    rect_tree[3][1] = half_rect[j];
                }
                else if(abs(half_rect[j].x-whole_rect[i].x)<bound_wid && half_rect[j].y>lower_bound_y[0]&&half_rect[j].y<upper_bound_y[0])
                {
                    decision_tree[0][0] = 1;
                    decision_tree[1][0] = 1;
                    rect_tree[0][0] = half_rect[j];
                    rect_tree[1][0] = half_rect[j];
                }
                else if(abs(half_rect[j].x-whole_rect[i].x)<bound_wid && half_rect[j].y>lower_bound_y[1]&&half_rect[j].y<upper_bound_y[1])
                {
                    decision_tree[2][0] = 1;
                    decision_tree[3][0] = 1;
                    rect_tree[2][0] = half_rect[j];
                    rect_tree[3][0] = half_rect[j];
                }
                else if(abs(half_rect[j].y-whole_rect[i].y)<bound_hei && half_rect[j].x>lower_bound_x[0]&&half_rect[j].x<upper_bound_x[0])
                {
                    decision_tree[1][2] = 1;
                    decision_tree[2][2] = 1;
                    rect_tree[1][2] = half_rect[j];
                    rect_tree[2][2] = half_rect[j];
                    // cout<<"[2] whole_rect[i].x:"<<whole_rect[i].x;
                    // cout<<" whole_rect[i].y:"<<whole_rect[i].y<<endl;
                }
                else if(abs(half_rect[j].y-whole_rect[i].y)<bound_hei && half_rect[j].x>lower_bound_x[1]&&half_rect[j].x<upper_bound_x[1])
                {
                    decision_tree[0][2] = 1;
                    decision_tree[3][2] = 1;
                    rect_tree[0][2] = half_rect[j];
                    rect_tree[3][2] = half_rect[j];
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
                rect_res.push_back(whole_rect[i]);
                if(k==0)
                {
                    center = Point((whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2,(whole_rect[i].y+whole_rect[i].height+rect_tree[k][1].y)/2);
                    line(img, Point(whole_rect[i].x,whole_rect[i].y+whole_rect[i].height), Point(rect_tree[k][0].x,rect_tree[k][0].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x,rect_tree[k][0].y), Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y), Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y+rect_tree[k][2].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y+rect_tree[k][2].height), Point(whole_rect[i].x,whole_rect[i].y+whole_rect[i].height), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(whole_rect[i].y+whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
                else if(k==1)
                {
                    line(img, Point(whole_rect[i].x+whole_rect[i].width,whole_rect[i].y+whole_rect[i].height), Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y), Point(rect_tree[k][1].x,rect_tree[k][1].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x,rect_tree[k][1].y), Point(rect_tree[k][2].x,rect_tree[k][2].y+rect_tree[k][2].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x,rect_tree[k][2].y+rect_tree[k][2].height), Point(whole_rect[i].x+whole_rect[i].width,whole_rect[i].y+whole_rect[i].height), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(whole_rect[i].y+whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
                else if(k==2)
                {
                    line(img, Point(whole_rect[i].x+whole_rect[i].width,whole_rect[i].y), Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y+rect_tree[k][0].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x+rect_tree[k][0].width,rect_tree[k][0].y+rect_tree[k][0].height), Point(rect_tree[k][1].x,rect_tree[k][1].y+rect_tree[k][1].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x,rect_tree[k][1].y+rect_tree[k][1].height), Point(rect_tree[k][2].x,rect_tree[k][2].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x,rect_tree[k][2].y), Point(whole_rect[i].x+whole_rect[i].width,whole_rect[i].y), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(whole_rect[i].y+whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
                }
                else if(k==3)
                {
                    line(img, Point(whole_rect[i].x,whole_rect[i].y), Point(rect_tree[k][0].x,rect_tree[k][0].y+rect_tree[k][0].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][0].x,rect_tree[k][0].y+rect_tree[k][0].height), Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y+rect_tree[k][1].height), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][1].x+rect_tree[k][1].width,rect_tree[k][1].y+rect_tree[k][1].height), Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y), Scalar(255,0,0), 4, 8, 0);
                    line(img, Point(rect_tree[k][2].x+rect_tree[k][2].width,rect_tree[k][2].y), Point(whole_rect[i].x,whole_rect[i].y), Scalar(255,0,0), 4, 8, 0);
                    putText(img, res[k], Point((whole_rect[i].x+rect_tree[k][1].x+rect_tree[k][1].width)/2-40,(whole_rect[i].y+whole_rect[i].height+rect_tree[k][1].y)/2), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255), 2, 8, 0);
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
            rect_res.push_back(whole_rect[i]);
            putText(img, res[max_k], Point(whole_rect[i].x,whole_rect[i].y), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,128,255), 2, 8, 0);
        }
    }
}