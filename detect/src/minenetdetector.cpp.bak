#include "minenetdetector.hpp"

void Minenetdetector::Run()
{
    logger.info("Net Run");
    Detector_thread = thread(&Minenetdetector::DetectorNet_Run,this);
}

void Minenetdetector::Join() {
    logger.info("Waiting for [Net]");
    Detector_thread.join();
    logger.info("[Net] joined");
}

void Minenetdetector::Detector_Run(Mat& img){

}

void Minenetdetector::DetectorNet_Run()
{
    bool receive_flag = 0;
    int first_flag = 1;
    umt::Subscriber<cv::Mat> sub("channel0");
    umt::Publisher<std::vector<std::vector<cv::Point>>> pub("Armor");
    umt::Publisher<cv::Mat> img_pub("channel1");
    while(param.get_run_mode()!=HALT)
    { 
        cv::Mat img = sub.pop();
        cv::Mat ori_img = img.clone();
        anchor_point.clear();
        if(!img.empty()){
            if(param.get_run_mode() == GoldMode)
            {
                cv::Mat new_image = pad_image(img, cv::Size(640, 640));
                cv::Mat canvas;
                std::vector<float> blob(640*640*3);
                cv::resize(img, canvas, cv::Size(640, int(640*(float)(img.rows)/img.cols)));
                img2blob(new_image, blob);
                std::vector<armor::Armor> detections = detect(blob);
                double scale = param.frame_width / 640.0;

                const float threshold = 0.4;
                detections.erase(std::remove_if(detections.begin(), detections.end(), [&](const armor::Armor& armor) {
                    return armor.conf < threshold;
                }), detections.end());

                sort(detections.begin(), detections.end(), [](armor::Armor a, armor::Armor b) {
                    return a.conf > b.conf;
                });
                
                for(int i = 0; i < detections.size(); ++i){
                    vector<cv::Point> pts;
                    for(int j = 0; j < 4; ++j)
                    {
                        detections[i].pts[j].x =  detections[i].pts[j].x * scale;
                        detections[i].pts[j].y =  detections[i].pts[j].y * scale;
                        pts.push_back(detections[i].pts[j]);
                    }
                    anchor_point.push_back(pts);
                }
                logger.info("Armor_size: {}", detections.size());
                draw(img, detections);
            }
            img_pub.push(ori_img);
            pub.push(anchor_point);
        }
    }
}


void Minenetdetector::get_mask(Mat &img, Mat &mask){
    for(int i = 0; i < img.rows; ++i){
        for(int j = 0; j < img.cols; ++j){
            if(mask.at<uchar>(i,j) == 0){
                img.at<Vec3b>(i,j)[0] = 255;
                img.at<Vec3b>(i,j)[1] = 255;
                img.at<Vec3b>(i,j)[2] = 255;
            }
        }
    }
}

void Minenetdetector::get_corner_withnet(Mat &img)
{
    Mat mask = Mat::zeros(img.size(), CV_8UC1);
    for(auto &mine_side:anchor_point){
        int x = 0;
        int y = 0;
        int bound_small = 30;
        int bound_big = 100;
        int w = 100;
        int h = 100;
        for(int t = 0; t < mine_side.size(); t++){
            switch(t){
                case 0:
                    first_points.push_back(mine_side[t]);
                    x = max(mine_side[t].x - bound_small, 0);
                    y = max(mine_side[t].y - bound_small, 0);
                    for(int i = x; i < min(x + w, img.cols); i++){
                        for(int j = y; j < min(y + h, img.rows); j++){
                            mask.at<uchar>(i,j) = 255;
                        }
                    }
                    break;
                case 1:
                    x = max( mine_side[t].x - bound_small, 0);
                    y = max( mine_side[t].y - bound_big, 0);
                    for(int i = x; i < min(x + w, img.cols); i++){
                        for(int j = y; j < min(y + h, img.rows); j++){
                            mask.at<uchar>(i,j) = 255;
                        }
                    }
                    break;
                case 2:
                    third_points.push_back(mine_side[t]);
                    x = max( mine_side[t].x - bound_big, 0);
                    y = max( mine_side[t].y - bound_big, 0);
                    for(int i = x; i < min(x + w, img.cols); i++){
                        for(int j = y; j < min(y + h, img.rows); j++){
                            mask.at<uchar>(i,j) = 255;
                        }
                    }
                    break;
                case 3:
                    x = max( mine_side[t].x - bound_big, 0);
                    y = max( mine_side[t].y - bound_small, 0);
                    for(int i = x; i < min(x + w, img.cols); i++){
                        for(int j = y; j < min(y + h, img.rows); j++){
                            mask.at<uchar>(i,j) = 255;
                        }
                    }
                    break;
            }
        }
    }
    logger.info("get corner with net");
    get_mask(img, mask);
    // imshow("mask", mask);
    waitKey(1);
}




void Minenetdetector::img2blob(cv::Mat &img, std::vector<float> &dst)
{
    // cv::dnn::blobFromImage(img, 1./255, img.size, cv::Scalar(), true);
    // return;
    int img_h = img.rows;
    int img_w = img.cols;

    float *blob_data = dst.data();

    size_t i = 0;
    for (size_t row = 0; row < img_h; ++row)
    {
        uchar *uc_pixel = img.data + row * img.step;
        for (size_t col = 0; col < img_w; ++col)
        {
            // 三通道
            blob_data[i] = (float)uc_pixel[2] / 255.0;
            blob_data[i + img_h * img_w] = (float)uc_pixel[1] / 255.0;
            blob_data[i + 2 * img_h * img_w] = (float)uc_pixel[0] / 255.0;
            uc_pixel += 3;
            ++i;
        }
    }
}

/**
 * @brief write the image(cv::Mat) to the float array (wrapped by InferenceEngine)
 *
 * @param img source image
 * @param blob destination
 */
void Minenetdetector::copyBlob(std::vector<float> &blob, InferenceEngine::Blob::Ptr &ieBlob) {
    InferenceEngine::MemoryBlob::Ptr mblob =
            InferenceEngine::as<InferenceEngine::MemoryBlob>(ieBlob);
    if (!mblob) {
        THROW_IE_EXCEPTION
                << "We expect blob to be inherited from MemoryBlob in matU8ToBlob, "
                << "but by fact we were not able to cast inputBlob to MemoryBlob";
    }
    // locked memory holder should be alive all time while access to its buffer
    // happens
    auto mblobHolder = mblob->wmap();

    float *ie_blob_data = mblobHolder.as<float *>();

    memcpy(ie_blob_data, blob.data(), sizeof(float) * blob.size());
}

cv::Mat Minenetdetector::pad_image(cv::Mat image, cv::Size2i size)
{
    float h = size.height, w = size.width;
    ih = image.rows, iw = image.cols;

    scale = std::min(h / ih, w  / iw);
    int nw = int(iw*scale + 0.5), nh = int(ih*scale + 0.5);

    pad_w = int(w - nw) / 2, pad_h = int(h - nh) / 2;
    cv::resize(image, image, cv::Size(nw, nh), cv::InterpolationFlags::INTER_CUBIC);
    cv::Mat new_image(size, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::Rect roi(pad_w, pad_h, image.cols, image.rows);
    image.copyTo(new_image(roi));

    return new_image;
}

/**
 * @brief Construct a new Detector:: Detector object
 *
 * @param name name of model file (without extension name)
 * @param type detect-mode (0 for amor, 1 for wind)
 * @param log_level level of logging
 */
Minenetdetector::Minenetdetector(std::string name, int _type, int log_level):
        type(_type), INPUT_W(640), INPUT_H(640), NUM_KPTS(8),
        NUM_CLASSES(_type?2:2), NUM_COLORS(2),
        NMS_THRESH(param.detector_args.nms_thresh[_type]), BBOX_CONF_THRESH(param.detector_args.conf_thresh[_type])
{
    // minedetector = std::make_unique<Minedetector>();
    logger.info("start_init_network");

    // load anchors
    // std::string config_path = name + ".toml";
    // auto &&config = toml::parse(config_path);

    // for(int i=0; i<4; ++i) { // TODO: check anchors nums
    //     anchors[i] = Data(anchors_vector[i].begin(), anchors_vector[i].end());
    // }

    model_xml = name + ".xml";
    model_bin = name + ".bin";

    // ie.SetConfig({{CONFIG_KEY(CACHE_DIR), "../net_cache"}});
    network = ie.ReadNetwork(model_xml, model_bin);

    input_name = network.getInputsInfo().begin()->first;

    for (auto iter : network.getOutputsInfo())
    {
        auto dims = iter.second->getDims();
        output_layers.push_back((spl::OutLayer){
                .idx = (int)output_names.size(),
                .num_out = (int)dims[2]});
        output_names.push_back(iter.first);
        // assert(dims[2] == NUM_KPTS + NUM_CLASSES + NUM_COLORS + 1 && "Output dimension wrong!");
        iter.second->setPrecision(InferenceEngine::Precision::FP32);
        // logger.info("found network output: {}, size:{}, no: {}"
        //             ,iter.first,vecsize_to_string(dims),dims[2]);
    }
    executable_network = ie.LoadNetwork(network, "GPU");
    infer_request = executable_network.CreateInferRequest();

    logger.info("network_init_done");
}

/**
 * @brief main process of detection
 *
 * @param blob blob data converted from source image
 * @return std::vector<Armor> detect result
 */
std::vector<armor::Armor> Minenetdetector::detect(std::vector<float> &blob)
{
    auto resize_start = std::chrono::steady_clock::now();
    // cv::Mat pr_img = static_resize(img);
    InferenceEngine::Blob::Ptr ieBlob = infer_request.GetBlob(input_name);
    copyBlob(blob, ieBlob);

    auto infer_start = std::chrono::steady_clock::now();

    infer_request.Infer();

    auto decode_start = std::chrono::steady_clock::now();
    std::vector<armor::Armor> objects;

    for (auto layer : output_layers)
    {
        const InferenceEngine::Blob::Ptr output_blob =
                infer_request.GetBlob(output_names[layer.idx]);
        InferenceEngine::MemoryBlob::CPtr moutput =
                InferenceEngine::as<InferenceEngine::MemoryBlob>(output_blob);
        auto moutputHolder = moutput->rmap();
        const float *net_pred =
                moutputHolder.as<const InferenceEngine::PrecisionTrait<
                        InferenceEngine::Precision::FP32>::value_type *>();

        decode_outputs(net_pred, objects, layer, INPUT_W, INPUT_H);
    }

    objects = do_merge_nms(objects);

    // logger.info("detected {} objects", objects.size());
    return objects;
}

void Minenetdetector::draw(cv::Mat img, const std::vector<armor::Armor> &objects)
{
    for (auto &obj : objects) {
        if(obj.conf < 0.4) continue;

        cv::Scalar color = cv::Scalar(0, 1, 0);
        float c_mean = cv::mean(color)[0];
        cv::Scalar txt_color;
        if (c_mean > 0.5){
            txt_color = cv::Scalar(0, 0, 0);
        }
        else{
            txt_color = cv::Scalar(255, 255, 255);
        }

        // cv::rectangle(image, obj.rect, color * 255, 2);
        for (int p = 0; p < 4; ++p) {
            logger.info("x:{} y:{}\n", obj.pts[p].x, obj.pts[p].y);
            cv::line(img, obj.pts[p], obj.pts[(p + 1) % 4], color * 255, 2);
        }
        // for (int p = 0; p < 5; ++p) {
        //     cv::circle(image, obj.pts[p], 2, cv::Scalar(0, 0, 255));
        // }

        auto text =  to_string(obj.conf * 100);

        int baseLine = 0;
        cv::Size label_size =
                cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

        cv::Scalar txt_bk_color = color * 0.7 * 255;

        int x = obj.rect.x;
        int y = obj.rect.y + 1;

        x = std::max(std::min(x, img.cols), 0);
        y = std::max(std::min(y, img.rows), 0);

        cv::rectangle(
                img,
                cv::Rect(cv::Point(x, y),
                         cv::Size(label_size.width, label_size.height + baseLine)),
                txt_bk_color, -1);

        cv::putText(img, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
    }
    cv::// imshow("[NET]",img);
    waitKey(1);
}

/**
 * @brief resize a image without changing the original aspect ratio
 *
 * @param img source image
 * @return cv::Mat result image
 */

cv::Mat Minenetdetector::static_resize(cv::Mat &img)
{
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    // r = std::min(r, 1.0f);
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(img, re, re.size());
    // cv::Mat out(INPUT_W, INPUT_H, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat out(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
    re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
    return out;
}

/**
 * @brief decode the output of network model
 *
 * @param prob raw float point, network output
 * @param objects gain detected objects to it by @a push_back
 * @param layer_info information about the last layer, differs by output stride of YOLO
 * @param img_w width of original image
 * @param img_h height of original image
 */
void Minenetdetector::decode_outputs(const float *prob, std::vector<armor::Armor> &objects,
                              spl::OutLayer layer_info, const int img_w,
                              const int img_h)
{
    // std::vector<int> classIds;
    // std::vector<int> indices;
    // std::vector<float> confidences;
    // std::vector<cv::Rect> bboxes;
    float scale = std::min(INPUT_W / (img_w * 1.0), INPUT_H / (img_h * 1.0));
    int no = layer_info.num_out;
    float pred_data[no];

    for (int idx = 0; idx < 8400; ++idx)
    {
        float rough_conf = *std::max_element(&prob[idx * no + NUM_KPTS],
                                             &prob[idx * no + no]);

        if (rough_conf > BBOX_CONF_THRESH)
        {
            std::memcpy(pred_data, &prob[idx * no], no*sizeof(float));
            int col_id = std::max_element(pred_data + NUM_KPTS + NUM_CLASSES + 2,
                                          pred_data + NUM_KPTS + NUM_CLASSES + 2 +
                                          NUM_COLORS) -
                         (pred_data + NUM_KPTS + NUM_CLASSES + 2);
            int cls_id =
                    std::max_element(pred_data + NUM_KPTS,
                                     pred_data + NUM_KPTS + NUM_CLASSES ) -
                    (pred_data + NUM_KPTS);

            int t_size = std::max_element(pred_data + NUM_KPTS + NUM_CLASSES,
                                          pred_data + NUM_KPTS + NUM_CLASSES + 2) -
                         (pred_data + NUM_KPTS + NUM_CLASSES);

            double final_conf = std::min({pred_data[NUM_KPTS + NUM_CLASSES + 2 + col_id],
                                          pred_data[NUM_KPTS + cls_id]});
            if (final_conf > BBOX_CONF_THRESH)
            {
                // std::cout << final_conf << " " << col_id << " "
                //           << cls_id << std::endl;
                armor::Armor now;

                for (int p = 0; p < (NUM_KPTS / 2); ++p)
                {
                    float px = std::max(std::min((pred_data[p * 2] - pad_w) / scale, (float)(img_w)), 0.f);
                    float py = std::max(std::min((pred_data[p * 2 + 1] - pad_h) / scale, (float)(img_h)), 0.f);
                    now.pts[p] = cv::Point2f(px, py);
                }

                now.rect = cv::Rect(now.pts[0], now.pts[2]);
                now.conf = final_conf;
                now.color = col_id;
                now.type = cls_id;
                now.t_size = t_size;
                objects.push_back(now);
            }
        }
    }
}

/**
 * @brief do Non-Maximum Suppression on detected objects, avoiding overlapped objects
 *
 * @param objects all detected objects
 * @return std::vector<armor::Armor> result
 */
std::vector<armor::Armor> Minenetdetector::do_nms(std::vector<armor::Armor> &objects)
{
    std::vector<int> classIds;
    std::vector<int> indices;
    std::vector<float> confidences;
    std::vector<cv::Rect> bboxes;
    std::vector<armor::Armor> result;
    for (size_t i = 0; i < objects.size(); ++i)
    {
        bboxes.push_back(objects[i].rect);
        confidences.push_back(objects[i].conf);
        int cls_id = objects[i].color * 9 + objects[i].type;
        classIds.push_back(cls_id);
    }
    cv::dnn::NMSBoxes(bboxes, confidences, BBOX_CONF_THRESH, NMS_THRESH,
                      indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        result.push_back(objects[indices[i]]);
    }
    return result;
}


float Minenetdetector::calc_iou(const armor::Armor& a,const armor::Armor& b){
    cv::Rect_<float> inter = a.rect & b.rect;
    float inter_area = inter.area();
    float union_area = a.rect.area() + b.rect.area() - inter_area;
    return inter_area / union_area;
}

/**
 * @brief do Merge Non-Maximum Suppression on detected objects, avoiding overlapped objects
 *
 * @param objects all detected objects
 * @return std::vector<armor::Armor> result
 */
std::vector<armor::Armor> Minenetdetector::do_merge_nms(std::vector<armor::Armor> &objects){
    std::vector<armor::Armor> result;
    std::vector<pick_merge_store> picked;

    std::sort(objects.begin(),objects.end(),armor_compare());
    for(int i = 0;i < objects.size();++i){
        armor::Armor& now = objects[i];
        bool keep = 1;
        for(int j = 0;j < picked.size();++j){
            armor::Armor& pre = objects[picked[j].id];
            float iou = calc_iou(now,pre);

            //store for merge_nms
            if(iou > NMS_THRESH || isnan(iou)){
                keep = 0;
                if(iou > MERGE_THRESH && now.color == pre.color && now.type == pre.type && now.t_size == pre.t_size){
                    picked[j].merge_confs.push_back(now.conf);
                    for(int k = 0;k < 5; ++k){
                        picked[j].merge_pts.push_back(now.pts[k]);
                    }
                }
                break;
            }
        }
        if(keep){
            picked.push_back({i,{},{}});
        }
    }
    for(int i = 0;i < picked.size();++i){
        int merge_num = picked[i].merge_confs.size();
        armor::Armor now = objects[picked[i].id];
        double conf_sum = now.conf;
        for(int j = 0;j < 5;++j) now.pts[j] *= now.conf;
        for(int j = 0;j < merge_num;++j){
            for(int k = 0;k < 5;++k){
                now.pts[k] += picked[i].merge_pts[j * 5 + k] * picked[i].merge_confs[j];
            }
            conf_sum += picked[i].merge_confs[j];
        }
        for(int j = 0;j < 5;++j) now.pts[j] /= conf_sum;
        result.emplace_back(now);
    }
    return result;
}
