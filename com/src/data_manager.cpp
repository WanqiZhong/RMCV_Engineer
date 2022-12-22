//
// Created by ayb on 2021/7/10.
//

#include <data_manager.h>
#include <parameter.h>
extern RmParameter g_para;
DataManager::DataManager(SendData *new_data)
        : data_(
        new_data) { //不如就使用pimpl手法!,注意shared_ptr构造函数只接受非const
    data_->start_flag = 's';
    data_->is_error_data = 0;
    data_->is_standard = 0;
    data_->error_x = 0;
    data_->end_flag = 'e';
}

DataManager::DataManager(shared_ptr<SendData> new_data) : data_(new_data) {
    data_->start_flag = 's';
    data_->is_error_data = 0;
    data_->is_standard = 0;
    data_->error_x = 0;
    data_->end_flag = 'e';
}

void DataManager::update_error(const int &new_error) {
    data_->error_x = new_error;
}

void DataManager::update_direction(const char &new_dir) {
    data_->direction_x = static_cast<int>(new_dir); // ASC?
}

bool DataManager::update_state(cv::Point &target_points) {
    int error = target_points.x - g_para.src_width_ / 2;
    if (error < 0) {
        update_direction('l');
        update_error(0 - error);
        if (data_->error_x > g_para.error_max)
            update_error(g_para.error_max);
    } else {
        update_direction('r');
        update_error(error);
        if (data_->error_x > g_para.error_max)
            update_error(g_para.error_max);
    }
#ifdef DEBUG
    show_data();
#endif
    if (data_->error_x < g_para.mineral_threshold_error) {
        data_->is_standard = 1;
        return true;
    }
    return false;
}

inline void DataManager::show_data() const {
    cout << "Error X:" << data_->error_x << "       Direction X:" << data_->direction_x << endl;
}

DataManager::~DataManager() {}
