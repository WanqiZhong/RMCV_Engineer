#include "UVC.hpp"

UVC::UVC(const char *camera_id) {

    this->camera_id = camera_id;
    fd = open(camera_id, O_RDWR);
    if (fd < 0) {
        perror("open video failed");
    }
}

UVC::~UVC() {
    close(fd);
}

int UVC::setCamParam(struct v4l2_queryctrl *qctrl, int value) {
    struct v4l2_control ctrl;
    ctrl.id = qctrl->id;
    ctrl.value = value;

    int ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    if (ret == 0) {
        printf("Succeed ");
    } else {
        printf("Failed ");
    }
    printf("set %s = %d\n", qctrl->name, ctrl.value);

    return ret;
}

int UVC::setCamParam(int id, int value) {
    struct v4l2_control ctrl;
    struct v4l2_queryctrl qctrl;

    qctrl.id = id;
    ioctl(fd, VIDIOC_QUERYCTRL, &qctrl);

    ctrl.id = id;
    ctrl.value = value;

    int ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    if (ret == 0) {
        printf("Succeed ");
    } else {
        printf("Failed ");
    }
    printf("set %s = %d\n", qctrl.name, ctrl.value);

    return ret;
}

void UVC::queryCamParam(int id, int fd) {
    struct v4l2_queryctrl qctrl;
    qctrl.id = id;
    ioctl(fd, VIDIOC_QUERYCTRL, &qctrl);

    struct v4l2_control ctrl;
    ctrl.id = qctrl.id;
    ioctl(fd, VIDIOC_G_CTRL, &ctrl);

    printf("%-14s : id=%08x, type=%d, minimum=%d, maximum=%d\n"
           "\t\t value = %d, step=%d, default_value=%d\n",
           qctrl.name, qctrl.id, qctrl.type, qctrl.minimum, qctrl.maximum,
           ctrl.value, qctrl.step, qctrl.default_value);

}

void UVC::initUVC(int explosure, int fps) {

    setCamParam(V4L2_CID_EXPOSURE_AUTO, 1);
    // AUTO_WHITE_BALANCE  1-ON 0-OFF
    setCamParam(V4L2_CID_AUTO_WHITE_BALANCE, 1);

    // EXPOSURE_TIME  minimum=1, maximum=5000
    setCamParam(V4L2_CID_EXPOSURE_ABSOLUTE, explosure);

}

void UVC::queryUVC(){
    queryCamParam(V4L2_CID_AUTO_WHITE_BALANCE, fd);
    queryCamParam(V4L2_CID_EXPOSURE_ABSOLUTE, fd);
}



