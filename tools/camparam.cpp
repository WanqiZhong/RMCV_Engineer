#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <string.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>


const char *camera_id = "/dev/video0";
int fd;

int setCamParam (struct v4l2_queryctrl *qctrl, int value)
{
    struct v4l2_control ctrl;
    ctrl.id = qctrl->id;
    ctrl.value = value;

    int ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    if(ret == 0){
        printf("Succeed ");
    }
    else{
        printf("Failed ");
    }
    printf("set %s = %d\n", qctrl->name ,ctrl.value);

    return 0;
}

int setCamParam (int id, int value)
{
    struct v4l2_control ctrl;
    struct v4l2_queryctrl qctrl;

    qctrl.id = id;
    ioctl(fd, VIDIOC_QUERYCTRL, &qctrl);

    ctrl.id = id;
    ctrl.value = value;

    int ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    if(ret == 0){
        printf("Succeed ");
    }
    else{
        printf("Failed ");
    }
    printf("set %s = %d\n", qctrl.name ,ctrl.value);

    return 0;
}

void queryCamParam(int id)
{
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

int main(int argc, char **argv)
{
    // Input /dev/video*
    if(argc >= 2){
        camera_id = argv[1];
    }

    fd = open(camera_id, O_RDWR);
    if (fd < 0)
    {
        perror("open video failed");
        return -1;
    }

    // AUTO_WHITE_BALANCE  1-ON 0-OFF
    setCamParam(V4L2_CID_AUTO_WHITE_BALANCE, 1);
    
    setCamParam(V4L2_CID_EXPOSURE_AUTO, 1);

    // EXPOSURE_TIME  minimum=1, maximum=5000
    setCamParam(V4L2_CID_EXPOSURE_ABSOLUTE, 10);
    

    /* Below can't work. */
    /* queryCamParam(V4L2_CID_EXPOSURE); */

    // WHITE_BALANCE_TEMPERATURE  minimum=2800, maximum=6500
    // queryCamParam(V4L2_CID_WHITE_BALANCE_TEMPERATURE);

    /* Below equal to V4L2_CID_WHITE_BALANCE_TEMPERATURE */
    /*  queryCamParam(V4L2_CID_BLACK_LEVEL);
        queryCamParam(V4L2_CID_DO_WHITE_BALANCE);
        queryCamParam(V4L2_CID_BLUE_BALANCE);
        queryCamParam(V4L2_CID_AUTO_WHITE_BALANCE);  */
        
     while(true){
     	queryCamParam(V4L2_CID_BRIGHTNESS);
     }

    close(fd);
}
