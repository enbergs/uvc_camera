//
// Created by jsvirzi on 9/4/17.
//

#ifndef UVC_CAMERA_H
#define UVC_CAMERA_H

class UvcCamera;
typedef int (UvcCameraCallbackFxn)(UvcCamera *camera, unsigned char *data, void *ext);
typedef int (*UvcCameraLogFxn)(int level, const char *str, int length);
typedef int (*UvcCameraProcessImageFxn)(UvcCamera *camera, const unsigned char *i_img, unsigned char *o_img);

class UvcCamera {
public:

    enum {
        CAMERA_STATE_IDLE = 0,
        CAMERA_STATE_RUNNING,
        CAMERA_STATE_STOPPED
    };

    enum {
        LEVEL_WARNING,
        LEVEL_INFO,
        LEVEL_ERROR,
        LEVEL_FATAL
    };

    enum {
        CAMERA_UNKNOWN = 0,
        CAMERA_LI_USB3_MT9M021C,
        CAMERA_LI_USB3_MT9M021M,
        CAMERA_LI_USB3_MT9V034,
        CAMERA_LI_USB3_OV10635,
        CAMERA_ECON_SEE3CAMCU50,
        CAMERA_LI_USB3_AR023Z,
        N_CAMERAS
    };

    enum {
        COMPRESSION_NONE = 0,
        COMPRESSION_JPEG,
        N_COMPRESSION_SCHEMES
    };

    pthread_t tid;

    int fd, device_status;
    uint32_t width, height;
    size_t frame_size;
    unsigned char **capture_buffer;
    size_t *capture_length;
    unsigned char **user_buffer;
    size_t *user_buffer_length;
    unsigned int *user_buffer_status;
    int n_user_buffers, n_capture_buffers;

    char *log_buffer;
    size_t log_buffer_size;
    UvcCamera(const char *device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn = 0);
    UvcCamera(int device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn = 0);
    void setup(const char *device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn);
    int init(int n_capture_buffers = 0, int n_user_buffers = 0);
    UvcCameraLogFxn log_fxn;
    ~UvcCamera();

private:
    static int defaultLogFxn(int level, const char *msg, int len);
    static int defaultProcessImage(UvcCamera *camera, const unsigned char *i_img, unsigned char *o_img);
};

#endif // UVC_CAMERA_H
