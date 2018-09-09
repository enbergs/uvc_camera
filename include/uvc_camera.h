//
// Created by jsvirzi on 9/4/17.
//

#ifndef UVC_CAMERA_H
#define UVC_CAMERA_H

class UvcCamera;
typedef int (UvcCameraCallbackFxn)(UvcCamera *camera, unsigned char *data, void *ext);
typedef int (*UvcCameraLogFxn)(int level, const char *str, int length);
typedef int (*UvcCameraProcessImageFxn)(UvcCamera *camera, const unsigned char *i_img, unsigned char *o_img);



//R = Y + 1.140V
//G = Y - 0.395U - 0.581V
//B = Y + 2.032U

const uint16_t yuv_to_bgr_yb = 1;
const uint16_t yuv_to_bgr_yg = 1;
const uint16_t yuv_to_bgr_yr = 1;
const uint16_t yuv_to_bgr_ub = 0;
const uint16_t yuv_to_bgr_ug = 0;
const uint16_t yuv_to_bgr_ur = 0;
const uint16_t yuv_to_bgr_vb = 0;
const uint16_t yuv_to_bgr_vg = 0;
const uint16_t yuv_to_bgr_vr = 0;

void uyvy12ToArgb(const uint8_t *src, uint32_t *dst, int width, int height);
void uyvy8ToBgr(const uint8_t *src, uint32_t *dst, int width, int height);
void yuv422_to_y(const unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height, bool reverse_image = false);
void yuv422_to_yuv420sp(const unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height);
void YUV422_to_RGBA(const unsigned char *src, unsigned char * dst, unsigned int width, unsigned int height);
void quick_YUV422_to_RGBA(const unsigned char *src, uint32_t *dst, unsigned int width, unsigned int height);
void initialize_UYVY_to_RGBA();

class UvcCamera {
public:

    bool verbose;

    enum {
        ERROR_FRAME_TIMEOUT = -1,
        ERROR_SELECT = -2,
        ERROR_UVC_DQBUF = -3,
        ERROR_UVC_QBUF = -4,
        ERROR_CAPS = -5,
        ERROR_PIXEL_FORMAT = -6,
        ERROR_GET_PIXEL_FORMAT = -7,
        ERROR_REQUEST_BUFFERS = -8,
        ERROR_QUERY_BUFFER = -9,
        ERROR_MMAP = -10,
        ERROR_QUEUE_BUFFER = -11,
        ERROR_ENABLE_STREAMING = -12,
    };

    enum {
        LEVEL_DEBUG,
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
        CAMERA_ECON_SEE3CAMCU20,
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
    int n_capture_buffers;
    unsigned char **capture_buffer;
    size_t *capture_length;

    typedef struct {
        uint8_t *payload;
        uint32_t index;
        uint64_t timestamp;
        uint32_t flags;
    } FrameData;

    char *log_buffer;
    size_t log_buffer_size;
    UvcCamera(const char *device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn = 0);
    UvcCamera(int device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn = 0);
    void setup(const char *device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn);
    int open(int n_capture_buffers = 0, uint32_t pixel_format = 0);
    int close();
    UvcCameraLogFxn log_fxn;
    ~UvcCamera();
    int getFrame(FrameData *frame_data);
    int releaseFrame(int index);
    void setFrameTimeout(time_t waitMilliseconds);
    time_t frame_timeout_ms;

private:
    static int defaultLogFxn(int level, const char *msg, int len);
    static int defaultProcessImage(UvcCamera *camera, const unsigned char *i_img, unsigned char *o_img);
};

#endif // UVC_CAMERA_H
