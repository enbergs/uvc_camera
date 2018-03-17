//
// Created by jsvirzi on 9/4/17.
//

#ifndef UVC_CAMERA_H
#define UVC_CAMERA_H

typedef int (UvcCameraCallbackFxn)(class Camera *camera, unsigned char *data, void *ext);
typedef int (UvcCameraLogFxn)(int level, const char *str);

enum {
    FromCameraThreadId = 0,
    NThreadIds
} ThreadIds;

enum {
    LEVEL_WARNING,
    LEVEL_INFO,
    LEVEL_ERROR,
    LEVEL_FATAL
};

typedef struct {

    pthread_t *tid;

    int fd, device_status, n_capture_buffers;
    uint32_t width, height;
    unsigned char **capture_buffer;
    size_t *capture_length;
    unsigned char **user_buffer;
    size_t *user_buffer_length;
    unsigned int *user_buffer_status;

    int setup(const char *device_id, uint32_t width, uint32_t height);
    int init();
    UvcCameraLogFxn *log_fxn;

} UvcCamera;

#endif // UVC_CAMERA_H
