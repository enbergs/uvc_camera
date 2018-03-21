//
// Created by jsvirzi on 9/4/17.
//

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <inttypes.h>
#include <malloc.h>
#include <limits.h>

#include "uvc_camera.h"

/* TODO xioctl macro */
static int xioctl(int fd, int request, void *arg) {
#ifdef VERBOSE_DEBUG
    printf("xioctl(fd=%d, request=%d, arg=%p)\n", fd, request, arg);
#endif
    int status = ioctl(fd, request, arg);
#ifdef VERBOSE_DEBUG
    printf("xioctl(fd=%d, request=%d, arg=%p) return %d\n", fd, request, arg, status);
#endif
    return status;
}

void UvcCamera::setup(const char *device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn) {
    fd = ::open(device_id, O_RDWR | O_NONBLOCK);
    if(fd == -1) { perror("Opening video device"); }
    this->width = width;
    this->height = height;
    this->log_fxn = (log_fxn) ? log_fxn : &defaultLogFxn;
    verbose = false;
    frame_timeout_ms = 0;
}

int UvcCamera::defaultLogFxn(int level, const char *msg, int len) {
    int cur_time = time(0);
    printf("LOG: TIME=%d LEVEL=%d. MESSAGE=[%s]\n", cur_time, level, msg);
    return 0;
}

UvcCamera::UvcCamera(int device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn) {
    char *device_id_string = new char [ PATH_MAX ];
    snprintf(device_id_string, PATH_MAX, "/dev/video%d", device_id);
    setup(device_id_string, width, height, log_fxn);
}

UvcCamera::UvcCamera(const char *device_id, uint32_t width, uint32_t height, UvcCameraLogFxn log_fxn) {
    setup(device_id, width, height, log_fxn);
}

/* at this point: height, width and pixel size are known. init() initializes all buffers and camera parameters.
 * user must call start_capture() and stop_capture() to start and stop streaming from camera.
   complement of this function is close() which should be called when camera will no longer be used */
int UvcCamera::open(int n_capture_buffers, uint32_t pixel_format) {

    if (pixel_format == 0) { pixel_format = V4L2_PIX_FMT_YUYV; }

    struct v4l2_capability caps;
    memset(&caps, 0, sizeof(struct v4l2_capability));
    if (xioctl(fd, VIDIOC_QUERYCAP, &caps) == -1) {
        perror("query caps");
        return ERROR_CAPS;
    }

    const int kDefaultCaptureBuffers = 4;
    if (n_capture_buffers == 0) { n_capture_buffers = kDefaultCaptureBuffers; }

    this->n_capture_buffers = n_capture_buffers;

    const size_t kLogBufferSize = 256;
    log_buffer_size = kLogBufferSize;
    log_buffer = new char [kLogBufferSize];

/* set desired parameters */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if(xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("setting pixel format");
        return ERROR_PIXEL_FORMAT;
    }

/* read them back out to see how we actually configured */
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
        perror("getting pixel format");
        return ERROR_GET_PIXEL_FORMAT;
    }

    uint32_t actual_image_w = fmt.fmt.pix.width;
    uint32_t actual_image_h = fmt.fmt.pix.height;

    char fourcc[5];

    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    size_t n_bytes = snprintf(log_buffer, log_buffer_size, "granted camera parameters:\nWxH = %dx%d\nPixFmt: %s\nField: %d",
        actual_image_w, actual_image_h, fourcc, fmt.fmt.pix.field);
    if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);

    if((width != actual_image_w) || (height != actual_image_h)) {
        if(log_fxn) (*log_fxn)(LEVEL_WARNING, "requested image dimensions differ from granted values", 0);
    }

/* from here, height and width are sync'ed to the camera */
    width = actual_image_w;
    height = actual_image_h;

/*** mmap ***/
    n_bytes = snprintf(log_buffer, log_buffer_size, "initializing mmap: requesting %d buffers", n_capture_buffers);
    if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);

/* request buffers */
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(struct v4l2_requestbuffers));
    int requested_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.count = n_capture_buffers;
    req.type = requested_type;
    req.memory = V4L2_MEMORY_MMAP;
    if(xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("requesting buffer");
        return ERROR_REQUEST_BUFFERS;
    }

/* check request */
    n_bytes = snprintf(log_buffer, log_buffer_size, "initializing mmap: %d buffers granted. requested = %d",
        req.count, n_capture_buffers);
    if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);
    n_capture_buffers = req.count;

    capture_buffer = new unsigned char * [n_capture_buffers];
    capture_length = new size_t [n_capture_buffers];

    if(req.type != requested_type) {
        n_bytes = snprintf(log_buffer, log_buffer_size,
            "WARNING: driver changed TYPE of requested buffer (requested: %d granted: %d)", requested_type, req.type);
        if(log_fxn) (*log_fxn)(LEVEL_WARNING, log_buffer, n_bytes);
    }

    struct v4l2_buffer buf;
    for(unsigned int i=0;i<n_capture_buffers;++i) {

        n_bytes = snprintf(log_buffer, log_buffer_size, "init_mmap(): VIDIOC_QUERYBUF %d", i);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if(xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("VIDIOC_QUERYBUF");
            return ERROR_QUERY_BUFFER;
        }

        n_bytes = snprintf(log_buffer, log_buffer_size, "init_mmap(): mmap(length=%d, offset=%d)", buf.length, buf.m.offset);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);
        capture_length[i] = buf.length;
        capture_buffer[i] = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if(capture_buffer[i] == MAP_FAILED) {
            if(log_fxn) (*log_fxn)(LEVEL_FATAL, "mmap operation failed", 0);
            return ERROR_MMAP;
        }

        n_bytes = snprintf(log_buffer, log_buffer_size, "Length: %zd Address: %p", buf.length, capture_buffer[i]);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);
    }

    if(log_fxn) (*log_fxn)(LEVEL_INFO, "mmap initialization complete", 0);

    /* queue buffers */
    for(unsigned int i=0;i<n_capture_buffers;++i) {
        size_t n_bytes = snprintf(log_buffer, log_buffer_size, "camera: start_capture(): VIDIOC_QBUF %d", i);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            n_bytes = snprintf(log_buffer, log_buffer_size, "error: VIDIOC_QBUF %d", i);
            if(log_fxn) (*log_fxn)(LEVEL_ERROR, log_buffer, n_bytes);
            perror("queue buffer");
            return ERROR_QUEUE_BUFFER;
        }
    }

/* enable streaming */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("start capture");
        return ERROR_ENABLE_STREAMING;
    }

    frame_size = width * height * 4; /* accommodate BGRA */

    return true;

}

#if 0
/* thoroughly UNtested */
void uyvy12ToArgb(const uint8_t *src, uint32_t *dst, int width, int height) {
    uint8_t *p = (uint8_t *)src;
    uint8_t t1, t2, t3;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; col += 2) {
            t1 = *p++;
            t2 = *p++;
            t3 = *p++;
            uint16_t u = (t1 << 4) & (t2 >> 4);
            uint16_t y0 = ((t2 & 0xf) << 8) | t3;
            t1 = *p++;
            t2 = *p++;
            t3 = *p++;
            uint16_t v = (t1 << 4) & (t2 >> 4);
            uint16_t y1 = ((t2 & 0xf) << 8) | t3;

            uint16_t b = yuv_to_bgr_yb * y0 + y0 + yuv_to_bgr_ub * u + yuv_to_bgr_vb * v;
            uint16_t g = yuv_to_bgr_yg * y0 + y0 + yuv_to_bgr_ug * u + yuv_to_bgr_vg * v;
            uint16_t r = yuv_to_bgr_yr * y0 + y0 + yuv_to_bgr_ur * u + yuv_to_bgr_vr * v;
            uint32_t d = 0xff;
            d = (d << 8) | (b >> 4);
            d = (d << 8) | (g >> 4);
            d = (d << 8) | (r >> 4);
            *dst++ = d;
        }
    }
}
#endif

uint32_t UYVY_to_RGBA[256 * 256 * 256];
void initialize_UYVY_to_RGBA() {
    uint32_t *lut = UYVY_to_RGBA;
    int r, g, b, y, u, v, tu, tv;
    // Clamp out of range values
#define CLAMPRGB(t) (((t)>255)?255:(((t)<0)?0:(t)))
    for (int y = 0; y < 256; ++y) {
        for (int u = 0; u < 256; ++u) {
            for (int v = 0; v < 256; ++v) {
                tu = u - 128;
                tv = v - 128;

                // Color space conversion for RGB
                r = CLAMPRGB((298*y+409*tv+128)>>8);
                g = CLAMPRGB((298*y-100*tu-208*tv+128)>>8);
                b = CLAMPRGB((298*y+516*tu+128)>>8);

                *lut++ = 0xff000000 + ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
            }
        }
    }
}

void quick_YUV422_to_RGBA(const unsigned char *src, uint32_t *dst, unsigned int width, unsigned int height) {
    const unsigned char *yuv = src;
    unsigned char u0, y0, v0, y1;

    // Loop through 4 bytes at a time
    for (unsigned int y = 0; y < height; y ++ ) {
        for (unsigned int x = 0; x < width; x +=2 ) {
            u0  = *yuv++;
            y0  = *yuv++;
            v0  = *yuv++;
            y1  = *yuv++;
            *dst++ = UYVY_to_RGBA[(y0 << 16) + (u0 << 8) + v0];
            *dst++ = UYVY_to_RGBA[(y1 << 16) + (u0 << 8) + v0];
        }
    }
}

//
//        YUV422_to_RGBA
//
// Y sampled at every pixel
// U and V sampled at every second pixel
// 2 pixels in 1 DWORD
//
//	R = Y + 1.403V
//	G = Y - 0.344U - 0.714V
//	B = Y + 1.770U
//
//	R = 298*y + 409*v + 128 >> 8
//  G = 298*y - 100*u-208*v +128 >> 8
//  B = 298*y + 516*u +128 >> 8
//
void YUV422_to_RGBA(const unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height) {

#define CLAMPRGB(t) (((t)>255)?255:(((t)<0)?0:(t)))

    const unsigned char *yuv = src;
    unsigned char *rgba = dst;
    int r1 = 0 , g1 = 0 , b1 = 0;
    int r2 = 0 , g2 = 0 , b2 = 0;
    int u0 = 0 , y0 = 0 , v0 = 0, y1 = 0;

    for (unsigned int y = 0; y <height; y ++ ) {
        for (unsigned int x = 0; x <width*2; x +=4 ) {
            u0  = (int)*yuv++;
            y0  = (int)*yuv++;
            v0  = (int)*yuv++;
            y1  = (int)*yuv++;

            u0 -= 128;
            v0 -= 128;

            // Color space conversion for RGB
            r1 = CLAMPRGB((298*y0+409*v0+128)>>8);
            g1 = CLAMPRGB((298*y0-100*u0-208*v0+128)>>8);
            b1 = CLAMPRGB((298*y0+516*u0+128)>>8);
            r2 = CLAMPRGB((298*y1+409*v0+128)>>8);
            g2 = CLAMPRGB((298*y1-100*u0-208*v0+128)>>8);
            b2 = CLAMPRGB((298*y1+516*u0+128)>>8);

            *rgba++ = (unsigned char)b1;
            *rgba++ = (unsigned char)g1;
            *rgba++ = (unsigned char)r1;
            *rgba++ = 255;
            *rgba++ = (unsigned char)b2;
            *rgba++ = (unsigned char)g2;
            *rgba++ = (unsigned char)r2;
            *rgba++ = 255;
        }
    }
}

UvcCamera::~UvcCamera() {
}

int UvcCamera::getFrame(FrameData *frame_data) {

    frame_data->index = -9999;

    /* wait for a frame to be ready, by polling */
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv;
    memset(&tv, 0, sizeof(struct timeval));
    time_t seconds = frame_timeout_ms / 1000;
    tv.tv_sec = seconds;
    tv.tv_usec = (frame_timeout_ms - 1000 * seconds) / 1000;
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    if(r == 0) {
        size_t n_bytes = snprintf(log_buffer, log_buffer_size, "stream_from_camera(): select() timeout waiting for buffer");
        if (log_fxn != 0) { (*log_fxn)(UvcCamera::LEVEL_WARNING, log_buffer, n_bytes); }
        return ERROR_FRAME_TIMEOUT;
    } else if(r == -1) {
        perror("uvc - waiting for frame");
        return ERROR_SELECT;
    }

    if(verbose && log_fxn) {
        size_t n_bytes = snprintf(log_buffer, log_buffer_size, "stream_from_camera(): about to dequeue buffer");
        if (log_fxn != 0) { (*log_fxn)(UvcCamera::LEVEL_DEBUG, log_buffer, n_bytes); }
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if(xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
        perror("uvc - retrieving frame");
        return ERROR_UVC_DQBUF;
    }

    frame_data->index = buf.index;
    frame_data->payload = capture_buffer[frame_data->index];
    frame_data->flags = buf.flags;

    if(buf.flags & V4L2_BUF_FLAG_ERROR) {
        return releaseFrame(frame_data->index);
    }

    return frame_data->index;
}

int UvcCamera::releaseFrame(int index) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        perror("uvc - queue buffer");
        return ERROR_UVC_QBUF;
    }
    return 0;
}

int UvcCamera::close() {

    for (unsigned int i=0;i<n_capture_buffers;++i) {
        size_t n_bytes = snprintf(log_buffer, log_buffer_size, "buffer %d munmap(%p, %d)", i, capture_buffer[i], capture_length[i]);
        if(log_fxn) log_fxn(LEVEL_INFO, log_buffer, n_bytes);
        munmap(capture_buffer[i], capture_length[i]);
    }
    if (capture_buffer) delete [] capture_buffer;
    if (capture_length) delete [] capture_length;

    ::close(fd);

    if (log_buffer) delete [] log_buffer;

    return 0;
}
