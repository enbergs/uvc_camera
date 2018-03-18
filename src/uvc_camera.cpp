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
#include <uvc_camera.h>
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
    /* open the device */
    fd = open(device_id, O_RDWR | O_NONBLOCK);
    if(fd == -1) { perror("Opening video device"); }

    this->width = width;
    this->height = height;

    if (log_fxn == 0) {
        this->log_fxn = &defaultLogFxn;
    }
}

int UvcCamera::defaultLogFxn(int level, const char *msg, int len) {
    int cur_time = time(0);
    printf("LOG: TIME=%d LEVEL=%d. MESSAGE=[%s]\n", cur_time, level, msg);
    return 0;
}

int UvcCamera::defaultProcessImage(UvcCamera *camera, const unsigned char *i_img, unsigned char *o_img) {
    memcpy(o_img, i_img, camera->frame_size);
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
int UvcCamera::init(int n_capture_buffers, int n_user_buffers) {

    const int kDefaultCaptureBuffers = 4;
    if (n_capture_buffers == 0) { n_capture_buffers = kDefaultCaptureBuffers; }

    const int kDefaultUserBuffers = 8;
    if (n_user_buffers == 0) { n_user_buffers = kDefaultUserBuffers; }

    this->n_capture_buffers = n_capture_buffers;
    this->n_user_buffers = n_user_buffers;

    const size_t kLogBufferSize = 256;
    log_buffer_size = kLogBufferSize;
    log_buffer = new char [kLogBufferSize];

/* set the desired parameters */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if(xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("setting pixel format");
        return -1;
    }

/* read them back out to see how we actually configured */
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
        perror("getting pixel format");
        return -1;
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
    n_bytes = snprintf(log_buffer, log_buffer_size, "initializing mmap with %d buffers", n_capture_buffers);
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
        return 1;
    }

/* check request */
    n_bytes = snprintf(log_buffer, log_buffer_size, "init_mmap(): %d buffers granted. requested = %d",
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
            perror("VIDIOC_QUERYBUF"); /* TODO what now? */
        }

        n_bytes = snprintf(log_buffer, log_buffer_size, "init_mmap(): mmap(length=%d, offset=%d)", buf.length, buf.m.offset);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);
        capture_length[i] = buf.length;
        capture_buffer[i] = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if(capture_buffer[i] == MAP_FAILED) {
            if(log_fxn) (*log_fxn)(LEVEL_FATAL, "mmap operation failed", 0); /* TODO what now? */
        }

        n_bytes = snprintf(log_buffer, log_buffer_size, "Length: %zd Address: %p", buf.length, capture_buffer[i]);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, log_buffer, n_bytes);

#if 0
        /* jsv done in start_capture() */
		snprintf(logbuff, logbuff_length, "init_mmap(): VIDIOC_QBUF %d", i);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
			perror("Queue Buffer");
			return 1;
		}
#endif

    }

    if(log_fxn) (*log_fxn)(LEVEL_INFO, "mmap initialization complete", 0);

/*** memory mapping complete ***/

    cpu_set_t cpu_set;
    int i, err, time0;
    pthread_t thread;
    unsigned int cpu_mask;
    frame_size = width * height * 4; /* accommodate BGRA */

/* configure incoming = camera streamer */

    user_buffer_length = new size_t [n_user_buffers];
    user_buffer_status = new unsigned int [n_user_buffers];
    user_buffer = new unsigned char * [n_user_buffers];
    for (int i = 0; i < n_user_buffers; ++i) {
        user_buffer[i] = new unsigned char [frame_size];
        user_buffer_status[i] = CAMERA_STATE_IDLE;
        user_buffer_length[i] = frame_size;
    }

//    new unsigned char * [ n_incoming_buffers ];
//    incoming_buffer_semaphore = new unsigned int [ n_incoming_buffers ];
//    incoming_buffer_length = new int [ n_incoming_buffers ];
//    incoming_buffer_size = new int [ n_incoming_buffers ];
//    frame_capture_timestamp = new uint64_t [ n_incoming_buffers ];
//    frame_capture_index = new int [ n_incoming_buffers ];

//    if(isize == 0) {
//        isize = width * height * bytes_per_pixel; /* jsv. actually need to sync this with driver buffer size */
//        isize = width * height * 4; /* jsv. this needs to accomodate BGR */
//    }

//    for(int i=0;i<n_incoming_buffers;++i) {
//        incoming_buffer[i] = new unsigned char [ isize ];
//        incoming_buffer_semaphore[i] = BUFFER_EMPTY;
//        incoming_buffer_length[i] = 0;
//        incoming_buffer_size[i] = isize;
//    }

#if 0

/* camera streaming thread always needs to run, even if to /dev/null. it recycles buffers */
    camera_streamer_params.camera = this;
    camera_streamer_params.buffers = incoming_buffer;
    camera_streamer_params.bufflen = incoming_buffer_length;
    camera_streamer_params.semaphore = incoming_buffer_semaphore;
    camera_streamer_params.frame_capture_timestamp = frame_capture_timestamp;
    camera_streamer_params.frame_capture_index = frame_capture_index;
    camera_streamer_params.n_buffers = n_incoming_buffers;
    camera_streamer_params.thread_started = 0;
    camera_streamer_params.run = 1;
    camera_streamer_params.fd = fd;
    camera_streamer_params.capture_buffer = capture_buffer;
    camera_streamer_params.logbuff = camera_streamer_logbuff;
    camera_streamer_params.logbuff_length = camera_streamer_logbuff_length;
    camera_streamer_params.cpu_mask = incoming_streamer_cpu_mask;
    camera_streamer_params.device_status = &device_status;
    err = pthread_create(&tid[FromCameraThreadId], NULL, &stream_from_camera, (void *)&camera_streamer_params);
    thread = tid[FromCameraThreadId];
    cpu_mask = incoming_streamer_cpu_mask;
    if(err) {
        printf("unable to create new thread for camera streaming\n");
        return false;
    }

    if(cpu_mask) {
        CPU_ZERO(&cpu_set);
        for(i=0;i<32;++i) {
            if(cpu_mask & (1 << i)) CPU_SET(i, &cpu_set);
        }
        err = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpu_set);
        if(err) {
            snprintf(logbuff, logbuff_length, "unable to set thread affinity for camera streaming");
            if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
        }
        err = pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpu_set);
        if(err) {
            snprintf(logbuff, logbuff_length, "unable to get thread affinity for camera streaming");
            if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
        } else {
            for(i=0;i<CPU_SETSIZE;++i) {
                if(CPU_ISSET(i, &cpu_set)) {
                    snprintf(logbuff, logbuff_length, "thread affinity for camera streaming = %d", i);
                    if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
                }
            }
        }
    }

    time0 = time(NULL);
    while(camera_streamer_params.thread_started == 0) {
        int dtime = time(NULL) - time0;
        if(dtime > 5) {
            snprintf(logbuff, logbuff_length,
                "timeout waiting for camera streaming thread to initialize (%d seconds)...", dtime);
            if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
            return false;
        }
        snprintf(logbuff, logbuff_length, "waiting for camera streaming thread to initialize (%d seconds)...", dtime);
        if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
        usleep(1000000);
    }
    snprintf(logbuff, logbuff_length, "camera streaming thread successfully initialized");
    if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

/* do we want to record video? */
    writer = 0;

#endif

#if 0
    if(ofile && (raw_output == false)) {
		char ch0 = fourcc[0];
		char ch1 = fourcc[1];
		char ch2 = fourcc[2];
		char ch3 = fourcc[3];
		writer = new VideoWriter(ofile, CV_FOURCC(ch0, ch1, ch2, ch3), fps, cvSize(width, height), is_color);
		printf("output file = [%s] with FOURCC = [%s]\n", ofile, fourcc);
	}
#endif

#if 0
    /* jsv is this still needed? */
	if(display) {
		cvNamedWindow(display_window_name.c_str());
		display_params.wait = 10;
		display_params.window_name = display_window_name.c_str();
		register_callback(display_fxn, &display_params);
	}
#endif

    return true;

}

UvcCamera::~UvcCamera() {

    for(int i=0;i<n_capture_buffers;++i) {
        size_t n_bytes = snprintf(log_buffer, log_buffer_size, "buffer %d munmap(%p, %d)",
            i, capture_buffer[i], capture_length[i]);
        if(log_fxn) log_fxn(LEVEL_INFO, log_buffer, n_bytes);
        munmap(capture_buffer[i], capture_length[i]);
    }

    ::close(fd);
    size_t n_bytes = snprintf(log_buffer, log_buffer_size, "close(fd=%d)", fd);
    if(log_fxn) log_fxn(LEVEL_INFO, log_buffer, n_bytes);
    if(capture_buffer) delete [] capture_buffer;
    if(capture_length) delete [] capture_length;

    delete [] log_buffer;
    for (int i = 0; i < n_user_buffers; ++i) { delete [] user_buffer[i]; }
    delete [] user_buffer;
    delete [] user_buffer_length;
    delete [] user_buffer_status;
}

