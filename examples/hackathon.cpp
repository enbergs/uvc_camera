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
#include <signal.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <uvc_camera.h>

#include "vo_features.h"

using namespace cv;
using namespace std;

/* current issues */

/* TODO list
 * a) previous image/frame always has the same number of features
 * b) only good frames (more than X features) stay in the history channel
 */

/* default settings */

/* function prototypes */
void *camera_looper(void *ext);
void *analysis_looper(void *ext);
void *visualization_looper(void *ext);

constexpr uint32_t InvalidCameraFrameIndex = 0xffffffff;
constexpr size_t MaximumTypicalFeatures = 10000; /* TODO: the number of features we expect to see in typical image */
unsigned int width = 1280, height = 960, frame_index = 0, fps = 30;
constexpr unsigned int KeepHistory = 4; /* how many frames back we should keep */

static bool debug_mode = false;
bool run = true;

/* when we hit ^C tell main loop to exit */
static void stop (int sig) { run = false; }

int logger(int level, const char *str, int len) {
	int cur_time = time(0);
	printf("LOG: TIME=%d LEVEL=%d. MESSAGE=[%s]\n", cur_time, level, str);
	return 0;
}

typedef struct {
    std::vector<cv::Point2f> *features;
    cv::Mat *mat;
    UvcCamera::FrameData *frame_data; /* actually needed? TODO */
    uint8_t *y_data;
} FrameData;

typedef struct {
	bool *run;
	UvcCamera *camera;
	unsigned int frame_data_size;
	unsigned int frame_data_head;
	unsigned int frame_data_tail;
	unsigned int frame_data_mask;
	FrameData **frame_data_pool;
	unsigned int display_index;
	bool display_busy;
} ThreadParams;

// TODO - where to parse from calib.txt
double kFocalLengthPX = 718.8560;
cv::Point2d kPrinciplePointPX(607.1928, 185.2157);

int main(int argc, char **argv) {

	char str[1024];
	char logbuff[1024];
	int logbuff_length = sizeof(logbuff);
	int i, nframes = 0, device = -1, compression_scheme = UvcCamera::COMPRESSION_NONE, compression_quality = 0;
	std::string cfile, ofile, gfile = "gps.log", fourcc = "";
	bool calibrate = false, display = false, verbose = false;
	Mat camera_matrix, dist_coeffs;

    width = 1920; height = 1080;
	width = 1280; height = 960;

/* jsv need to implement */
	int n_output_buffers = 3;
    fps = 30;
    device = 0;

	for(i=1;i<argc;++i) {
		if(strcmp(argv[i], "-debug") == 0) debug_mode = true;
		else if(strcmp(argv[i], "-verbose") == 0) verbose = true;
		else if(strcmp(argv[i], "--device") == 0) device = atoi(argv[++i]);
		else if(strcmp(argv[i], "-d") == 0) device = atoi(argv[++i]);
		else if(strcmp(argv[i], "--height") == 0) height = atoi(argv[++i]);
		else if(strcmp(argv[i], "-h") == 0) height = atoi(argv[++i]);
		else if(strcmp(argv[i], "--width") == 0) width = atoi(argv[++i]);
		else if(strcmp(argv[i], "-w") == 0) width = atoi(argv[++i]);
		else if(strcmp(argv[i], "-c") == 0) cfile = argv[++i];
		else if(strcmp(argv[i], "--calibrate") == 0) cfile = argv[++i];
		else if(strcmp(argv[i], "--fourcc") == 0) fourcc = argv[++i];
		else if(strcmp(argv[i], "-o") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "--output") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "-display") == 0) display = true; 
//		else if(strcmp(argv[i], "-output_buffers") == 0) n_output_buffers = atoi(argv[++i]);
		else if(strcmp(argv[i], "-fps") == 0) fps = atoi(argv[++i]);
		else if(strcmp(argv[i], "-vga") == 0) {
			width = 640;
			height = 480;
		} else if(strcmp(argv[i], "-vga2") == 0) {
			width = 1280;
			height = 960;
		} else if(strcmp(argv[i], "-hd") == 0) {
			width = 1920;
			height = 1080;
		}
	}

	UvcCamera *camera = new UvcCamera(device, width, height, logger);
	camera->open();
	camera->frame_timeout_ms = 1000;
//	UvcCamera::FrameData frame_data;
//	uint32_t *bgr_data = new uint32_t [camera->width * camera->height];
//	uint8_t *y_data = new uint8_t [camera->width * camera->height];

	ThreadParams *thread_params = new ThreadParams;
	thread_params->camera = camera;

	/* sync to camera */
	width = camera->width;
	height = camera->height;
	std::string window_name("main");
	initialize_UYVY_to_RGBA();

	int err;
	uint32_t cpu_mask = 1;
	pthread_t camera_thread_id, analysis_thread_id, visualization_thread_id;
    cpu_set_t cpu_set;

    thread_params->run = &run;

    err = pthread_create(&camera_thread_id, NULL, camera_looper, thread_params); /* create thread */
    cpu_mask = 1;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(camera_thread_id, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&analysis_thread_id, NULL, analysis_looper, thread_params); /* create thread */
    cpu_mask = 2;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(analysis_thread_id, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&visualization_thread_id, NULL, visualization_looper, thread_params); /* create thread */
    cpu_mask = 4;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(visualization_thread_id, sizeof(cpu_set_t), &cpu_set);

    while (run) {
        usleep(100000);
//		uint8_t *src;
//		const time_t frame_timeout_ms = 1000;
//		int uvc_frame_index = camera->getFrame(&frame_data);
//		if (uvc_frame_index >= 0) {
//			quick_YUV422_to_RGBA(frame_data.payload, bgr_data, width, height);
//			yuv422_to_y(frame_data.payload, y_data, width, height);
//			// YUV422_to_RGBA(frame_data.payload, (uint8_t *) bgr_frame, width, height); /* TODO this also works but quick version is ... */
//			// Mat bgr_frame(height, width, CV_8UC4, bgr_frame);
//			// imshow(window_name, bgr_frame);
//			Mat y_frame(height, width, CV_8UC1, y_data);
//			imshow(window_name, y_frame);
//			waitKey(30);
//			camera->releaseFrame(frame_data.index);
//		} else if (uvc_frame_index < 0) {
//			printf("error\n");
//		}
	}

	camera->close();
	delete camera;

	return 0;
}

void *camera_looper(void *ext) {
	ThreadParams *thread_params = (ThreadParams *) ext;
	UvcCamera *camera = thread_params->camera;
	unsigned long int n_pixels = camera->width * camera->height;

	/* allocate what all you need */
	thread_params->frame_data_size = 8;
	thread_params->frame_data_mask = thread_params->frame_data_size - 1;
	thread_params->frame_data_head = 0;
	thread_params->frame_data_tail = 0;
	thread_params->frame_data_pool = new FrameData * [thread_params->frame_data_size];
	for (unsigned int i = 0; i < thread_params->frame_data_size; ++i) {
		thread_params->frame_data_pool[i] = new FrameData;
        FrameData *frame_data = thread_params->frame_data_pool[i];
		frame_data->frame_data = new UvcCamera::FrameData;
        frame_data->frame_data->payload = new uint8_t [2 * n_pixels]; /* 2 bytes per pixel in yuyv */
        frame_data->frame_data->index = InvalidCameraFrameIndex;
        frame_data->y_data = new uint8_t [n_pixels];
        frame_data->mat = new cv::Mat(camera->height, camera->width, CV_8UC1, frame_data->y_data);
        frame_data->features = new std::vector<cv::Point2f>(MaximumTypicalFeatures);
	}

	while (*thread_params->run != 0) {

        /* do not potentially overwrite currently accessed data. need to keep a few frames of history in queue */
        unsigned int instantaneous_tail = thread_params->frame_data_tail; /* tail is subject to change in other thread */
        unsigned int instantaneous_head = thread_params->frame_data_head;
	    for (unsigned i = 0; i < KeepHistory; ++i) {
	        unsigned int stay_away = (instantaneous_tail - i) & thread_params->frame_data_mask;
	        if (instantaneous_head == stay_away) {
	            usleep(1000);
                continue;
	        }
	    }

	    /* in normal operation, display frame is in list of history frames, and next check is superfluous. but, to be safe... */
	    if (thread_params->display_busy && (instantaneous_head == thread_params->display_index)) { /* do not overwrite data in display */
            usleep(1000);
            continue;
	    }

	    FrameData *frame_data = thread_params->frame_data_pool[thread_params->frame_data_head];
		int uvc_frame_index = camera->getFrame(frame_data->frame_data); /* grab data */
        if (uvc_frame_index >= 0) {
            yuv422_to_y(frame_data->frame_data->payload, frame_data->y_data, width, height); /* convert to grey scale */
            camera->releaseFrame(uvc_frame_index); /* let camera buffers go back into pool */
        } else if (uvc_frame_index < 0) {
            printf("TODO error\n");
        }

		/* wrap up loop */
        thread_params->frame_data_head = (thread_params->frame_data_head + 1) & thread_params->frame_data_mask;
	}

	/* clean up; free resources */

//	for (unsigned int i = 0; i < thread_params->frame_data_size; ++i) {
//		delete [] thread_params->frame_data_pool[i]->payload;
//		delete [] thread_params->frame_data_pool[i];
//	}
//	delete [] thread_params->frame_data_pool;
//
//	for (unsigned int i = 0; i < thread_params->frame_data_size; ++i) {
//		delete [] thread_params->y_data_pool[i];
//	}
//	delete [] thread_params->y_data_pool;

    return NULL;
}

void *analysis_looper(void *ext) {
	ThreadParams *thread_params = (ThreadParams *) ext;
	unsigned int n_frames = 0;
	std::vector<uchar> status;
    while (*thread_params->run != 0) {
        unsigned int room = (thread_params->frame_data_head - thread_params->frame_data_tail) % thread_params->frame_data_mask;
        // if (room < KeepHistory) { usleep(1000); continue; }
        if (room < 1) { usleep(1000); continue; }

        FrameData *frame_data = thread_params->frame_data_pool[thread_params->frame_data_tail];
        const cv::Mat &current_image = *frame_data->mat;
        // TODO necessary? frame_data->features->clear(); /* reset vector */
        featureDetection(current_image, *frame_data->features);

        ++n_frames;

        /* fetch previous frame */
        unsigned int previous_index = (thread_params->frame_data_tail - 1) & thread_params->frame_data_mask;
        FrameData *prev_frame_data = thread_params->frame_data_pool[previous_index];
        const cv::Mat &previous_image = *prev_frame_data->mat;

		size_t n_previous_features = prev_frame_data->features->size(), n_current_features = frame_data->features->size();
		printf("numbers of features = %zu(%d) / %zu(%d)\n", n_previous_features, previous_index, n_current_features, thread_params->frame_data_tail);

		if (n_frames > 1) { /* we need at least two frames to get the pipeline working */
            cv::Mat mask, R, t;
            int minimum_feature_set_size = 10;
            printf("feature set sizes = %zu/%zu\n", prev_frame_data->features->size(), frame_data->features->size());
            if (frame_data->features->size() >= minimum_feature_set_size) {
                featureTracking(previous_image, current_image, *prev_frame_data->features, *frame_data->features, status);
                cv::Mat E = findEssentialMat(*prev_frame_data->features, *frame_data->features, kFocalLengthPX, kPrinciplePointPX, cv::RANSAC, 0.999, 1.0, mask);
                recoverPose(E, *prev_frame_data->features, *frame_data->features, R, t, kFocalLengthPX, kPrinciplePointPX, mask);
            }
			size_t n_prev_features = prev_frame_data->features->size(), n_curr_features = frame_data->features->size(); /* TODO check again */
			printf("check numbers of features = %zu / %zu\n", n_prev_features, n_curr_features);
        } else if (n_frames == 0) {
			featureDetection(current_image, *frame_data->features);
		}

        if (thread_params->display_busy == false) {
            thread_params->display_index = thread_params->frame_data_tail;
            thread_params->display_busy = true;
        }

        thread_params->frame_data_tail = (thread_params->frame_data_tail + 1) & thread_params->frame_data_mask; /* wrap up loop */
    }
    return NULL;
}

void *visualization_looper(void *ext) {
    ThreadParams *thread_params = (ThreadParams *) ext;
    thread_params->display_busy = false;
    while (*thread_params->run != 0) {
        if (thread_params->display_busy == true) {
            FrameData *frame_data = thread_params->frame_data_pool[thread_params->display_index];
            const cv::Mat image = *frame_data->mat;
            imshow("main", image);
            cv::waitKey(30);
            thread_params->display_busy = false;
        }
    }
    return NULL;
}