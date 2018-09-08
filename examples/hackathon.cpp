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

using namespace cv;
using namespace std;

/* default settings */

/* function prototypes */

unsigned int width = 1280, height = 960, frame_index = 0, fps = 30;

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
	int *run;
	UvcCamera *camera;
	unsigned int frame_data_size;
	unsigned int frame_data_head;
	unsigned int frame_data_tail;
	unsigned int frame_data_mask;
	UvcCamera::FrameData **frame_data_pool;
	unsigned int y_data_size;
	unsigned int y_data_head;
	unsigned int y_data_tail;
	unsigned int y_data_mask;
	uint8_t **y_data_pool;
} ThreadParams;

typedef struct {
    std::vector<cv::Point2f> features;
    cv::Mat mat;
} FrameData;

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
		else if(strcmp(argv[i], "-mjpg") == 0) fourcc = "MJPG";
		else if(strcmp(argv[i], "-jpg") == 0) {
			compression_scheme = UvcCamera::COMPRESSION_JPEG;
			compression_quality = atoi(argv[++i]);
		}
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
	UvcCamera::FrameData frame_data;
	uint32_t *bgr_data = new uint32_t [camera->width * camera->height];
	uint8_t *y_data = new uint8_t [camera->width * camera->height];

	/* sync to camera */
	width = camera->width;
	height = camera->height;
	std::string window_name("main");
	initialize_UYVY_to_RGBA();

	while (run) {
		uint8_t *src;
		const time_t frame_timeout_ms = 1000;
		int uvc_frame_index = camera->getFrame(&frame_data);
		if (uvc_frame_index >= 0) {
			quick_YUV422_to_RGBA(frame_data.payload, bgr_data, width, height);
			yuv422_to_y(frame_data.payload, y_data, width, height);
			// YUV422_to_RGBA(frame_data.payload, (uint8_t *) bgr_frame, width, height); /* TODO this also works but quick version is ... */
			// Mat bgr_frame(height, width, CV_8UC4, bgr_frame);
			// imshow(window_name, bgr_frame);
			Mat y_frame(height, width, CV_8UC1, y_data);
			imshow(window_name, y_frame);
			waitKey(30);
			camera->releaseFrame(frame_data.index);
		} else if (uvc_frame_index < 0) {
			printf("error\n");
		}
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
	thread_params->frame_data_pool = new UvcCamera::FrameData * [thread_params->frame_data_size];
	for (unsigned int i = 0; i < thread_params->frame_data_size; ++i) {
		thread_params->frame_data_pool[i] = new UvcCamera::FrameData;
		thread_params->frame_data_pool[i]->payload = new uint8_t [2 * n_pixels]; /* 2 bytes per pixel in yuv420 */
	}

	thread_params->y_data_size = 8;
	thread_params->y_data_head = 0;
	thread_params->y_data_tail = 0;
	thread_params->y_data_pool = new uint8_t * [thread_params->y_data_size];
	for (unsigned int i = 0; i < thread_params->y_data_size; ++i) {
		thread_params->y_data_pool[i] = new uint8_t [n_pixels];
	}

	while (*thread_params->run != 0) {
		int uvc_frame_index = camera->getFrame(thread_params->frame_data_pool[thread_params->frame_data_head]);
		thread_params->frame_data_head = (thread_params->frame_data_head + 1) & thread_params->frame_data_mask;
		vector<uchar> status;
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
	}

	/* clean up; free resources */

	for (unsigned int i = 0; i < thread_params->frame_data_size; ++i) {
		delete [] thread_params->frame_data_pool[i]->payload;
		delete [] thread_params->frame_data_pool[i];
	}
	delete [] thread_params->frame_data_pool;

	for (unsigned int i = 0; i < thread_params->y_data_size; ++i) {
		delete [] thread_params->y_data_pool[i];
	}
	delete [] thread_params->y_data_pool;

}

void *analysis_looper(void *ext) {
	ThreadParams *thread_params = (ThreadParams *) ext;
}