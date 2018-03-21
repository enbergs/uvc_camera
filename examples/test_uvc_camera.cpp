/* TODO jsv
	(a) sync the resolution requested with resolution granted by the camera
	(b) Anchor structure information in the output stream
	(c) currently, raw streaming is performed in a separate thread. OpenCV compression is performed on-the-fly
	(d) if not using *autoexposure*, figure out what a good default setting is. look at *coarse_integration*
	(e) the whole VideoWriter component needs to be redesigned
 */
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
#define DEFAULT_SLEEP_WAIT 10000

/* function prototypes */
int analyze_dynamic_range(unsigned short int *frame_buffer, int width, int height, double epsilon);
int utc_time(char *result, int result_size);

unsigned int width = 1280, height = 960, frame_index = 0, fps = 30;

static bool debug_mode = false;
bool run = true;

/* when we hit ^C tell main loop to exit */
static void stop (int sig) { run = false; }

#if 0

#define GAMMA_LUT_SIZE 4096
typedef struct {
	int height, width, wait, exposure;
	const char *window_name;
	unsigned char *work_buffer;
	unsigned char gamma_lut[GAMMA_LUT_SIZE]; /* 12 bits LUT for gamma correction */
} DisplayParams;

int exposure_setting = 500;
bool display_fxn(Camera *camera, unsigned char *data_ptr, void *ext) {
	DisplayParams *params = (DisplayParams *)ext;
	Mat frame;
	int height = camera->get_height(), width = camera->get_width(), type = camera->get_type();
	if(type == Camera::CAMERA_LI_USB3_MT9M021M) {
		frame = Mat(height, width, CV_16UC1, (void *)data_ptr);
	} else if(type == Camera::CAMERA_LI_USB3_MT9M021C) {
		// frame = Mat(height / 2, width / 2, CV_16UC3, (void *)data_ptr);
		frame = Mat(height, width, CV_8UC3, (void *)data_ptr);
	}
	imshow(params->window_name, frame);
	int key = cvWaitKey(10);
	if(key == '+') {
		unsigned int current;
		camera->read_m021_register(0x3012, &current);
		printf("exposure_setting value = %d\n", current);
		++exposure_setting;
		if(exposure_setting > 3000) exposure_setting = 3000;
		camera->write_m021_register(0x3012, exposure_setting);
	} else if(key == '-') {
		unsigned int current;
		camera->read_m021_register(0x3012, &current);
		printf("exposure_setting value = %d\n", current);
		--exposure_setting;
		if(exposure_setting < 3) exposure_setting = 3;
		camera->write_m021_register(0x3012, exposure_setting);
	}
}

#endif

int logger(int level, const char *str, int len) {
	int cur_time = time(0);
	printf("LOG: TIME=%d LEVEL=%d. MESSAGE=[%s]\n", cur_time, level, str);
	return 0;
}

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
	camera->init();
	camera->frame_timeout_ms = 1000;
	UvcCamera::FrameData frame_data;
	uint32_t *bgr_frame = new uint32_t [camera->width * camera->height];

	/* sync to camera */
	width = camera->width;
	height = camera->height;
	std::string window_name("main");

	while (run) {
		uint8_t *src;
		const time_t frame_timeout_ms = 1000;
		int uvc_frame_index = camera->getFrame(&frame_data);
		if (uvc_frame_index >= 0) {
			uyvy8ToBgr(frame_data.payload, bgr_frame, width, height);
			Mat frame(height, width, CV_8UC4, bgr_frame);
			imshow(window_name, frame);
			waitKey(30);
			camera->releaseFrame(frame_data.index);
		} else if (uvc_frame_index < 0) {
			printf("error\n");
		}
	}

	// this is also possible: camera->log_fxn = logger;

#if 0
	if(cfile.length()) {
		if(verbose) printf("calibration file = [%s]\n", cfile.c_str());
		FileStorage fs(cfile.c_str(), FileStorage::READ);
		fs["Camera_Matrix"] >> camera_matrix;
		fs["Distortion_Coefficients"] >> dist_coeffs;
		if(verbose) printf("camera matrix = \n");
		std::cout << camera_matrix;
		std::cout << dist_coeffs;
		calibrate = true;
	}
	if(verbose) printf("allocating buffers for %dX%d resolution\n", width, height);
	signal(SIGINT, stop); /* ^C  exception handling */ 

/* any compression */
	if(compression_scheme != COMPRESSION_NONE) {
		std::vector<int> params = std::vector<int>(1);
		params[0] = compression_quality;
		camera->set_compression(COMPRESSION_JPEG, params);
		camera->set_fourcc(fourcc.c_str());
		snprintf(logbuff, logbuff_length, "camera configured for compression scheme %d with quality %d",
			compression_scheme, compression_quality);
		logger(LEVEL_INFO, logbuff);
	}

/* output streaming */
	if(ofile.length()) camera->set_output_file(ofile.c_str());
	int osize = 0; /* automatic determination of output buffer size */
	camera->configure_output_streaming(n_output_buffers, osize);
//	camera->set_display(display);
	camera->set_verbose(verbose);
	camera->init();

	DisplayParams *display_params = 0; 
	if(display) {
		display_params = new DisplayParams;
		display_params->window_name = "main";
		camera->register_callback(display_fxn, display_params);
	}

/* initialize/start the capture session */
	if(camera->start_capture() == false) {
		snprintf(logbuff, logbuff_length, "unable to start capture");
		logger(LEVEL_INFO, logbuff);
		return 1;
	}

	int dtime, time0 = time(0);

	snprintf(logbuff, logbuff_length, "starting capture session at %d", time0);
	logger(LEVEL_INFO, logbuff);

	std::vector<uchar> compression_data(1024*1024); /* preallocate a large vector for compressed output, if any */

	while(run) {

		void *incoming_buffer = camera->get_incoming_buffer(true);
		while(incoming_buffer == 0) {
			usleep(10000);
			incoming_buffer = camera->get_incoming_buffer();
		}
if(debug_mode) printf("main loop: got incoming buffer [%p]\n", incoming_buffer);
		
		void *outgoing_buffer = camera->get_outgoing_buffer();
		while(outgoing_buffer == 0) {
			usleep(10000); /* we shouldn't have to wait here if things go well */
			outgoing_buffer = camera->get_outgoing_buffer();
		}
if(debug_mode) printf("main loop: got outgoing buffer [%p]\n", outgoing_buffer);

		int isize = camera->get_incoming_buffer_length();
		int osize = camera->get_outgoing_buffer_size(); 

		int fsize = isize;
		compression_data.clear();
		if(compression_scheme != Camera::COMPRESSION_NONE) {
			camera->compress_frame((unsigned char *)incoming_buffer, compression_data); 
			fsize = compression_data.size();
		}

		int nwrite = 0;
#if 0
		int nwrite = sizeof(FrameInfo) + fsize;

		if(nwrite <= osize) { /* the frame will fit into the outgoing buffer. write out video data */
			unsigned char *p0 = (unsigned char *)outgoing_buffer;
			FrameInfo *pi = (FrameInfo *)p0;
			pi->preamble.structure_type = FRAME_INFO;
			pi->preamble.structure_length = sizeof(FrameInfo);
			pi->preamble.payload_length = fsize; 
			pi->preamble.time_stamp = 0xaaaa5555; /* jsv. let's fix this */
			pi->channel = device;
			pi->height = height;
			pi->width = width;
			pi->index = frame_index;
			unsigned char *dst = (unsigned char *)&pi[1];
			if(fsize) {
				std::vector<uchar>::const_iterator it, last = compression_data.end();
				for(it=compression_data.begin();it!=last;++it) { *dst++ = *it; }
			} else {
				memcpy(dst, incoming_buffer, isize);
			}
		}
#endif

//		memcpy(outgoing_buffer, incoming_buffer, nwrite);
		// camera->release_outgoing_buffer(nwrite);
		// camera->release_incoming_buffer();
		
		++nframes;
		dtime = time(0) - time0;
		if(nframes && ((nframes % 300) == 0) && (dtime > 0)) {
			snprintf(logbuff, logbuff_length, "%d frames in %d seconds. fps = %d", nframes, dtime, nframes / dtime);
			logger(LEVEL_INFO, logbuff);
		}
	}

	dtime = time(0) - time0;
	double fps = (double)nframes;
	fps = fps / dtime;
	snprintf(logbuff, logbuff_length, "%f frames per second", fps);
	logger(LEVEL_INFO, logbuff);

	if(camera->stop_capture()) return 1;

	if(display_params) delete display_params;

#endif

	return 0;
}


