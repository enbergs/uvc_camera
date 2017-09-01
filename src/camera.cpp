// #define VERBOSE_DEBUG 1
/* TODO jsv
	(a) sync the resolution requested with resolution granted by the camera
	(b) currently, raw streaming is performed in a separate thread. OpenCV compression is performed on-the-fly
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
#include <pthread.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <inttypes.h>

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <camera.h>

using namespace cv;
using namespace std;

/* Versions */
#define FWVERSION "0.0"
#define GITHASH "hasheesh"
static char version[] = FWVERSION;
static char githash[] = GITHASH;

/* default settings */
#define DISK_STREAMER_DEFAULT_SLEEP_WAIT 100000
#define INCOMING_BUFFER_DEFAULT_SLEEP_WAIT 100000
#define DEFAULT_JPEG_COMPRESSION_QUALITY 95

/* function prototypes */
int utc_time(char *result, int result_size);
void bayer_to_rgb24(uint8_t *pBay, uint8_t *pRGB24, int width, int height, int pix_order);
void bayer16_to_rgb888(unsigned short int *bayer, unsigned char *bgr, int width, int height, int shift, bool start_with_green = true, bool blue_line = false);
void *stream_to_disk(void *DiskStreamerParams);
void *stream_from_camera(void *CameraStreamerParams);
static int xioctl(int fd, int request, void *arg);

bool Camera::set_output_file(const char *filename) {
	int len = strlen(filename);
	ofile = new char [ len + 1 ];
	sprintf(ofile, "%s", filename);
}

bool Camera::configure_output_streaming(int n_buffers, int size, unsigned int cpu_mask) {
	outgoing_streamer_cpu_mask = cpu_mask;
	this->n_outgoing_buffers = n_buffers;
	this->osize = size;
	return true;
}

bool Camera::configure_input_streaming(int n_buffers, int size, unsigned int cpu_mask) {
	incoming_streamer_cpu_mask = cpu_mask;
	this->n_incoming_buffers = n_buffers;
	this->isize = size;
	return true;
}

bool Camera::register_callback(CameraCallbackFxn *fxn, void *ext) {
	CameraCallbackParams params;
	params.fxn = fxn;
	params.ext = ext;
	callback_params.push_back(params);
};

bool Camera::set_compression(int scheme, std::vector<int> const &params) {
	if(scheme == COMPRESSION_JPEG && params.size() >= 1) {
		compression_scheme = scheme;
		compression_quality = params[0];
		return true;
	}
	return false;
};

/* at this point: height, width and pixel size are known.
   init() initializes all the buffers and camera parameters, but does not "start" the streaming.
   the user must call start_capture() and stop_capture() to start and stop streaming from camera
   the complement of this function is close() which should be called when the camera will no
   longer be used */
bool Camera::init() {

/* set the desired parameters */
	struct v4l2_format fmt;
	memset(&fmt, 0, sizeof(struct v4l2_format));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;

	if(xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
		snprintf(logbuff, logbuff_length, "error setting pixel format");
		if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
		perror("Setting Pixel Format");
		return false;
	}

/* read them back out to see how we actually configured */
	memset(&fmt, 0, sizeof(struct v4l2_format));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(xioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
		snprintf(logbuff, logbuff_length, "error getting pixel format");
		if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
		perror("Getting Pixel Format");
		return false;
	}

	if((width != fmt.fmt.pix.width) || (height != fmt.fmt.pix.height)) {
		snprintf(logbuff, logbuff_length,
			"start_capture(): requested WxH = %dx%d. granted %dx%d",
			width, height, fmt.fmt.pix.width, fmt.fmt.pix.height);
		if(log_fxn) (*log_fxn)(LEVEL_WARNING, logbuff);
	} else {
		snprintf(logbuff, logbuff_length, "start_capture(): WxH = %dx%d", width, height);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	}

	if(verbose) { /* jsv. need to be able to set pixel format above. right now, we are dictated format */ 
		char fourcc[5];
		strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
		snprintf(logbuff, logbuff_length, "Selected Camera Mode:\nWidth: %d Height: %d\nPixFmt: %s\nField: %d",
			fmt.fmt.pix.width, fmt.fmt.pix.height, fourcc, fmt.fmt.pix.field);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	}

/* from here, height and width are sync'ed to the camera */
	height = fmt.fmt.pix.height;
	width = fmt.fmt.pix.width; 
	switch(type) {
	case CAMERA_LI_USB3_MT9M021C:
	case CAMERA_LI_USB3_MT9M021M: 
	case CAMERA_LI_USB3_OV10635:
    case CAMERA_LI_USB3_AR023Z:
		bytes_per_pixel = 2;
		frame_size = height * width * bytes_per_pixel; /* number of bytes */
		break;
	default:
		break;
	}

/*** mmap ***/
	snprintf(logbuff, logbuff_length, "initializing mmap with %d buffers", n_capture_buffers);
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

/* request buffers */
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(struct v4l2_requestbuffers));
	req.count = n_capture_buffers;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if(xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
		perror("Requesting Buffer");
		return 1;
	}

/* check request */
	snprintf(logbuff, logbuff_length, "init_mmap(): %d buffers granted. requested = %d", 
		req.count, n_capture_buffers);
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

	if(req.count != n_capture_buffers) {
		snprintf(logbuff, logbuff_length, 
			"driver granted %d of %d requested buffers",
			req.count, n_capture_buffers);
		if(log_fxn) (*log_fxn)(LEVEL_WARNING, logbuff);
		n_capture_buffers = req.count; /* adjust */
	}

	capture_buffer = new unsigned char * [ n_capture_buffers ];
	capture_length = new int [ n_capture_buffers ];

	if(req.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		snprintf(logbuff, logbuff_length, 
			"WARNING: driver changed TYPE of requested buffer (requested: %d granted: %d)",
			V4L2_BUF_TYPE_VIDEO_CAPTURE, req.type);
		if(log_fxn) (*log_fxn)(LEVEL_WARNING, logbuff);
	}

	struct v4l2_buffer buf;
	for(unsigned int i=0;i<n_capture_buffers;++i) {

		snprintf(logbuff, logbuff_length, "init_mmap(): VIDIOC_QUERYBUF %d", i);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if(xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
			perror("Querying Buffer");
			return false;
		}

		snprintf(logbuff, logbuff_length, "init_mmap(): mmap(length=%d, offset=%d)", buf.length, buf.m.offset);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		capture_length[i] = buf.length;
		if(frame_size > capture_length[i]) { /* we have a problem. driver not allocating enough space for frame */
			snprintf(logbuff, logbuff_length, "driver allocated insufficient memory for frames");
			if(log_fxn) (*log_fxn)(LEVEL_FATAL, logbuff);
		}
		capture_buffer[i] = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
		if(capture_buffer[i] == MAP_FAILED) {
			snprintf(logbuff, logbuff_length, "mmap() operation failed");
			if(log_fxn) (*log_fxn)(LEVEL_FATAL, logbuff);
		}
		snprintf(logbuff, logbuff_length, "Length: %d Address: %p", buf.length, capture_buffer[i]);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

		if(buf.length != frame_size) {
			snprintf(logbuff, logbuff_length, "WARNING: frame size = %d. buffer length for mmap = %d",
				frame_size, buf.length);
			if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		}

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

	snprintf(logbuff, logbuff_length, "mmap initialization complete");
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

/*** memory mapping complete ***/

/* configure disk streamer = outgoing */

	outgoing_buffer = new unsigned char * [ n_outgoing_buffers ];
	outgoing_buffer_semaphore = new unsigned int [ n_outgoing_buffers ];
	outgoing_buffer_length = new int [ n_outgoing_buffers ];
	outgoing_buffer_size = new int [ n_outgoing_buffers ];

	if(osize == 0) {
		osize = width * height * bytes_per_pixel; /* jsv. actually need to sync this with driver buffer size */ 
		osize = width * height * 4; /* jsv. this needs to accomodate BGR */ 
	}

	for(int i=0;i<n_outgoing_buffers;++i) {
		outgoing_buffer[i] = new unsigned char [ osize ];
		outgoing_buffer_semaphore[i] = BUFFER_EMPTY;
		outgoing_buffer_length[i] = 0;
		outgoing_buffer_size[i] = osize;
	}

	cpu_set_t cpu_set;
	int i, err, time0;
	pthread_t thread;
	unsigned int cpu_mask;

/* disk streaming thread always needs to run, even if to /dev/null. it recycles buffers */
	disk_streamer_params.camera = this;
	disk_streamer_params.buffers = outgoing_buffer; 
	disk_streamer_params.bufflen = outgoing_buffer_length; 
	disk_streamer_params.semaphore = outgoing_buffer_semaphore;
	disk_streamer_params.n_buffers = n_outgoing_buffers;
	disk_streamer_params.thread_started = 0;
	disk_streamer_params.run = 1;
	disk_streamer_params.ofile = ofile;
	disk_streamer_params.logbuff = disk_streamer_logbuff;
	disk_streamer_params.logbuff_length = disk_streamer_logbuff_length;
	disk_streamer_params.cpu_mask = outgoing_streamer_cpu_mask;
	disk_streamer_params.sleep_wait = DISK_STREAMER_DEFAULT_SLEEP_WAIT; /* idle time in output streamer */
	err = pthread_create(&tid[ToDiskThreadId], NULL, &stream_to_disk, (void *)&disk_streamer_params);
	thread = tid[ToDiskThreadId];
	cpu_mask = outgoing_streamer_cpu_mask;
	if(err) {
		snprintf(logbuff, logbuff_length, "unable to create new thread for data streaming");
		if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
		return false;
	}

	if(cpu_mask) {
		CPU_ZERO(&cpu_set);
		for(i=0;i<32;++i) {
			if(cpu_mask & (1 << i)) CPU_SET(i, &cpu_set);
		}
		err = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpu_set);
		if(err) {
			snprintf(logbuff, logbuff_length, "unable to set thread affinity for disk streaming");
			if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
		}
		err = pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpu_set);
		if(err) {
			snprintf(logbuff, logbuff_length, "unable to get thread affinity for disk streaming");
			if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
		} else {
			for(i=0;i<CPU_SETSIZE;++i) {
				if(CPU_ISSET(i, &cpu_set)) {
					snprintf(logbuff, logbuff_length, "thread affinity for disk streaming = %d", i);
					if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
				}
			}
		}
	}

	time0 = time(NULL);
	while(disk_streamer_params.thread_started == 0) {
		int dtime = time(NULL) - time0;
		if(dtime > 5) {
			snprintf(logbuff, logbuff_length, 
				"timeout waiting for disk streaming thread to initialize (%d seconds)...", dtime);
			if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
			return false;
		}
		snprintf(logbuff, logbuff_length, "waiting for disk streaming thread to initialize (%d seconds)...", dtime);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		usleep(1000000);
	}
	snprintf(logbuff, logbuff_length, "disk streaming thread successfully initialized");
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

/* configure incoming = camera streamer */

	incoming_buffer = new unsigned char * [ n_incoming_buffers ];
	incoming_buffer_semaphore = new unsigned int [ n_incoming_buffers ];
	incoming_buffer_length = new int [ n_incoming_buffers ];
	incoming_buffer_size = new int [ n_incoming_buffers ];
	frame_capture_timestamp = new uint64_t [ n_incoming_buffers ];
	frame_capture_index = new int [ n_incoming_buffers ];

	if(isize == 0) {
		isize = width * height * bytes_per_pixel; /* jsv. actually need to sync this with driver buffer size */ 
		isize = width * height * 4; /* jsv. this needs to accomodate BGR */ 
	}

	for(int i=0;i<n_incoming_buffers;++i) {
		incoming_buffer[i] = new unsigned char [ isize ];
		incoming_buffer_semaphore[i] = BUFFER_EMPTY;
		incoming_buffer_length[i] = 0;
		incoming_buffer_size[i] = isize;
	}

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

bool Camera::close() {

	char *thread_result;

	if(camera_state != CAMERA_STATE_STOPPED) stop_capture();

	camera_streamer_params.run = disk_streamer_params.run = 0; /* tell threads to stop running */

/* there is some redundancy in shutting down services. first, ensure the loop running in thread stops */
	int timeout = time(0) + 8; /* 8 seconds to stop is long enough */
	while(camera_streamer_params.thread_started) {
		if(time(0) > timeout) {
			snprintf(logbuff, logbuff_length, "camera streamer thread termination timeout");
			if(log_fxn) log_fxn(LEVEL_FATAL, logbuff);
			
		}
		snprintf(logbuff, logbuff_length, "waiting for camera streamer thread to terminate");
		if(log_fxn) log_fxn(LEVEL_INFO, logbuff);
		usleep(100000);
	}

/* and then check the thread function actually returned */
	snprintf(logbuff, logbuff_length, "shutting down camera streamer service");
	log_fxn(LEVEL_INFO, logbuff);
	pthread_join(tid[FromCameraThreadId], (void**) &thread_result);
	snprintf(logbuff, logbuff_length, "camera streamer stopped");
	log_fxn(LEVEL_INFO, logbuff);

/* repeat for the disk streamer */
	while(disk_streamer_params.thread_started) {
		if(time(0) > timeout) {
			snprintf(logbuff, logbuff_length, "disk streamer thread termination timeout");
			if(log_fxn) log_fxn(LEVEL_FATAL, logbuff);
			
		}
		snprintf(logbuff, logbuff_length, "waiting for disk streamer thread to terminate");
		if(log_fxn) log_fxn(LEVEL_INFO, logbuff);
		usleep(1000000);
	}

	snprintf(logbuff, logbuff_length, "shutting down disk streamer service");
	log_fxn(LEVEL_INFO, logbuff);
	pthread_join(tid[ToDiskThreadId], (void**) &thread_result);
	snprintf(logbuff, logbuff_length, "disk streamer stopped");
	log_fxn(LEVEL_INFO, logbuff);

#if 0
/* clear buffers by setting count = 0 */
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(struct v4l2_requestbuffers));
	req.count = 0;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if(xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
		perror("Requesting Buffer");
		return 1;
	}
#endif

	return true;
}

Camera::Camera(int device, int width, int height, CameraLogFxn *log_fxn, int user_specified_type) : 
	calibrate(false), 
	n_outgoing_buffers(10),
	outgoing_buffer(0),
	outgoing_buffer_semaphore(0),
	outgoing_buffer_length(0),
	n_incoming_buffers(10),
	incoming_buffer(0),
	incoming_buffer_semaphore(0),
	incoming_buffer_length(0),
	temp_buffer(0),
	n_capture_buffers(4),
	capture_buffer(0),
	capture_length(0),
	coarse_integration(0), gain(0),
	isize(0),
	osize(0),
	fps(0),
	is_color(false),
	verbose(false),
	debug(false),
	ofile(0),
	display_window_name("camera_private"),
	compression_scheme(COMPRESSION_NONE),
	compression_quality(DEFAULT_JPEG_COMPRESSION_QUALITY),
	frame_index(0),
	incoming_index(0),
	outgoing_index(0),
	incoming_streaming(false),
	outgoing_streaming(false),
	camera_state(CAMERA_STATE_IDLE),
	frame_capture_timestamp(0),
	frame_capture_index(0),
	tid(0),
	writer(0) 
	{

	logbuff_length = 1024;
	disk_streamer_logbuff_length = 1024;
	camera_streamer_logbuff_length = 1024;
	logbuff = new char [ logbuff_length ];
	disk_streamer_logbuff = new char [ disk_streamer_logbuff_length ];
	camera_streamer_logbuff = new char [ camera_streamer_logbuff_length ];

	this->log_fxn = log_fxn;
	this->height = height;
	this->width = width;
	if(user_specified_type == -1) type = CAMERA_UNKNOWN;
	else type = user_specified_type;

	is_color = false;

	tid = new pthread_t [ NThreadIds ];

	tid = new pthread_t [ NThreadIds ];

	memset(&disk_streamer_params, 0, sizeof(DiskStreamerParams));

	sleep_wait = INCOMING_BUFFER_DEFAULT_SLEEP_WAIT; /* idle time waiting for buffer to free up */

/* instead of using _uint8, etc. stick to the standard types. check assumptions about size, though */
	if(sizeof(unsigned short int) != 2) {
		printf("mismatch in expect unsigned short int size = %zu. expect 2\n", sizeof(unsigned short int));
		return ;
	}

	if(sizeof(unsigned int) != 4) {
		printf("mismatch in expect unsigned int size = %zu. expect 4\n", sizeof(unsigned int));
		return ;
	}

/* open the device */
	sprintf(str, "/dev/video%d", device);
	fd = open(str, O_RDWR | O_NONBLOCK);
	if(fd == -1) { perror("Opening video device"); return; }

/* discover camera and parameters */
	if(print_caps() == false) { return ; }

	device_status = true;
}

Camera::~Camera() {

	if(camera_state != CAMERA_STATE_STOPPED) stop_capture();

	this->close();

	for(unsigned int i=0;i<n_capture_buffers;++i) {
		snprintf(logbuff, logbuff_length, "buffer %d munmap(%p, %d)", i, capture_buffer[i], capture_length[i]);
		if(log_fxn) log_fxn(LEVEL_INFO, logbuff);
		munmap(capture_buffer[i], capture_length[i]);
	}

	::close(fd);
	snprintf(logbuff, logbuff_length, "close(fd=%d)", fd);
	if(log_fxn) log_fxn(LEVEL_INFO, logbuff);
	if(capture_buffer) delete [] capture_buffer;
	if(capture_length) delete [] capture_length;
	if(outgoing_buffer) {
		for(int i=0;i<n_outgoing_buffers;++i) {
			delete [] outgoing_buffer[i];
		}
		delete [] outgoing_buffer;
	}
	if(outgoing_buffer_semaphore) delete [] outgoing_buffer_semaphore;
	if(outgoing_buffer_length) delete [] outgoing_buffer_length;
	if(incoming_buffer) {
		for(int i=0;i<n_incoming_buffers;++i) delete [] incoming_buffer[i];
		delete [] incoming_buffer;
	}
	if(incoming_buffer_semaphore) delete [] incoming_buffer_semaphore;
	if(incoming_buffer_length) delete [] incoming_buffer_length;
	if(writer) delete writer;
	if(ofile) delete [] ofile;
	if(logbuff) delete [] logbuff;
	if(disk_streamer_logbuff) delete [] disk_streamer_logbuff;
	if(camera_streamer_logbuff) delete [] camera_streamer_logbuff;
	if(frame_capture_timestamp) delete [] frame_capture_timestamp;
	if(frame_capture_index) delete [] frame_capture_index;
	if(tid) delete [] tid;
}

bool debug_mode = false;

extern bool run;
bool pause_ops = false;
static void stop(int sig) { run = false; } /* when we hit ^C tell main loop to exit */
static void set_pause(int sig) { pause_ops = true; }
static void clr_pause(int sig) { pause_ops = false; }

enum {
	VIDEO_INFO,
	FRAME_INFO,
	GPS_INFO
};

typedef struct _VideoInfo {
	int structure_type, structure_length, payload_length;
	long long unsigned int time_stamp;
	int frame_height, frame_width, pixel_bits, frame_size;
	float rate;
} VideoInfo;

typedef struct _FrameInfo {
	int structure_type, structure_length, payload_length;
	long long unsigned int time_stamp;
	int height, width, pixel_bits, size, index;
	float rate;
} FrameInfo;

typedef struct _GPSInfo {
	int structure_type, structure_length;
	long long unsigned int time_stamp;
	double lat, lon;
} GPSInfo;

#if 0
long long unsigned int get_time_stamp() {
	long long unsigned int result = time(0);
	result = (result << 32) | (clock() & 0xffffffff);
	return result;
};
#endif

static int xioctl(int fd, int request, void *arg) {
#ifdef VERBOSE_DEBUG
printf("xioctl(fd=%d, request=%d, arg=%p) [enter]\n", fd, request, arg);
#endif
	int status = ioctl(fd, request, arg);
#ifdef VERBOSE_DEBUG
printf("xioctl(fd=%d, request=%d, arg=%p) [return]\n", fd, request, arg);
#endif
	return status;
#if 0
	int r;
	do r = ioctl(fd, request, arg);
	while (r == -1 && EINTR == errno);
	return r;
#endif
}

GPSInfo get_gps_info() {
	GPSInfo result = { GPS_INFO, sizeof(GPSInfo) };
	return result;
}

/* note: one should preallocate a large vector for ovec so that resizing won't happen */
bool Camera::compress_frame(unsigned char *ibuff, std::vector<uchar> &obuff) {
	int compression_scheme = get_compression_scheme();
	if(compression_scheme == COMPRESSION_JPEG) {
		int compression_quality = get_compression_params(Camera::COMPRESSION_PARAMS_QUALITY);
		std::vector<int> compression_params = std::vector<int>(2);
		compression_params[0] = CV_IMWRITE_JPEG_QUALITY;
		compression_params[1] = compression_quality;
		if(type == Camera::CAMERA_LI_USB3_MT9M021M) {
/* jsv. i am not proud of this. I have to take a perfectly good grey scale 16-bit image 
   and scale down to 8-bits for the compression to work */
/* instead of this: Mat src_frame(height, width, CV_16UC1, (void *)ibuff);
 * i have to do the following: */
			// jsv was unsigned char *tbuff = new unsigned char[ height * width * 3];
			unsigned char *tbuff = new unsigned char[ height * width ];
			unsigned short int *xp = (unsigned short int *)ibuff;
			for(int i=0;i<height;++i) {
				for(int j=0;j<width;++j) {
					unsigned short int xi = *xp++; 
					unsigned char x = (xi >> 8);
					tbuff[i * width + j] = x;
				}
			}
			Mat src_frame(height, width, CV_8UC1, (void *)tbuff);
			obuff.clear();
			imencode(".jpg", src_frame, obuff, compression_params);
// printf("compressed size = %d\n", obuff.size());
			Mat jpegimage = imdecode(Mat(obuff), CV_LOAD_IMAGE_COLOR);
// Mat jpegimage = imdecode(Mat(obuff),CV_LOAD_IMAGE_ANYDEPTH);
			imshow("decode", jpegimage);
			waitKey(10);

			delete [] tbuff;
		} else if(type == Camera::CAMERA_LI_USB3_MT9M021C) {
/* TODO jsv rework for 8-bit or 16-bit */
/* jsv. i am not proud of this. I have to take a perfectly good 16-bit BGR image 
   and scale down to 8-bits for the compression to work */
/* instead of this: Mat src_frame(height, width, CV_16UC3, (void *)ibuff);
 * i have to do the following: */
			int nchans = 3, rows = height / 2, cols = width / 2;
			unsigned char *tbuff = new unsigned char[ rows * cols * 3 ];
			unsigned short int *xp = (unsigned short int *)ibuff;
			for(int i=0;i<rows;++i) {
				for(int j=0;j<cols;++j) {
					unsigned short int bi = *xp++; 
					unsigned char b = (bi >> 8);
					unsigned short int gi = *xp++; 
					unsigned char g = (gi >> 8);
					unsigned short int ri = *xp++; 
					unsigned char r = (ri >> 8);
					tbuff[nchans * (i * width + j) + 0] = b;
					tbuff[nchans * (i * width + j) + 1] = g;
					tbuff[nchans * (i * width + j) + 2] = r;
				}
			}
			Mat src_frame(height, width, CV_8UC3, (void *)tbuff);
			obuff.clear();
			imencode(".jpg", src_frame, obuff, compression_params);
// printf("compressed size = %d\n", obuff.size());
			Mat jpegimage = imdecode(Mat(obuff),CV_LOAD_IMAGE_COLOR);
// Mat jpegimage = imdecode(Mat(obuff),CV_LOAD_IMAGE_ANYDEPTH);
			imshow("decode", jpegimage);
			waitKey(10);

			delete [] tbuff;
		}
	}
	return true;
}

/* note: this function doesn't return until the run flag is cleared. 
   its sole job is to stream buffers and data to disk.
   the semaphore is set by the data acquisition, and cleared by stream_to_disk() once written to disk.
   this is NOT intended to be reentrant code only because the file being written is not closed until the end.
   otherwise, the code follows reentrant-safe practices */

void *stream_to_disk(void *ptr) {

	DiskStreamerParams *params = (DiskStreamerParams *)ptr;

	signal(SIGINT, stop); /* ^C  exception handling */ 
	signal(SIGTERM, stop); /* exception handling */ 
	signal(SIGUSR1, set_pause); /* USR1 signal pauses */ 
	signal(SIGUSR2, clr_pause); /* USR2 signal resumes */ 

	FILE *fp = 0;
	int index = 0;
	Camera *camera = params->camera;
	const char *ofile = params->ofile;
	std::string cur_filename;
	int logbuff_length = params->logbuff_length;
	char *logbuff = params->logbuff;
	CameraLogFxn *log_fxn = camera->log_fxn;

	if(ofile) {
		cur_filename = ofile;
		fp = fopen(cur_filename.c_str(), "wb"); 
		snprintf(logbuff, logbuff_length, 
			"stream_to_disk(): file %s opened for output", (char *)ofile);
	} else {
		snprintf(logbuff, logbuff_length, "stream_to_disk(): data will be discarded");
	}
	if(log_fxn) (*log_fxn)(Camera::LEVEL_INFO, logbuff);
	params->thread_started = 1;

	int n_buffers = params->n_buffers;
	unsigned int *semaphore = params->semaphore;
	unsigned char **buffers = params->buffers;
	int *bufflen = params->bufflen;

	int cur_filesize = 0, file_index = 0; /* cumulative file size */

	while(params->run && run) {

		if(pause_ops) {
			if(log_fxn) {
				snprintf(logbuff, logbuff_length, "stream_to_disk(): enter PAUSED state");
				(*log_fxn)(LEVEL_INFO, logbuff);
			}

			while(pause_ops) { 
				snprintf(logbuff, logbuff_length, "stream_to_disk(): state = PAUSED");
				(*log_fxn)(LEVEL_INFO, logbuff);
				sleep(2);
			}

			if(log_fxn) {
				snprintf(logbuff, logbuff_length, "stream_to_disk(): resume operations");
				(*log_fxn)(LEVEL_INFO, logbuff);
			}
		}

// printf("stream_to_disk(): bufflen(%d) = %d\n", index, bufflen[index]);
		if(fp) { /* if actually writing to disk, wait */
// printf("fp = %p\n", fp);
			while(semaphore[index] == Camera::BUFFER_EMPTY) { /* wait until data ready */
				usleep(params->sleep_wait);
				if(run == false) goto grand_finale;
			}
			fwrite(buffers[index], 1, bufflen[index], fp);
		} else {
// int sleep_wait = params->sleep_wait;
// sleep_wait = 1000000;
// printf("sleep wait = %d. index = %d / %d. run = %d/%d\n", params->sleep_wait, index, params->n_buffers, run ? 1 : 0, params->run ? 1 : 0);
			usleep(params->sleep_wait);
		}

		semaphore[index] = Camera::BUFFER_EMPTY; /* clear data ready flag */
		++index;
		index = index % n_buffers;
	}

grand_finale:

	if(fp) fclose(fp);

	params->thread_started = 0;

	if(log_fxn) {
		snprintf(logbuff, logbuff_length, "disk output streamer terminated");
		(*log_fxn)(Camera::LEVEL_INFO, logbuff);
	}

	return 0;
}

static void convert_border_bayer16_line_to_bgr24(unsigned short int *bayer, unsigned short int *adjacent_bayer,
	unsigned char *bgr, int width, int shift, bool start_with_green, bool blue_line) {
	int t0, t1;

	if (start_with_green) { /* first pixel */
		if (blue_line) {
			*bgr++ = bayer[1] >> shift;
			*bgr++ = bayer[0] >> shift;
			*bgr++ = adjacent_bayer[0] >> shift;
		} else {
			*bgr++ = adjacent_bayer[0] >> shift;
			*bgr++ = bayer[0] >> shift;
			*bgr++ = bayer[1] >> shift;
		}
		/* Second pixel */
		t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
		t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
		if (blue_line) {
			*bgr++ = bayer[1] >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = t1 >> shift;
		} else {
			*bgr++ = t1 >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = bayer[1] >> shift;
		}
		bayer++;
		adjacent_bayer++;
		width -= 2;
	} else { /* first pixel */
		t0 = (bayer[1] + adjacent_bayer[0] + 1) >> 1;
		if (blue_line) {
			*bgr++ = bayer[0] >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = adjacent_bayer[1] >> shift;
		} else {
			*bgr++ = adjacent_bayer[1] >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = bayer[0] >> shift;
		}
		width--;
	}

	if (blue_line) {
		for ( ; width > 2; width -= 2) {
			t0 = (bayer[0] + bayer[2] + 1) >> 1;
			*bgr++ = t0 >> shift;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = adjacent_bayer[1] >> shift;
			bayer++;
			adjacent_bayer++;

			t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
			t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = t1 >> shift;
			bayer++;
			adjacent_bayer++;
		}
	} else {
		for ( ; width > 2; width -= 2) {
			t0 = (bayer[0] + bayer[2] + 1) >> 1;
			*bgr++ = adjacent_bayer[1] >> shift;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = t0 >> shift;
			bayer++;
			adjacent_bayer++;

			t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
			t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
			*bgr++ = t1 >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = bayer[1] >> shift;
			bayer++;
			adjacent_bayer++;
		}
	}

	if (width == 2) { /* penultimate pixel */
		t0 = (bayer[0] + bayer[2] + 1) >> 1;
		if (blue_line) {
			*bgr++ = t0 >> shift;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = adjacent_bayer[1] >> shift;
		} else {
			*bgr++ = adjacent_bayer[1] >> shift;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = t0 >> shift;
		}
		/* Last pixel */
		t0 = (bayer[1] + adjacent_bayer[2] + 1) >> 1;
		if (blue_line) {
			*bgr++ = bayer[2] >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = adjacent_bayer[1] >> shift;
		} else {
			*bgr++ = adjacent_bayer[1] >> shift;
			*bgr++ = t0 >> shift;
			*bgr++ = bayer[2] >> shift;
		}
	} else { /* last pixel */
		if (blue_line) {
			*bgr++ = bayer[0] >> shift;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = adjacent_bayer[1] >> shift;
		} else {
			*bgr++ = adjacent_bayer[1] >> shift;
			*bgr++ = bayer[1] >> shift;
			*bgr++ = bayer[0] >> shift;
		}
	}
}

/* From libdc1394, which on turn was based on OpenCV's Bayer decoding */
void bayer16_to_rgb888(unsigned short int *bayer, unsigned char *bgr, int width, int height, int shift,
	bool start_with_green, bool blue_line) {

	/* render the first line */
	convert_border_bayer16_line_to_bgr24(bayer, bayer + width, bgr, width, shift, start_with_green, blue_line);

	bgr += width * 3;

	/* reduce height by 2 because of the special case top/bottom line */
	for(height-=2;height;height--) {
		int t0, t1;
		/* (width - 2) because of the border */
		unsigned short int *bayerEnd = bayer + (width - 2);

		if(start_with_green) {
			/* OpenCV has a bug in the next line, which was
			t0 = (bayer[0] + bayer[width * 2] + 1) >> 1; */
			t0 = (bayer[1] + bayer[width * 2 + 1] + 1) >> 1;
			/* write first pixel */
			t1 = (bayer[0] + bayer[width * 2] + bayer[width + 1] + 1) / 3;
			if(blue_line) {
				*bgr++ = t0 >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = bayer[width] >> shift;
			} else {
				*bgr++ = bayer[width] >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = t0 >> shift;
			}

			/* write second pixel */
			t1 = (bayer[width] + bayer[width + 2] + 1) >> 1;
			if (blue_line) {
				*bgr++ = t0 >> shift;
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = t1 >> shift;
			} else {
				*bgr++ = t1 >> shift;
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = t0 >> shift;
			}
			bayer++;
		} else {
			/* write first pixel */
			t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
			if (blue_line) {
				*bgr++ = t0 >> shift;
				*bgr++ = bayer[width] >> shift;
				*bgr++ = bayer[width + 1] >> shift;
			} else {
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = bayer[width] >> shift;
				*bgr++ = t0 >> shift;
			}
		}

		if (blue_line) {
			for(;bayer<=bayerEnd-2;bayer+=2) {
				t0 = (bayer[0] + bayer[2] + bayer[width * 2] + bayer[width * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[width] + bayer[width + 2] + bayer[width * 2 + 1] + 2) >> 2;
				*bgr++ = t0 >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = bayer[width + 1] >> shift;

				t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
				t1 = (bayer[width + 1] + bayer[width + 3] + 1) >> 1;
				*bgr++ = t0 >> shift;
				*bgr++ = bayer[width + 2] >> shift;
				*bgr++ = t1 >> shift;
			}
		} else {
			for (;bayer<=bayerEnd-2;bayer+=2) {
				t0 = (bayer[0] + bayer[2] + bayer[width * 2] + bayer[width * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[width] + bayer[width + 2] + bayer[width * 2 + 1] + 2) >> 2;
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = t0 >> shift;

				t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
				t1 = (bayer[width + 1] + bayer[width + 3] + 1) >> 1;
				*bgr++ = t1 >> shift;
				*bgr++ = bayer[width + 2] >> shift;
				*bgr++ = t0 >> shift;
			}
		}

		if(bayer < bayerEnd) {
		/* write second to last pixel */
			t0 = (bayer[0] + bayer[2] + bayer[width * 2] + bayer[width * 2 + 2] + 2) >> 2;
			t1 = (bayer[1] + bayer[width] + bayer[width + 2] + bayer[width * 2 + 1] + 2) >> 2;
			if (blue_line) {
				*bgr++ = t0 >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = bayer[width + 1] >> shift;
			} else {
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = t0 >> shift;
			}
		/* write last pixel */
			t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
			if (blue_line) {
				*bgr++ = t0 >> shift;
				*bgr++ = bayer[width + 2] >> shift;
				*bgr++ = bayer[width + 1] >> shift;
			} else {
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = bayer[width + 2] >> shift;
				*bgr++ = t0 >> shift;
			}
			bayer++;
		} else {
			/* write last pixel */
			t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
			t1 = (bayer[1] + bayer[width * 2 + 1] + bayer[width] + 1) / 3;
			if (blue_line) {
				*bgr++ = t0 >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = bayer[width + 1] >> shift;
			} else {
				*bgr++ = bayer[width + 1] >> shift;
				*bgr++ = t1 >> shift;
				*bgr++ = t0 >> shift;
			}
		}

		/* skip 2 border pixels */
		bayer += 2;

		blue_line = !blue_line;
		start_with_green = !start_with_green;
	}

	/* render the last line */
	convert_border_bayer16_line_to_bgr24(bayer + width, bayer, bgr, width, shift, !start_with_green, !blue_line);
}

bool Camera::print_caps() {

	struct v4l2_capability cap;
	memset(&cap, 0, sizeof(cap));
	if(xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
		perror("Querying Capabilities");
		return false;
	}

	unsigned int expected_min_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	if((cap.capabilities & expected_min_caps) != expected_min_caps) {
		snprintf(logbuff, logbuff_length, "camera does not have required capabilities");
		if(log_fxn) (*log_fxn)(LEVEL_FATAL, logbuff);
		return false;
	}

/* jsv. are these showing up in the logs */
	snprintf(logbuff, logbuff_length, "Driver Caps: Driver: \"%s\" Card: \"%s\"", cap.driver, cap.card);
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

/* jsv. are these showing up in the logs */
	snprintf(logbuff, logbuff_length,
		"Driver Caps: Bus: \"%s\" Version: %d.%d Capabilities: %08x",
		cap.bus_info, (cap.version>>16)&0xff, (cap.version>>24)&0xff, cap.capabilities);
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

	if(type == CAMERA_UNKNOWN) { /* automatic discovery */
		snprintf(logbuff, logbuff_length, "attempting automatic detection of camera type");
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		if(strcmp((char *)cap.card, "MT9V034") == 0) {
			type = CAMERA_LI_USB3_MT9V034;
			is_color = false;
		} else if(strcmp((char *)cap.card, "MT9M021C") == 0) {
			type = CAMERA_LI_USB3_MT9M021C;
			is_color = true;
		} else if((strcmp((char *)cap.card, "MT9M021M") == 0) || 
			(strcmp((char *)cap.card, "MT9M031") == 0)) {
			is_color = false;
			type = CAMERA_LI_USB3_MT9M021M;
			snprintf(logbuff, logbuff_length, "camera module MT9M021 monochrome detected");
			if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		} else if(strcmp((char *)cap.card, "LI-10635") == 0) {
			is_color = true;
			type = CAMERA_LI_USB3_OV10635;
		} else if(strcmp((char *)cap.card, "See3CAMCU50") == 0) {
            is_color = true;
            type = CAMERA_ECON_SEE3CAMCU50;
        } else if(strcmp((char *)cap.card, "AR023ZWDR") == 0) {
            is_color = true;
            type = CAMERA_LI_USB3_AR023Z;
		} else {
			snprintf(logbuff, logbuff_length, "unknown camera detected");
			if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
			return false;
		}
	} else if(type == CAMERA_LI_USB3_MT9M021C) {
		is_color = true;
		snprintf(logbuff, logbuff_length, "user specified LI-MT9M021C (color)");
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	} else if(type == CAMERA_LI_USB3_MT9M021M) {
		is_color = false;
		snprintf(logbuff, logbuff_length, "user specified LI-MT9M021M (monochrome)");
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	} else {
		snprintf(logbuff, logbuff_length, "user specified invalid camera type");
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	}

	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof(cropcap));
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(xioctl (fd, VIDIOC_CROPCAP, &cropcap) == -1) {
		perror("Querying Cropping Capabilities");
		return false;
	}

	snprintf(logbuff, logbuff_length, 
		"Camera Cropping: Bounds: %dx%d+%d+%d Default: %dx%d+%d+%d Aspect: %d/%d",
		cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
		cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
		cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

	int support_grbg10 = 0;

	struct v4l2_fmtdesc fmtdesc = {0};
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	char fourcc[5] = {0};
	char c, e;
	snprintf(logbuff, logbuff_length, "==> FMT :");
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	snprintf(logbuff, logbuff_length, "==> CE Desc --------------------");
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	while(xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
		strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
		if(fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10) support_grbg10 = 1;
		c = fmtdesc.flags & 1? 'C' : ' ';
		e = fmtdesc.flags & 2? 'E' : ' ';
		snprintf(logbuff, logbuff_length, "==> %s: %c%c %s", fourcc, c, e, fmtdesc.description);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		fmtdesc.index++;
	}

	return true;
}

bool Camera::stop_capture() {

	if(camera_state == CAMERA_STATE_STOPPED) {
		snprintf(logbuff, logbuff_length, "stop_capture() called, but camera already stopped");
		if(log_fxn) (*log_fxn)(LEVEL_WARNING, logbuff);
		return false;
	}

	camera_state = CAMERA_STATE_STOPPED;

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(xioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
		perror("Stop Capture");
		return false;
	}

	snprintf(logbuff, logbuff_length, "capture stopped");
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
	return true;
}

bool Camera::start_capture() {

	if(camera_state == CAMERA_STATE_RUNNING) {
		snprintf(logbuff, logbuff_length, "camera already running");
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		return false;
	}

	snprintf(logbuff, logbuff_length, "start_capture(%d)", fd);
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

/* queue the buffers */
	struct v4l2_buffer buf;
	for(unsigned int i=0;i<n_capture_buffers;++i) {
		snprintf(logbuff, logbuff_length, "camera: start_capture(): VIDIOC_QBUF %d", i);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
			snprintf(logbuff, logbuff_length, "error: VIDIOC_QBUF %d", i);
			if(log_fxn) (*log_fxn)(LEVEL_ERROR, logbuff);
			perror("Queue Buffer");
			return 1;
		}
	}

/* enable streaming */
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
		perror("Start Capture");
		return false;
	}

	incoming_streaming = true;

	snprintf(logbuff, logbuff_length, "camera: capture started");
	if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);

	for(int i=0;i<n_outgoing_buffers;++i) {
		outgoing_buffer_semaphore[i] = BUFFER_EMPTY;
	}

	for(int i=0;i<n_incoming_buffers;++i) {
		incoming_buffer_semaphore[i] = BUFFER_EMPTY;
	}

	camera_state = CAMERA_STATE_RUNNING;

	return true;
}

/* this function is not intended to return. it runs as a separate thread */
void *stream_from_camera(void *ptr) {

	CameraStreamerParams *params = (CameraStreamerParams *)ptr;

	signal(SIGINT, stop); /* ^C  exception handling */ 

	int index = 0;

	Camera *camera = params->camera;
	int height = camera->get_height();
	int width = camera->get_width();
	int type = camera->get_type();
	int frame_size = camera->get_frame_size();
	int bytes_per_pixel = camera->get_bytes_per_pixel();
	int n_buffers = params->n_buffers;
	int fd = params->fd;
	bool calibrate = camera->get_calibrate();
	CameraLogFxn *log_fxn = camera->log_fxn;

	unsigned int *semaphore = params->semaphore;
	unsigned char **buffers = params->buffers;
	uint64 *frame_capture_timestamp = params->frame_capture_timestamp;
	int *frame_capture_index = params->frame_capture_index;
	int *bufflen = params->bufflen;
	char *logbuff = params->logbuff;
	int logbuff_length = params->logbuff_length;

	int size = height * width * 4; /* capture worst case scenario */
	unsigned char *temp_buffer = new unsigned char [ size ];

	params->thread_started = 1;

	while(camera->get_incoming_streaming() == false) { usleep(10000); }

	snprintf(logbuff, logbuff_length, "stream_from_camera(): incoming capture/streaming started");
	if(log_fxn) (*log_fxn)(Camera::LEVEL_INFO, logbuff);

	*params->device_status = true; /* if anything happens to the camera, device_status := false */

	while(params->run && run && *params->device_status) {

		if(camera->get_camera_state() != Camera::CAMERA_STATE_RUNNING) {
			usleep(100000);
			continue;
		}

if(debug_mode) printf("stream_from_camera(): waiting for frame to be ready\n");

		if(pause_ops) {
			if(log_fxn) {
				snprintf(logbuff, logbuff_length, "stream_from_camera(): enter PAUSE state");
				(*log_fxn)(LEVEL_INFO, logbuff);
			}
			while(pause_ops) { sleep(2); }
			if(log_fxn) {
				snprintf(logbuff, logbuff_length, "stream_from_camera(): resume operations");
				(*log_fxn)(LEVEL_INFO, logbuff);
			}
		}

	/* wait for a frame to be ready, by polling */
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		struct timeval tv;
		memset(&tv, 0, sizeof(struct timeval));
		tv.tv_sec = 2;
		int r = select(fd+1, &fds, NULL, NULL, &tv);
		if(r == 0) {
			snprintf(logbuff, logbuff_length, "stream_from_camera(): select() timeout waiting for buffer");
			(*log_fxn)(Camera::LEVEL_WARNING, logbuff);
			continue;
		} else if(r == -1) {
			*params->device_status = false;
			perror("Waiting for Frame");
			break;
		}

		if(debug_mode && log_fxn) {
			snprintf(logbuff, logbuff_length, "stream_from_camera(): about to dequeue buffer");
			(*log_fxn)(Camera::LEVEL_DEBUG, logbuff);
		}

		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if(xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
			*params->device_status = false;
			perror("Retrieving Frame");
			break;
		}

		unsigned long int buffer_flags = buf.flags;
		int capture_index = buf.index; /* index of input capture buffer from camera */

		if(buf.flags & V4L2_BUF_FLAG_ERROR) {
		/* there was an error in this buffer. requeue */
			memset(&buf, 0, sizeof(struct v4l2_buffer));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = capture_index;
			if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
				*params->device_status = false;
				perror("Queue Buffer");
				break;
			}
			continue;
		}

		if(debug_mode && log_fxn) {
			snprintf(logbuff, logbuff_length,
				"stream_from_camera(): captured image index = %d. length=%d\n", buf.index, buf.length);
			(*log_fxn)(Camera::LEVEL_DEBUG, logbuff);
		}

#if 0
/* jsv. for testing. remove this next block. it would be interesting to see if this ever fails */
		if(semaphore[index] == Camera::BUFFER_FULL) {
			if(log_fxn) {
				snprintf(logbuff, logbuff_length, 
					"camera capture state machine error. buffer full");
				(*log_fxn)(Camera::LEVEL_INFO, logbuff);
			}
		}
#endif

#if 0
		if(buffer_flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) { /* prints out timestamp and index for each frame */
			snprintf(logbuff, logbuff_length,
				"timestamp = %lx %ld.%6.6ld", buffer_flags, buf.timestamp.tv_sec, buf.timestamp.tv_usec);
			(*log_fxn)(Camera::LEVEL_INFO, logbuff);
		}
#endif

#if 0
		snprintf(logbuff, logbuff_length, "Image Length: %d", buf.bytesused);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
#endif

#if 0
		if(buffer_flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) { /* prints out timestamp and index for each frame */
			snprintf(logbuff, logbuff_length,
				"timestamp = %lx %ld.%6.6ld", buffer_flags, buf.timestamp.tv_sec, buf.timestamp.tv_usec);
			(*log_fxn)(Camera::LEVEL_INFO, logbuff);
		}
#endif

#if 0
		snprintf(logbuff, logbuff_length, "Image Length: %d", buf.bytesused);
		if(log_fxn) (*log_fxn)(LEVEL_INFO, logbuff);
#endif

		unsigned char *src_buffer = params->capture_buffer[capture_index];
		int src_length = buf.length;
		unsigned char *dst_buffer = buffers[index];
		int dst_length = bufflen[index];
		bufflen[index] = src_length; 
		int frame_capture_time = buf.timestamp.tv_sec;
		// frame_capture_time = frame_capture_time * 1000000 + buf.timestamp.tv_usec; /* microseconds */
		frame_capture_time = frame_capture_time * 1000 + (buf.timestamp.tv_usec / 1000); /* milliseconds */
		frame_capture_timestamp[index] = frame_capture_time;
		frame_capture_index[index] = buf.sequence;

		if(debug_mode && log_fxn) {
			snprintf(logbuff, logbuff_length, "stream_from_camera(TYPE=%d): preprocess image", type);
			(*log_fxn)(Camera::LEVEL_DEBUG, logbuff);
		}

	/* nature of copy to input buffer depends on a few factors, e.g. type of camera, calibration, etc.
	   so life gets a little complicated, but broad strokes picture is same: data ends up in dst_buffer */
		if(type == Camera::CAMERA_LI_USB3_MT9M021M) { 

		/* calibration procedure involves going through an intermediate correction */
		/* this loop does 2 pixels at a time by reading/shifting an uint32, whereas data is uint16 */
			unsigned int *dst = calibrate ? (unsigned int *)temp_buffer : (unsigned int *)dst_buffer;
			unsigned int *src = (unsigned int *)src_buffer;
			for(unsigned int i=0;i<height;++i) { /* loop over data as it gets copied over to frame buffer */
				for(unsigned int j=0;j<width;j+=2) {
					unsigned int x = *src++;
					x <<= 4; /* shift up by 4 to use most dynamic range */
					*dst++ = x;
				}
			}
			// bufflen = height * width * 2; /* jsv. compare to src length ? */ 

			if(calibrate) {
				Mat frame(height, width, CV_16UC1, (void *)temp_buffer);
				Mat corrected_frame(height, width, CV_16UC1, (void *)dst_buffer);
			/* frame_buffer will contain corrected data, but in BGR format */
				undistort(frame, corrected_frame, camera->camera_matrix, camera->dist_coeffs);
			}

		} else if(type == Camera::CAMERA_LI_USB3_MT9M021C) { 

#if 0
/* this code discards the bayering scheme and makes BGR pixels at the expense of half resolution */
			int row, col, rows = height / 2, cols = width / 2;
			unsigned short int *src = (unsigned short int *)src_buffer;
			// unsigned char *dst = (unsigned char *)dst_buffer;
			unsigned short int *dst = (unsigned short int *)dst_buffer;
			for(row=0;row<rows;++row) {
				unsigned short int *upper_line = &src[(row * 2 + 0) * (cols * 2)];
				unsigned short int *lower_line = &src[(row * 2 + 1) * (cols * 2)];
				for(col=0;col<cols;++col) {
					unsigned short int g1 = upper_line[col * 2 + 0]; 
					unsigned short int g2 = lower_line[col * 2 + 1]; 
					unsigned short int g = (g1 >> 1) + (g2 >> 1); /* shift before add */
					// g = g1;
					unsigned short int b = lower_line[col * 2 + 0];
					unsigned short int r = upper_line[col * 2 + 1];
					// *dst++ = (b >> 4); 
					// *dst++ = (g >> 4); 
					// *dst++ = (r >> 4); 
					*dst++ = (b << 4); 
					*dst++ = (g << 4); 
					*dst++ = (r << 4); 
				}
			}
#endif

			bayer16_to_rgb888((unsigned short int *)src_buffer, dst_buffer, width, height, 4);
#if 0
			if(calibrate) {
				bayer16_to_rgb888((unsigned short int *)src_buffer, temp_buffer, width, height, 4);
				Mat frame(height, width, CV_8UC3, (void *)temp_buffer);
				Mat corrected_frame(height, width, CV_8UC3, (void *)dst_buffer);
			/* frame_buffer contains will contain corrected data */
				undistort(frame, corrected_frame, camera->camera_matrix, camera->dist_coeffs);
			} else {
				bayer16_to_rgb888((unsigned short int *)src_buffer, dst_buffer, width, height, 4);
			}
			// bufflen = height * width * 2; /* jsv. compare to src length ? */ 
#endif

		} else if(type == Camera::CAMERA_LI_USB3_OV10635) { 

			int n_copy = src_length;
			if(n_copy > dst_length) n_copy = dst_length; /* jsv. issue warning */
			memcpy(dst_buffer, src_buffer, n_copy);

		} else { /* everything else so far */

			int n_copy = src_length;
			if(n_copy > dst_length) n_copy = dst_length; /* jsv. issue warning */
			memcpy(dst_buffer, src_buffer, n_copy);

		}

		if(debug_mode && log_fxn) {
			snprintf(logbuff, logbuff_length, 
				"stream_from_camera(): incoming: marking buffer %d / %d as full", index, n_buffers);
			(*log_fxn)(Camera::LEVEL_DEBUG, logbuff);
		}

		memset(&buf, 0, sizeof(struct v4l2_buffer));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = capture_index; /* requeue current buffer */
		if(xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
			*params->device_status = false;
			perror("Queue Buffer");
			break;
		}

	/* return the capture buffer back so kernel can capture camera data into it */
		if(debug_mode && log_fxn) {
			snprintf(logbuff, logbuff_length, 
				"stream_from_camera(): capture buffer %d returned to pool", capture_index);
			(*log_fxn)(Camera::LEVEL_DEBUG, logbuff);
		}

	/* callbacks */
		std::vector<CameraCallbackParams>::const_iterator cbiter, cblast = camera->callback_params.end();
		for(cbiter = camera->callback_params.begin();cbiter != cblast; ++cbiter) {
			CameraCallbackFxn *fxn = cbiter->fxn;
			void *ext = cbiter->ext;
			(*fxn)(camera, dst_buffer, ext);
		}

	/* the only way to mark this buffer as full, is that the next buffer is empty.
	   if the next buffer is not empty, then the assumption is that the buffer client is running behind */
		int next_index = (index + 1) % n_buffers;
		if(semaphore[next_index] == Camera::BUFFER_EMPTY) {
			semaphore[index] = Camera::BUFFER_FULL; /* ready for consumption */
			index = next_index;
		}

		++camera->frame_index;
	}

grand_finale:

	delete [] temp_buffer;

	params->thread_started = 0;

	if(log_fxn) {
		snprintf(logbuff, logbuff_length, "camera input streamer terminated");
		(*log_fxn)(Camera::LEVEL_INFO, logbuff);
	}

#if 0
jsv
	if(*params->device_status == false) {
	/* we are here because something happened to camera. we don't want to exit thread until instructed */
		while(params->run && run) {
			usleep(500000);
		}
	}
#endif

	return 0;
}

void *Camera::get_outgoing_buffer(bool wait_ready) {

	if(debug && log_fxn) {
		snprintf(logbuff, logbuff_length, "get_outgoing_buffer(%d)", outgoing_index);
		(*log_fxn)(Camera::LEVEL_DEBUG, logbuff);
	}

	if(outgoing_buffer_semaphore[outgoing_index] == BUFFER_FULL) {
		if(log_fxn) {
			snprintf(logbuff, logbuff_length,
				"WARNING: streaming thread may be unable to keep real time data flow to disk. "
				"frame index = %d. out index = %d", frame_index, outgoing_index);
			(*log_fxn)(Camera::LEVEL_INFO, logbuff);
		}
		if(wait_ready == false) return 0;
	}

	int n_seconds = 4;
	int sunset = time(0) + n_seconds; 
	while(outgoing_buffer_semaphore[outgoing_index] == BUFFER_FULL) {
		if(time(0) > sunset) {
			if(log_fxn) {
				snprintf(logbuff, logbuff_length,
					"WARNING: %d seconds without buffer(%d) written to disk",
					n_seconds, outgoing_index);
				(*log_fxn)(Camera::LEVEL_INFO, logbuff);
			}
			return 0;
		}
		usleep(sleep_wait); /* wait until written to disk */
	}

	return outgoing_buffer[outgoing_index];
}

bool Camera::release_outgoing_buffer(int n_write, int index) {
	if(index < 0) {
//		if(n_write == 0) n_write = height * width * bytes_per_pixel;
		outgoing_buffer_length[outgoing_index] = n_write;
		outgoing_buffer_semaphore[outgoing_index] = BUFFER_FULL;
		++outgoing_index;
		if(outgoing_index >= n_outgoing_buffers) outgoing_index = 0;
	} else {
		if(index != outgoing_index) {
			if(verbose) printf("WARNING: outgoing indices out of sync\n");
			return false;
		}
		outgoing_buffer_semaphore[index] = BUFFER_FULL; /* still do what was requested */
	}
	return true;
}

bool Camera::release_incoming_buffer(int index) {
	if(index < 0) {
		incoming_buffer_semaphore[incoming_index] = BUFFER_EMPTY;
		++incoming_index;
		if(incoming_index >= n_incoming_buffers) incoming_index = 0;
	} else {
		if(index != incoming_index) {
			if(verbose) printf("WARNING: incoming indices out of sync\n");
			return false;
		}
		incoming_buffer_semaphore[index] = BUFFER_EMPTY; /* still do what was requested */
	}
	return true;
}

static uint64_t get_time_ms() {
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	uint64_t seconds = spec.tv_sec;
	uint64_t nano_seconds = spec.tv_nsec;
	uint64_t now = seconds * 1000 + nano_seconds / 1000000;
// printf("now = %"PRIu64". secs = %"PRIu64". nanoseconds = %"PRIu64"\n", now, seconds, nano_seconds);
	return now;
}

void *Camera::get_incoming_buffer(bool wait_ready) {

if(debug) printf("get_incoming_buffer(): index = %d. buffer = %p\n", incoming_index, incoming_buffer[incoming_index]);

	uint64_t now = get_time_ms();
	uint64_t timeout = now + 5000; /* half second */
	while(incoming_buffer_semaphore[incoming_index] == BUFFER_EMPTY) {
		uint64_t now = get_time_ms();
		if(now > timeout) {
			snprintf(logbuff, logbuff_length, "camera:: timeout waiting for incoming buffer");
			if(log_fxn) log_fxn(LEVEL_ERROR, logbuff);
			return 0;
		} else { 
			usleep(sleep_wait);
		}
	} 

	return incoming_buffer[incoming_index];

}

/* get current exposure settings */
bool Camera::get_exposure(unsigned char *value) {

	__u8 temp[128];
	int i;

	temp[0] = 0; /* read */
	temp[1] = 0x30;
	temp[2] = 0x12; 
	temp[3] = 0; 
	temp[4] = 0; 

	struct uvc_xu_control_query xu_query = {
		.unit		= 3,
		.selector	= 0x0e,
		.query		= UVC_SET_CUR,
		.size		= 5,
		.data		= temp,
	};

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) {
		int res = errno;
		const char *err;
		switch(res) {
			case ENOENT:	err = "Extension unit or control not found"; break;
			case ENOBUFS:	err = "Buffer size does not match control size"; break;
			case EINVAL:	err = "Invalid request code"; break;
			case EBADRQC:	err = "Request not supported by control"; break;
			default:	err = strerror(res); break;
		}

		printf("failed %s. (System code: %d) \n\r", err, res);
		return false;
	}

	printf("current exposure setting = [%2.2x, %2.2x, %2.2x, %2.2x, %2.2x]\n",
		temp[0], temp[1], temp[2], temp[3], temp[4]);

	printf("UUID bytes:%d\n", 64);
	for(i=0;i<64;i++) printf("%02x-", temp[i]);
	printf("\n");

	return true;
}

bool Camera::set_exposure(int coarse, int fine) {

/*5 bytes:
byte0:read/write flag,read:0;write:1
byte1:16bits address high
byte2:16bits address low
byte3:16bits value high
byte4:16bits value low

For example:
value[5] = {1,0x30,0x12,0x02,0x00}//this will set 0x3012(corase integration time) to 0x0200,that's 512 row exposure time
value[5] = {1,0x30,0x14,0x02,0x00}//this will set 0x3014(fine integration time) to 0x0200,that's 512/(74.25M),about 6.9us
*/	

	__u8 temp[128];

/*** first display the current value ***/
//	bool stat = get_exposure(fd, temp);

/*** set the desired value ***/

	printf("set_exposure(fd=%d, coarse=%d, fine=%d)\n", fd, coarse, fine);

	temp[0] = 1; /* write */
	temp[1] = 0x30; 
	temp[2] = 0x12; 
	temp[3] = (coarse >> 8) & 0xff; 
	temp[4] = coarse & 0xff; 

	// printf("values=[%2.2x,%2.2x,%2.2x,%2.2x,%2.2x]\n", temp[0], temp[1], temp[2], temp[3], temp[4]);

	struct uvc_xu_control_query xu_control = {
		.unit		= 3,
		.selector	= 0x0e,
		.query		= UVC_SET_CUR,
		.size		= 5,
		.data		= temp,
	};

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_control) != 0) {
		int res = errno;
		const char *err;
		switch(res) {
			case ENOENT:	err = "Extension unit or control not found"; break;
			case ENOBUFS:	err = "Buffer size does not match control size"; break;
			case EINVAL:	err = "Invalid request code"; break;
			case EBADRQC:	err = "Request not supported by control"; break;
			default:	err = strerror(res); break;
		}

		printf("failed %s. (System code: %d) \n\r", err, res);
		return false;
	}

/*** read back the current setting to verify ***/
//	stat = get_exposure(fd, temp);

	return true;

}

int utc_time(char *result, int result_size) {
	time_t nowtime;
	struct tm *ptr_time;
	char buffer[64];
	time(&nowtime); /* get current time */
	ptr_time = localtime(&nowtime); /* localtime returns pointer to tm structure */
/* convert time to specific string */
	strftime(result, result_size, "%Y-%m-%d %H:%M:%S+0000", ptr_time);
	return 0;
}

bool Camera::write_m021_register(unsigned int reg, unsigned int val) {

/* 5 bytes:
	byte0:read/write flag,read:0;write:1
	byte1:16bits address high
	byte2:16bits address low
	byte3:16bits value high
	byte4:16bits value low

	For example:
	value[5] = {1,0x30,0x12,0x02,0x00}//this will set 0x3012(corase integration time) to 0x0200,that's 512 row exposure time
	value[5] = {1,0x30,0x14,0x02,0x00}//this will set 0x3014(fine integration time) to 0x0200,that's 512/(74.25M),about 6.9us
*/	

	__u8 temp[128];

/*** first display the current value ***/
//	bool stat = get_exposure(fd, temp);

/*** set the desired value ***/

	temp[0] = 1; /* write */
	temp[1] = (reg >> 8) & 0xff; 
	temp[2] = (reg & 0xff); 
	temp[3] = (val >> 8) & 0xff; 
	temp[4] = (val & 0xff);

	// printf("values=[%2.2x,%2.2x,%2.2x,%2.2x,%2.2x]\n", temp[0], temp[1], temp[2], temp[3], temp[4]);

	struct uvc_xu_control_query xu_control = {
		.unit		= 3,
		.selector	= 0x0e,
		.query		= UVC_SET_CUR,
		.size		= 5,
		.data		= temp,
	};

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_control) != 0) {
		int res = errno;
		const char *err;
		switch(res) {
			case ENOENT:	err = "Extension unit or control not found"; break;
			case ENOBUFS:	err = "Buffer size does not match control size"; break;
			case EINVAL:	err = "Invalid request code"; break;
			case EBADRQC:	err = "Request not supported by control"; break;
			default:	err = strerror(res); break;
		}

		printf("failed %s. (System code: %d) \n\r", err, res);
		return false;
	}

	return true;

}

bool Camera::read_m021_register(unsigned int reg, unsigned int *val) {

/*5 bytes:
byte0:read/write flag,read:0;write:1
byte1:16bits address high
byte2:16bits address low
byte3:16bits value high
byte4:16bits value low

For example:
value[5] = {1,0x30,0x12,0x02,0x00}//this will set 0x3012(corase integration time) to 0x0200,that's 512 row exposure time
value[5] = {1,0x30,0x14,0x02,0x00}//this will set 0x3014(fine integration time) to 0x0200,that's 512/(74.25M),about 6.9us
*/	

	__u8 temp[128];

/*** first display the current value ***/
//	bool stat = get_exposure(fd, temp);

/*** set the desired value ***/

	temp[0] = 0; /* read */
	temp[1] = (reg >> 8) & 0xff; 
	temp[2] = reg & 0xff; 
	temp[3] = 0x00; 
	temp[4] = 0x00; 

	// printf("values=[%2.2x,%2.2x,%2.2x,%2.2x,%2.2x]\n", temp[0], temp[1], temp[2], temp[3], temp[4]);

	struct uvc_xu_control_query xu_control = {
		.unit		= 3,
		.selector	= 0x0e,
		.query		= UVC_SET_CUR,
		.size		= 5,
		.data		= temp,
	};

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_control) != 0) {
		int res = errno;
		const char *err;
		switch(res) {
			case ENOENT:	err = "Extension unit or control not found"; break;
			case ENOBUFS:	err = "Buffer size does not match control size"; break;
			case EINVAL:	err = "Invalid request code"; break;
			case EBADRQC:	err = "Request not supported by control"; break;
			default:	err = strerror(res); break;
		}

		printf("failed %s. (System code: %d) \n\r", err, res);
		return false;
	}

	xu_control.query = UVC_GET_CUR;

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_control) != 0) {
		int res = errno;
		const char *err;
		switch(res) {
			case ENOENT:	err = "Extension unit or control not found"; break;
			case ENOBUFS:	err = "Buffer size does not match control size"; break;
			case EINVAL:	err = "Invalid request code"; break;
			case EBADRQC:	err = "Request not supported by control"; break;
			default:	err = strerror(res); break;
		}

		printf("failed %s. (System code: %d) \n\r", err, res);
		return false;
	}

	printf("UUID: %2.2x %2.2x %2.2x %2.2x %2.2x\n", temp[0], temp[1], temp[2], temp[3], temp[4]);

	unsigned int x;
	x = temp[3];
	x = (x << 8) + temp[4];
	*val = x; 
/*** read back the current setting to verify ***/
//	stat = get_exposure(fd, temp);

	return true;

}

bool Camera::set_capture_region_of_interest(RegionOfInterest *roi) {
	write_m021_register(M021_X_ADDR_START_REGISTER, roi->start_col); 
	write_m021_register(M021_Y_ADDR_START_REGISTER, roi->start_row); 
	write_m021_register(M021_X_ADDR_END_REGISTER, roi->end_col); 
	write_m021_register(M021_Y_ADDR_END_REGISTER, roi->end_row); 
	return true;
}

bool Camera::set_capture_region_of_interest(int start_col, int start_row, int end_col, int end_row) {
	RegionOfInterest roi;
	roi.start_col = start_col;
	roi.start_row = start_row;
	roi.end_col = end_col;
	roi.end_row = end_row;
	return set_capture_region_of_interest(&roi);
}

bool Camera::set_autoexposure_region_of_interest(RegionOfInterest *roi) {
	int x_size = 1 + roi->end_col - roi->start_col;
	int y_size = 1 + roi->end_row - roi->start_row;
	write_m021_register(M021_AE_ROI_X_START_OFFSET_REGISTER, roi->start_col); 
	write_m021_register(M021_AE_ROI_Y_START_OFFSET_REGISTER, roi->start_row);
	write_m021_register(M021_AE_ROI_X_SIZE_REGISTER, x_size); 
	write_m021_register(M021_AE_ROI_Y_SIZE_REGISTER, y_size); 
	return true;
}

bool Camera::set_autoexposure_region_of_interest(int start_col, int start_row, int end_col, int end_row) {
	RegionOfInterest roi;
	roi.start_col = start_col;
	roi.start_row = start_row;
	roi.end_col = end_col;
	roi.end_row = end_row;
	return set_autoexposure_region_of_interest(&roi);
}

