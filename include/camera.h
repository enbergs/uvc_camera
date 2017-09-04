#ifndef CAMERA_H
#define CAMERA_H

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

using namespace cv;
using namespace std;

enum {
	LEVEL_WARNING,
	LEVEL_INFO,
	LEVEL_ERROR,
	LEVEL_FATAL
};

/* forward declarations */
class Camera;

typedef struct {
	int fd;
	int n_buffers;
	unsigned char **capture_buffer; /* this is hardware/driver related */
	// jsv int n_capture_buffers;
	unsigned char **buffers;
	unsigned int *semaphore;
	uint64_t *frame_capture_timestamp;
	int *frame_capture_index;
	int *bufflen;
	Camera *camera;
	int thread_started, run;
	int sleep_wait; /* how long to sleep between polling, in useconds */
	char *logbuff;
	int logbuff_length;
	unsigned int cpu_mask;
	bool *device_status;
} CameraStreamerParams;

typedef struct {
	const char *ofile;
	int n_buffers;
	unsigned char **buffers;
	unsigned int *semaphore;
	int *bufflen;
	Camera *camera;
	int thread_started, run;
	int sleep_wait; /* how long to sleep between polling, in useconds */
	char *logbuff;
	int logbuff_length;
	unsigned int cpu_mask;
} DiskStreamerParams;

typedef struct {
	int cols, rows, start_row, end_row, start_col, end_col; 
} RegionOfInterest;

typedef bool (CameraCallbackFxn)(class Camera *camera, unsigned char *data, void *ext);
typedef bool (CameraLogFxn)(int level, const char *str);

typedef struct {
	CameraCallbackFxn *fxn;
	void *ext;
} CameraCallbackParams;

class Camera {

	public:

	enum {
		CAMERA_STATE_IDLE = 0,
		CAMERA_STATE_RUNNING,
		CAMERA_STATE_STOPPED
	};

	enum {
		COMPRESSION_PARAMS_QUALITY = 0
	};

	enum {
		BUFFER_EMPTY = 0,
		BUFFER_FULL = 1
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

	enum {
		LEVEL_DEBUG = 0,
		LEVEL_INFO,
		LEVEL_WARNING,
		LEVEL_ERROR,
	};

	Camera(int device, int width, int height, CameraLogFxn *log_fxn = 0, int user_specified_type = -1);
	~Camera();
	bool init();
	bool close();
	bool get_exposure(unsigned char *value);
	bool set_exposure(int coarse, int fine);
	CameraLogFxn *log_fxn;
	void *capture_frame(bool wait_ready = false);
	bool print_caps();
	// int init_mmap();
	bool stop_capture();
	bool start_capture();
	inline bool get_device_status() { return device_status; };
	inline int get_camera_state() { return camera_state; };
	inline int get_height() { return height; };
	inline int get_width() { return width; };
	inline int get_frame_size() { return frame_size; };
	inline int get_bytes_per_pixel() { return bytes_per_pixel; };
	inline int get_type() { return type; };
	inline int get_compression_scheme() { return compression_scheme; };
	inline int get_compression_params(int which) { 
		switch(which) {
		case COMPRESSION_PARAMS_QUALITY:
			return compression_quality;
		default:
			return -1;
		}
	};
	inline uint64_t get_incoming_buffer_timestamp() { return frame_capture_timestamp[incoming_index]; }
	inline int get_incoming_buffer_index() { return frame_capture_index[incoming_index]; }

	inline bool set_fourcc(const char *fourcc) { memcpy(this->fourcc, fourcc, 4); return true; }; 
	inline void set_verbose(bool verbose) { this->verbose = verbose; };
	inline bool get_calibrate() { return calibrate; };
	inline bool get_verbose() { return verbose; };
	bool set_output_file(const char *filename);
	bool set_compression(int scheme, std::vector<int> const &params);
	bool write_m021_register(unsigned int reg, unsigned int val);
	bool read_m021_register(unsigned int reg, unsigned int *val);
	bool configure_output_streaming(int n_buffers, int size, unsigned int cpu_mask = 0);
	bool configure_input_streaming(int n_buffers, int size, unsigned int cpu_mask = 0);
	bool register_callback(CameraCallbackFxn *fxn, void *ext);
	bool compress_frame(unsigned char *ibuff, std::vector<uchar> &obuff);
	void *get_incoming_buffer(bool wait_ready = false);
	void *get_outgoing_buffer(bool wait_ready = false);
	bool release_outgoing_buffer(int n_write, int index = -1);
	bool release_incoming_buffer(int index = -1);
	inline int get_incoming_buffer_size() { return incoming_buffer_size[incoming_index]; }
	inline int get_outgoing_buffer_size() { return outgoing_buffer_size[outgoing_index]; }
	inline int get_incoming_buffer_length() { return incoming_buffer_length[incoming_index]; }
	inline int get_outgoing_buffer_length() { return outgoing_buffer_length[outgoing_index]; }
	int get_n_capture_buffers() { return n_capture_buffers; }
	int get_n_incoming_buffers() { return n_incoming_buffers; }
	bool get_incoming_streaming() { return incoming_streaming; };
	bool get_outgoing_streaming() { return outgoing_streaming; };
	bool set_capture_region_of_interest(RegionOfInterest *roi);
	bool set_capture_region_of_interest(int start_row, int start_col, int end_row, int end_col);
	bool set_autoexposure_region_of_interest(RegionOfInterest *roi);
	bool set_autoexposure_region_of_interest(int start_row, int start_col, int end_row, int end_col);
	inline bool color() { return is_color; };

/* yes. these are exposed as public; deal with it.
   we are all supposed to be experts here. if you feel uncomfortable with the next lines,
   put down the keyboard, close your laptop screen, slowly take 3 steps back and run for help */
	DiskStreamerParams disk_streamer_params;
	CameraStreamerParams camera_streamer_params;
	bool debug;
	bool device_status;

	int frame_index;
	std::vector<CameraCallbackParams> callback_params;
	Mat camera_matrix, dist_coeffs;

	private:
	uint64_t *frame_capture_timestamp;
	int *frame_capture_index;
	bool incoming_streaming, outgoing_streaming;
	int compression_scheme, compression_quality;
	std::string display_window_name; /* name of optional output window */
/* for reasons due to running in different threads, 
   it's better that ofile be *our* private array, not some std::string */
	char *ofile;
	char fourcc[4];
	int coarse_integration, gain;
	bool calibrate, display, verbose;
	int fd, type, fps, frame_size, bytes_per_pixel;
	int incoming_index, outgoing_index, osize, isize;
	// int fill_index, capture_index;
	bool is_color;
	unsigned char *temp_buffer;
    uint32_t width, height;

/* for streaming data from camera */
	unsigned int incoming_streamer_cpu_mask;
	unsigned int n_capture_buffers;
	unsigned char **capture_buffer;
	int *capture_length;
	VideoWriter *writer;

/* for streaming data to disk */
	unsigned int outgoing_streamer_cpu_mask;
	unsigned char **outgoing_buffer, **incoming_buffer;
	unsigned int *outgoing_buffer_semaphore, *incoming_buffer_semaphore;
	int *outgoing_buffer_size, *incoming_buffer_size;
	int *outgoing_buffer_length, *incoming_buffer_length;
	char *logbuff, *disk_streamer_logbuff, *camera_streamer_logbuff;
	int disk_streamer_logbuff_length, camera_streamer_logbuff_length;
	size_t logbuff_length;

	int n_outgoing_buffers, n_incoming_buffers;

	char str[1024]; /* general purpose */

	int camera_state;

	enum {
		FromCameraThreadId = 0,
		ToDiskThreadId,
		NThreadIds
	} ThreadIds;

	pthread_t *tid;

	int sleep_wait;

};

/* M021 camera registers */

#define M021_Y_ADDR_START_REGISTER 0x3002
#define M021_X_ADDR_START_REGISTER 0x3004
#define M021_Y_ADDR_END_REGISTER 0x3006
#define M021_X_ADDR_END_REGISTER 0x3008
#define M021_FRAME_LENGTH_LINES_REGISTER 0x300A
#define M021_FRAME_LINE_LENGTH_PCK_REGISTER 0x300C
#define M021_EXPOSURE_REGISTER 0x3012
#define M021_DIGITAL_GAIN_REGISTER  0x0305E
#define M021_ANALOG_GAIN_REGISTER 0x30B0
#define M021_RESET_REGISTER 0x0301A
/* format for digital gain: xxx.yyyyy where 0b00100000 is 1x gain, 0b00110000 is 1.5x */
/* digital gain varies from 1 to 7.97 */
/* step size is 0.01325 for yyyyy, step size for xxx is 1 */
/* analog gain is bits 5:4 - can be set to 1, 2, 4, 8x */
#define M021_AE_CTRL_REGISTER 0x3100
/* jsv. who is using the following define statement? wrong name. look at line above */
#define M021_AE_ENABLE_REGISTER 0x3100
#define M021_AE_MAX_EXPOSURE_REGISTER 0x311C
#define M021_AE_MIN_EXPOSURE_REGISTER 0x311E
#define M021_AE_ROI_X_START_OFFSET_REGISTER 0x3140
#define M021_AE_ROI_Y_START_OFFSET_REGISTER 0x3142
#define M021_AE_ROI_X_SIZE_REGISTER 0x3144
#define M021_AE_ROI_Y_SIZE_REGISTER 0x3146
#define M021_AE_MEAN_I_REGISTER 0x3152

#endif
