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
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <uvc_camera.h>

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
void *imu_daq_looper(void *ext);
void *imu_ana_looper(void *ext);

constexpr uint32_t InvalidCameraFrameIndex = 0xffffffff;
constexpr size_t MaximumTypicalFeatures = 10000; /* TODO: the number of features we expect to see in typical image */
unsigned int width = 1280, height = 960, frame_index = 0, fps = 30;
constexpr unsigned int KeepHistory = 4; /* how many frames back we should keep */
constexpr unsigned int MinimumFeaturesForTracking = 500;

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
    std::vector<cv::Point2f> *eigen_features; /* detected features */
    std::vector<cv::Point2f> *track_fwd_features; /* features tracked from next image */
    std::vector<cv::Point2f> *track_rev_features; /* features tracked from previous image */
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
	bool reverse_image;
	cv::Mat trajectory_mat;
	std::string trajectory_window_name;
	std::string image_window_name;
	cv::Mat global_rotation;
	cv::Mat global_translation;
	double ***acc_pool;
	unsigned int acc_buff_size;
	unsigned int acc_size;
	unsigned int acc_head;
	unsigned int acc_tail;
	unsigned int acc_mask;
	unsigned int image_height;
	unsigned int image_width;
} ThreadParams;

// TODO - where to parse from calib.txt
double kFocalLengthPX = 718.8560;
cv::Point2d kPrinciplePointPX(607.1928, 185.2157);

void featureDetection(const Mat &img_1, vector<Point2f> &points1)	{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

double distance(const std::vector<cv::Point2f> &v1, const std::vector<cv::Point2f> &v2) {
	size_t n = v1.size();
	if ((n == 0) || (n != v2.size())) { return -1.0; }
	std::vector<cv::Point2f>::const_iterator it_v1 = v1.begin(), v1_last = v1.end();
	std::vector<cv::Point2f>::const_iterator it_v2 = v2.begin();
	double acc = 0.0;
	for (; it_v1 != v1_last; ++it_v1, ++it_v2) {
		double x1_prev = it_v1->x, x2_prev = it_v2->x;
		double y1_prev = it_v1->y, y2_prev = it_v2->y;
		double diff_x = x2_prev - x1_prev;
		double diff_y = y2_prev - y1_prev;
		acc = acc + (diff_x * diff_x + diff_y * diff_y);
	}
	return acc / n;
}

void featureTracking(const Mat &img1, const Mat &img2, const vector<Point2f> &prev_eigen_features, vector<Point2f> &prev_tracked_features, vector<Point2f> &next_tracked_features)	{
	vector<float> err;
	Size winSize = Size(21, 21);
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
	std::vector<uchar> status;
	std::vector<cv::Point2f> points;

	calcOpticalFlowPyrLK(img1, img2, prev_eigen_features, points, status, err, winSize, 3, termcrit, 0, 0.001);

//	double a = distance(prev_eigen_features, points);
//	printf(">>> size of feature set = %zu. distance = %f\n", prev_eigen_features.size(), a);
//	int n_asterisks = floor(a / 10000.0);
//	for (int i = 0; i < n_asterisks; ++i) {
//		printf("*");
//	}
//	printf("\n");

	size_t n = status.size();
	prev_tracked_features.clear();
	prev_tracked_features.reserve(n);
	next_tracked_features.clear();
	next_tracked_features.reserve(n);
	std::vector<uchar>::const_iterator it_status, last_status = status.end();
	std::vector<cv::Point2f>::const_iterator it_point = points.begin(); /* points and status have same size */
	if (points.size() != n) { printf("mismatch between points and status\n"); return; }

	for (it_status = status.begin(); it_status != last_status; ++it_status, ++it_point) {
		bool remove = (*it_status == 0) || (it_point->x < 0) || (it_point->y < 0);
		if (remove == false) {
			prev_tracked_features.push_back(*it_point);
			next_tracked_features.push_back(*it_point);
		}
	}
}

int main(int argc, char **argv) {

	char str[1024];
	char logbuff[1024];
	int logbuff_length = sizeof(logbuff);
	int i, nframes = 0, compression_scheme = UvcCamera::COMPRESSION_NONE, compression_quality = 0;
	std::string ifile, cfile, ofile, gfile = "gps.log", fourcc = "", device;
	bool calibrate = false, display = false, verbose = false, reverse_image = false;
	Mat camera_matrix, dist_coeffs;

    width = 1920; height = 1080;
	width = 1280; height = 960;

/* jsv need to implement */
	int n_output_buffers = 3;
    fps = 30;

	for(i=1;i<argc;++i) {
		if(strcmp(argv[i], "-debug") == 0) debug_mode = true;
		else if(strcmp(argv[i], "-verbose") == 0) verbose = true;
		else if(strcmp(argv[i], "--device") == 0) device = argv[++i];
		else if(strcmp(argv[i], "-d") == 0) device = argv[++i];
		else if(strcmp(argv[i], "--height") == 0) height = atoi(argv[++i]);
		else if(strcmp(argv[i], "-h") == 0) height = atoi(argv[++i]);
		else if(strcmp(argv[i], "--width") == 0) width = atoi(argv[++i]);
		else if(strcmp(argv[i], "-w") == 0) width = atoi(argv[++i]);
		else if(strcmp(argv[i], "-c") == 0) cfile = argv[++i];
		else if(strcmp(argv[i], "-i") == 0) ifile = argv[++i];
		else if(strcmp(argv[i], "--calibrate") == 0) cfile = argv[++i];
		else if(strcmp(argv[i], "--fourcc") == 0) fourcc = argv[++i];
		else if(strcmp(argv[i], "-o") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "--output") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "-display") == 0) display = true; 
		else if(strcmp(argv[i], "-fps") == 0) fps = atoi(argv[++i]);
		else if(strcmp(argv[i], "-reverse") == 0) reverse_image = true;
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

	ThreadParams *thread_params = new ThreadParams;
	thread_params->reverse_image = reverse_image;
	thread_params->image_window_name = "main";
	thread_params->trajectory_window_name = "traj";
	thread_params->trajectory_mat = cv::Mat(640, 480, CV_8UC1);
	thread_params->global_rotation = cv::Mat::eye(3, 3, CV_64FC1);
	thread_params->global_translation = cv::Mat::zeros(3, 1, CV_64FC1);

	if (ifile.length() != 0) {
		device = ifile;
		thread_params->image_height = 370;
		thread_params->image_width = 1226;
		height = 0;
		debug_mode = true;
	} else {
		thread_params->image_height = height;
		thread_params->image_width = width;
		debug_mode = false;
	}

	UvcCamera *camera = new UvcCamera(device.c_str(), width, height, logger);
	camera->open();
	camera->frame_timeout_ms = 1000;
	thread_params->camera = camera;

	/* sync to camera */
	if (debug_mode == false) {
		thread_params->image_width = camera->width;
		thread_params->image_height = camera->height;
	}
	width = thread_params->image_width; /* TODO get rid of */
	height = thread_params->image_height; /* TODO get rid of */
//	std::string window_name(thread_params->image_window_name.c_str());
//	std::string window_name(thread_params->trajectory_window_name.c_str());
	initialize_UYVY_to_RGBA();

	int err;
	uint32_t cpu_mask = 1;
	pthread_t camera_thread_id, analysis_thread_id, visualization_thread_id;
	pthread_t imu_daq_thread_id, imu_ana_thread_id;
    cpu_set_t cpu_set;

    thread_params->run = &run;

    err = pthread_create(&camera_thread_id, NULL, camera_looper, thread_params); /* create thread */
    cpu_mask = 0x01;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(camera_thread_id, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&analysis_thread_id, NULL, analysis_looper, thread_params); /* create thread */
    cpu_mask = 0x02;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(analysis_thread_id, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&visualization_thread_id, NULL, visualization_looper, thread_params); /* create thread */
    cpu_mask = 0x04;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(visualization_thread_id, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&imu_daq_thread_id, NULL, imu_daq_looper, thread_params); /* create thread */
    cpu_mask = 0x08;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(imu_daq_thread_id, sizeof(cpu_set_t), &cpu_set);

    err = pthread_create(&imu_ana_thread_id, NULL, imu_ana_looper, thread_params); /* create thread */
    cpu_mask = 0x10;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 32; ++i) { if (cpu_mask & (1 << i)) CPU_SET(i, &cpu_set); }
    err = pthread_setaffinity_np(imu_ana_thread_id, sizeof(cpu_set_t), &cpu_set);

    while (run) {
        usleep(100000);
	}

	camera->close();
	delete camera;

	return 0;
}

void *camera_looper(void *ext) {
	ThreadParams *thread_params = (ThreadParams *) ext;
	UvcCamera *camera = thread_params->camera;
	unsigned long int n_pixels = thread_params->image_width * thread_params->image_height;

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
        frame_data->frame_data->payload = new uint8_t [4 * n_pixels]; /* 2 bytes per pixel in yuyv. overkill TODO? */
        frame_data->frame_data->index = InvalidCameraFrameIndex;
        frame_data->y_data = new uint8_t [n_pixels];
        frame_data->mat = new cv::Mat(thread_params->image_height, thread_params->image_width, CV_8UC1, frame_data->y_data);
        frame_data->eigen_features = new std::vector<cv::Point2f>;
        frame_data->eigen_features->reserve(MaximumTypicalFeatures);
        frame_data->track_fwd_features = new std::vector<cv::Point2f>;
        frame_data->track_fwd_features->reserve(MaximumTypicalFeatures);
        frame_data->track_rev_features = new std::vector<cv::Point2f>;
        frame_data->track_rev_features->reserve(MaximumTypicalFeatures);
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
        	if (debug_mode) {
				memcpy(frame_data->y_data, frame_data->frame_data->payload, camera->width * camera->height);
        	} else {
				yuv422_to_y(frame_data->frame_data->payload, frame_data->y_data, camera->width, camera->height, thread_params->reverse_image); /* convert to grey scale */
			}
			camera->releaseFrame(uvc_frame_index); /* let camera buffers go back into pool */
        } else if (uvc_frame_index < 0) {
        	*thread_params->run = 0;
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
    while (*thread_params->run != 0) {
        unsigned int room = (thread_params->frame_data_head - thread_params->frame_data_tail) % thread_params->frame_data_mask;
        // if (room < KeepHistory) { usleep(1000); continue; }
        if (room < 1) { usleep(1000); continue; }

        FrameData *frame_data = thread_params->frame_data_pool[thread_params->frame_data_tail];
        const cv::Mat &current_image = *frame_data->mat;
        // TODO necessary? frame_data->eigen_features->clear(); /* reset vector */
        // TODO necessary? frame_data->eigen_features->reserve(); /* reserve space in vector */
		featureDetection(current_image, *frame_data->eigen_features);
		*frame_data->track_fwd_features = *frame_data->eigen_features;

        ++n_frames;

        /* fetch previous frame */
        unsigned int previous_index = (thread_params->frame_data_tail - 1) & thread_params->frame_data_mask;
        FrameData *prev_frame_data = thread_params->frame_data_pool[previous_index];
        const cv::Mat &previous_image = *prev_frame_data->mat;

		size_t n_previous_features = prev_frame_data->eigen_features->size(), n_current_features = frame_data->eigen_features->size();
		// printf("numbers of features = %zu(%d) / %zu(%d)\n", n_previous_features, previous_index, n_current_features, thread_params->frame_data_tail);

		if (n_frames > 1) { /* we need at least two frames to get the pipeline working */
            cv::Mat mask, relative_rotation, relative_translation;
            // printf("feature set sizes = %zu/%zu\n", prev_frame_data->eigen_features->size(), frame_data->eigen_features->size());
            bool stand_chance = (frame_data->eigen_features->size() >= MinimumFeaturesForTracking) &&
				(prev_frame_data->eigen_features->size() >= MinimumFeaturesForTracking);
            if (stand_chance) {
				std::vector<cv::Point2f> *prev_track = prev_frame_data->track_fwd_features; /* use track_features to replicate current functionality */
				std::vector<cv::Point2f> *next_track = frame_data->track_rev_features;
				size_t n1 = prev_track->size(), n2 = prev_frame_data->eigen_features->size(), n3 = prev_frame_data->track_fwd_features->size(), n4 = frame_data->track_fwd_features->size();
				featureTracking(previous_image, current_image, *frame_data->eigen_features, *prev_track, *next_track);
                cv::Mat E = findEssentialMat(*prev_track, *next_track, kFocalLengthPX, kPrinciplePointPX, cv::RANSAC, 0.999, 1.0, mask);
                recoverPose(E, *prev_track, *next_track, relative_rotation, relative_translation, kFocalLengthPX, kPrinciplePointPX, mask);

#if 0

                double m[3][3];
				double t[3];
				printf("****** ******\nrelative rotation = \n");
                for (unsigned int i = 0; i < 3; ++i) {
                	for (unsigned int j = 0; j < 3; ++j) {
                		m[i][j] = relative_rotation.at<double>(i, j);
                	}
                	t[i] = relative_translation.at<double>(i);
                	printf("\t%12.8f     %12.8f     %12.8f\n", m[i][0], m[i][1], m[i][2]);
                }

				printf("****** ******\nrelative translation = \n");
				printf("\t%12.8f     %12.8f     %12.8f\n", t[0], t[1], t[2]);

#endif

                double scale = 1.0; /* shukui TODO */

				cv::Mat translation = cv::Mat(3, 1, CV_64FC1);
				cv::Mat rotation = cv::Mat(3, 3, CV_64FC1);
				translation = scale * (thread_params->global_rotation * relative_translation) + thread_params->global_translation;
				rotation = thread_params->global_rotation * relative_rotation;
				thread_params->global_rotation = rotation;
				thread_params->global_translation = translation;
			}
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
            std::vector<cv::Point2f>::const_iterator it_feat, it_last = frame_data->eigen_features->end();
// uncomment to display features
//            for (it_feat = frame_data->eigen_features->begin(); it_feat != it_last; ++it_feat) {
//            	cv::circle(image, *it_feat, 2, cv::Scalar(255, 255, 255), 1);
//            }
            imshow(thread_params->image_window_name, image);

            cv::Point2i center;

            double x = thread_params->global_translation.at<double>(0);
            double y = thread_params->global_translation.at<double>(1);
            double z = thread_params->global_translation.at<double>(2);
            printf("translation = (%f, %f, %f)\n", x, y, z);

            center.x = (int) x + 320; /* TODO */
            center.y = (int) y + 240; /* TODO */
            cv::circle(thread_params->trajectory_mat, center, 4, cv::Scalar(255, 0, 0), 1);

            imshow(thread_params->trajectory_window_name, thread_params->trajectory_mat);
            cv::waitKey(30);
            thread_params->display_busy = false;
        }
    }
    return NULL;
}

void *imu_daq_looper(void *ext) {
#if 0
    ThreadParams *thread_params = (ThreadParams *) ext;
    thread_params->acc_buff_size = 128; /* 128 samples in each buff */
    thread_params->acc_size = 64;
    thread_params->acc_pool = new double ** [thread_params->acc_size];
    for (unsigned int i = 0; i < thread_params->acc_size; ++i) {
        thread_params->acc_pool[i] = new double * []
    }


    thread_params->acc_pool = new double ** [thread_params->acc_buff_size];
    while (*thread_params->run != 0) {
        usleep(1000);
    }
#endif
}

void *imu_ana_looper(void *ext) {
#if 0
    ThreadParams *thread_params = (ThreadParams *) ext;
    double *v_new, *v_old;
    double v0[3], v1[3];
    v_new = v0;
    v_old = v1;
    while (*thread_params->run != 0) {
        usleep(1000);
        unsigned int n = 0;
        double *vp = v_old;
        v_old = v_new;
        v_new = vp;
        // v_new[0] = 0.5 * accX;
        for (unsigned int i = 0; i < n; ++i) {
        }
    }
#endif
}
