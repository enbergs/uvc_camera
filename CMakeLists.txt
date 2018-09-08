cmake_minimum_required(VERSION 3.5)
project(test_uvc_camera)

find_package(OpenCV 3.0 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wall -std=c++11 -pthread -m64 -lm -ldl -rdynamic -lpthread -Wl,-rpath,$(XSENSSYS)/lib")

# set(CMAKE_CXX_STANDARD 11)
set(PROJECT_ROOT $ENV{HOME}/projects/uvc_camera)
include_directories(${PROJECT_ROOT}/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(SOURCE_FILES src/colorspaces.cpp src/uvc_camera.cpp)

add_executable(test_uvc_camera examples/test_uvc_camera.cpp ${SOURCE_FILES})
add_executable(pose_calibration_uvc_camera examples/pose_calibration_uvc_camera.cpp ${SOURCE_FILES})
# target_link_libraries(test_camera ${OpenCV_LIBS})
# target_link_libraries(test_camera /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so -lopencv_calib3d /usr/lib/x86_64-linux-gnu/libopencv_contrib.so -lopencv_contrib /usr/lib/x86_64-linux-gnu/libopencv_core.so -lopencv_core /usr/lib/x86_64-linux-gnu/libopencv_features2d.so -lopencv_features2d /usr/lib/x86_64-linux-gnu/libopencv_flann.so -lopencv_flann /usr/lib/x86_64-linux-gnu/libopencv_gpu.so -lopencv_gpu /usr/lib/x86_64-linux-gnu/libopencv_highgui.so -lopencv_highgui /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so -lopencv_imgproc /usr/lib/x86_64-linux-gnu/libopencv_legacy.so -lopencv_legacy /usr/lib/x86_64-linux-gnu/libopencv_ml.so -lopencv_ml /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so -lopencv_objdetect /usr/lib/x86_64-linux-gnu/libopencv_ocl.so -lopencv_ocl /usr/lib/x86_64-linux-gnu/libopencv_photo.so -lopencv_photo /usr/lib/x86_64-linux-gnu/libopencv_stitching.so -lopencv_stitching /usr/lib/x86_64-linux-gnu/libopencv_superres.so -lopencv_superres /usr/lib/x86_64-linux-gnu/libopencv_ts.so -lopencv_ts /usr/lib/x86_64-linux-gnu/libopencv_video.so -lopencv_video /usr/lib/x86_64-linux-gnu/libopencv_videostab.so -lopencv_videostab)
# target_link_libraries(test_uvc_camera -L/usr/local/lib -L/usr/local/cuda/lib64 -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lcufft -lnpps -lnppi -lnppc -lcudart -lrt -lpthread -lm -ldl)
target_link_libraries(test_uvc_camera ${OpenCV_LIBS})
target_link_libraries(pose_calibration_uvc_camera ${OpenCV_LIBS})
