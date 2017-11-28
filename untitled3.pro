TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    xiApiPlusOcv.cpp

HEADERS += xiApiPlusOcv.hpp

# XIMEA
INCLUDEPATH += C:\XIMEA\Examples\Include
LIBS += -LC:\XIMEA\API\x64 \
        -lxiapi64

# opencv 3.0.0
INCLUDEPATH += $$(OPENCV_DIR)\opencv3.3.0.0.1\build\native\include
LIBS += -L$$quote($$(OPENCV_DIR)\opencv3.3.0.0.1\build\native\lib\x64\v120\Release) \
        -lopencv_bgsegm300 \
        -lopencv_bioinspired300 \
        -lopencv_calib3d300 \
        -lopencv_ccalib300 \
        -lopencv_core300 \
        -lopencv_datasets300 \
        -lopencv_face300 \
        -lopencv_features2d300 \
        -lopencv_flann300 \
        -lopencv_highgui300 \
        -lopencv_imgcodecs300 \
        -lopencv_imgproc300 \
        -lopencv_line_descriptor300 \
        -lopencv_ml300 \
        -lopencv_objdetect300 \
        -lopencv_optflow300 \
        -lopencv_photo300 \
        -lopencv_rgbd300 \
        -lopencv_saliency300 \
        -lopencv_shape300 \
        -lopencv_stitching300 \
        -lopencv_superres300 \
        -lopencv_surface_matching300 \
        -lopencv_text300 \
        -lopencv_tracking300 \
        -lopencv_video300 \
        -lopencv_videoio300 \
        -lopencv_videostab300 \
        -lopencv_xfeatures2d300 \
        -lopencv_ximgproc300 \
        -lopencv_xobjdetect300 \
        -lopencv_xphoto300
