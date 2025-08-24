find_package(OpenCV REQUIRED)

VisCore_add_exe(rune_detect_demo
DEPENDS rune_detector
EXTERNAL ${OpenCV_LIBS}
)