
find_package(OpenCV REQUIRED)
VisCore_add_module(core
EXTERNAL ${OpenCV_LIBS})
