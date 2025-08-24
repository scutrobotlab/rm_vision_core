# 获取 Eigen 库路径
find_package(Eigen3 REQUIRED)

VisCore_add_module(rune_group
DEPENDS rune_tracker) 