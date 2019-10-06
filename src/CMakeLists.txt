find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


#add_executable(read read.cpp)
#add_executable(main main.cpp)
add_executable(main main.cpp read.cpp)

target_link_libraries(main Eigen3::Eigen ${OpenCV_LIBS})