cmake_minimum_required(VERSION 3.28)
project(OpenCV)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_executable(OpenCV main.cpp)
target_link_libraries(OpenCV ${OpenCV_LIBS})


#Features
add_executable(HarrisCornerDetector Features/HarrisCornerDetector.cpp)
target_link_libraries(HarrisCornerDetector ${OpenCV_LIBS})

add_executable(Shi-TomasiCornerDetector Features/Shi-TomasiCornerDetector.cpp)
target_link_libraries(Shi-TomasiCornerDetector ${OpenCV_LIBS})

add_executable(CreatingCornerDetector Features/CreatingCornerDetector.cpp)
target_link_libraries(CreatingCornerDetector ${OpenCV_LIBS})

add_executable(DetectingCornersLocationInSubpixels Features/DetectingCornersLocationInSubpixels.cpp)
target_link_libraries(DetectingCornersLocationInSubpixels ${OpenCV_LIBS})

add_executable(FeatureDetection Features/FeatureDetection.cpp)
target_link_libraries(FeatureDetection ${OpenCV_LIBS})

add_executable(FeatureDescription Features/FeatureDescription.cpp)
target_link_libraries(FeatureDescription ${OpenCV_LIBS})

add_executable(FeatureMatchingWithFLANN Features/FeatureMatchingWithFLANN.cpp)
target_link_libraries(FeatureMatchingWithFLANN ${OpenCV_LIBS})

add_executable(Features2D+HomographyFindObject Features/Features2D+HomographyFindObject.cpp)
target_link_libraries(Features2D+HomographyFindObject ${OpenCV_LIBS})

add_executable(AKAZELocalFeaturesMatching Features/AKAZELocalFeaturesMatching.cpp)
target_link_libraries(AKAZELocalFeaturesMatching ${OpenCV_LIBS})

