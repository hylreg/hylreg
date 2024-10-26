cmake_minimum_required(VERSION 3.16)

project(MyVision VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)

set(ONNXRUNTIME_ROOT "D:/Libraries/onnxruntime-win-x64-gpu-1.16.3")
if(WIN32)
    message(STATUS "This is Windows")
    set(ONNXRUNTIME_ROOT "D:/Libraries/onnxruntime-win-x64-gpu-1.16.3")

    set(ONNXRUNTIME_LIBS ${ONNXRUNTIME_ROOT}/lib/onnxruntime.lib)

elseif(UNIX)
    message(STATUS "This is Unix-like OS")
    if(APPLE)
        message(STATUS "This is macOS")
    else()
        message(STATUS "This is Linux")
        set(ONNXRUNTIME_ROOT "/home/hyl/Libraries/onnxruntime-linux-x64-gpu-1.16.3")
        set(ONNXRUNTIME_LIBS ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so)
        set(CMAKE_PREFIX_PATH "/home/hyl/Programs/Qt/6.8.0/gcc_64" )
    endif()
else()
    message(STATUS "Unknown OS")
endif()


include_directories(ORT ${ONNXRUNTIME_ROOT}/include)

find_package(OpenCV REQUIRED)
find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickControls2)

qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(appMyVision
    main.cpp
)

qt_add_qml_module(appMyVision
    URI MyVision
    VERSION 1.0
    QML_FILES
        Main.qml
        QML_FILES MyModule/MyComponent.qml
        RESOURCES res.qrc
        SOURCES vision.h vision.cpp
        # SOURCES videoframeprovider.h videoframeprovider.cpp
        # SOURCES worker.h worker.cpp
        SOURCES threadmanager.h threadmanager.cpp
        SOURCES cameraworker.h cameraworker.cpp
        SOURCES imageprovider.h imageprovider.cpp
        SOURCES modelmanager.h modelmanager.cpp
        SOURCES settingsmanager.h settingsmanager.cpp
        SOURCES model.h model.cpp
        SOURCES yolo_v8.h yolo_v8.cpp
        SOURCES YOLOv8-ONNXRuntime-CPP/inference.cpp YOLOv8-ONNXRuntime-CPP/inference.h
RESOURCES YOLOv8-ONNXRuntime-CPP/classes.txt YOLOv8-ONNXRuntime-CPP/CMakeLists.txt YOLOv8-ONNXRuntime-CPP/coco8.yaml YOLOv8-ONNXRuntime-CPP/README.md YOLOv8-ONNXRuntime-CPP/yolov8n.onnx YOLOv8-ONNXRuntime-CPP/yolov8n.pt
        # SOURCES mythread.h mythread.cpp

)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appMyVision PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appMyVision
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appMyVision
    PRIVATE
    Qt6::Quick
    Qt6::QuickControls2
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIBS}
)

include(GNUInstallDirs)
install(TARGETS appMyVision
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
add_custom_command(TARGET appMyVision POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/images
)
