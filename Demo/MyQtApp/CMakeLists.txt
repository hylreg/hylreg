cmake_minimum_required(VERSION 3.16)

project(Test VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)
#set(QT_ASSUME_STDERR_HAS_CONSOLE ON)


find_package(Qt6 6.5 REQUIRED COMPONENTS Quick Widgets)
find_package(OpenCV REQUIRED)

qt_standard_project_setup(REQUIRES 6.5)

qt_add_executable(appTest
    main.cpp
        imageProvider.cpp
        imageProvider.h

)

qt_add_qml_module(appTest
    URI Test
    VERSION 1.0
    QML_FILES
        Main.qml
        RESOURCES resource.qrc
        SOURCES imageprovider.h imageprovider.cpp
        SOURCES model.h model.cpp
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appTest PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appTest
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appTest
    PRIVATE Qt6::Quick
    Qt6::Widgets
        ${OpenCV_LIBS}
)

include(GNUInstallDirs)
install(TARGETS appTest
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
