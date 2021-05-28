QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        ../../src/vcglib_dev/wrap/ply/plylib.cpp \
        cleanup.cpp \
        main.cpp

INCLUDEPATH += \
        E:/OneDrive/Documents/meshlab/src \
        E:/OneDrive/src/vcglib_dev \
        E:/OneDrive/src/vcglib_dev/eigenlib
#        C:/src/vcpkg/installed/x64-windows/include/eigen3 \
#        C:/src/vcpkg/installed/x64-windows/include/

#win32:CONFIG(release, debug|release): LIBS += -L$$C:\src\vcpkg\installed\x64-windows\lib\boost_program_options-vc140-mt.lib
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$C:\src\vcpkg\installed\x64-windows\debug\lib\boost_program_options-vc140-mt-gd



# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    cleanup.h \
    mesh.h


