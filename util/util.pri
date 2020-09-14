HEADERS += util/Timer.h \
    util/StopWatch.h \
    util/VecN.h \
    util/VecNi.h \
    util/VecNu.h \
    util/Vec3.h \
    util/Vec2.h \
    util/Vec3i.h \
    util/Vec2i.h \
    util/Vec3u.h \
    util/Vec2u.h \
    util/Logger.h \
    util/Statistics.h \
    util/ColorUtil.h \
    util/PriorityQueue.h \
    util/LinkedList.h \
    util/Vector.h \
    util/AdjacencyMatrix.h \
    util/GLlib.h \
    util/Transform3D.h
SOURCES += \
    util/StopWatch.cpp \
    util/Timer.cpp \
    util/Logger.cpp \
    util/Statistics.cpp \
    util/ColorUtil.cpp \
    util/AdjacencyMatrix.cpp \
    util/GLlib.cpp \
    util/Transform3D.cpp
win32:HEADERS += util/TimerWindows.h
win32:SOURCES += util/TimerWindows.cpp
win32:HEADERS += util/StopWatchWindows.h
win32:SOURCES += util/StopWatchWindows.cpp
linux:HEADERS += util/TimerLinux.h
linux:SOURCES += util/TimerLinux.cpp
linux:HEADERS += util/StopWatchLinux.h
linux:SOURCES += util/StopWatchLinux.cpp
