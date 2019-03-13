TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

*-g++* {
    QMAKE_CXXFLAGS_DEBUG += -DLOGGING -DALT_MEAS
}


SOURCES += main.cpp \
    bme280.cpp \
    i2c.cpp \
    logger.cpp

HEADERS += \
    bme280.h \
    bme280_defs.h \
    i2c.h \
    logger.h \
    macros.h

target.path = /opt/BME280/
INSTALLS += target
