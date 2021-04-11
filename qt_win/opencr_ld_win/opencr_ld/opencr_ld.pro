TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += WIN32_BUILD

SOURCES += \
    ../../../msg/msg.c \
    ../../../main.c \
    ../../../opencr_ld.c \
    ../../../serial_win32.c

HEADERS += \
    ../../../opencr_ld.h \
    ../../../serial.h \
    ../../../type.h \
    ../../../msg/mavlink/com/mavlink.h \
    ../../../msg/mavlink/com/mavlink_msg_ack.h \
    ../../../msg/mavlink/com/mavlink_msg_packet.h \
    ../../../msg/mavlink/com/com.h \
    ../../../msg/mavlink/com/testsuite.h \
    ../../../msg/mavlink/com/version.h \
    ../../../msg/mavlink/checksum.h \
    ../../../msg/mavlink/mavlink_conversions.h \
    ../../../msg/mavlink/mavlink_helpers.h \
    ../../../msg/mavlink/mavlink_types.h \
    ../../../msg/mavlink/protocol.h \
    ../../../msg/def.h \
    ../../../msg/def_err.h \
    ../../../msg/msg.h
