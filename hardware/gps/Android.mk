LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false
LOCAL_SRC_FILES := misc.c minmea.c reader.c device.c gps.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)/include

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        liblog \
	libhardware

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_CFLAGS += -DHAVE_SYS_UIO_H -std=c99

LOCAL_MODULE := gps.default
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_FILENAME := gps.default

include $(BUILD_SHARED_LIBRARY)


