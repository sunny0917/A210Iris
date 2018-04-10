LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := A210Iris
LOCAL_SRC_FILES := serial.c \
					A210Iris.c \
					com_task.c \

LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -llog
  
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)
