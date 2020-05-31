/*
 * Copyright (c) 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 */
#ifndef __LINUX_FRONTLIGHT_H
#define __LINUX_FRONTLIGHT_H

#include <linux/backlight.h>

#define FL_MAGIC_NUMBER         'L'
#define FL_IOCTL_SET_INTENSITY  _IOW(FL_MAGIC_NUMBER, 0x01, int)
#define FL_IOCTL_GET_INTENSITY  _IOR(FL_MAGIC_NUMBER, 0x02, int)
#define FL_IOCTL_GET_RANGE_MAX  _IOR(FL_MAGIC_NUMBER, 0x03, int)
#define FL_IOCTL_SET_INTENSITY_FORCED  _IOW(FL_MAGIC_NUMBER, 0x04, int)

#endif

extern int frontlight_register(struct backlight_device *device);
