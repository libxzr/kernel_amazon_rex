#ifndef __CYTTSP5_IO_H__
#define __CYTTSP5_IO_H__

#include <linux/lab126_touch.h>

struct cyttsp5_grip_suppression_data {
	uint16_t xa;
	uint16_t xb;
	uint16_t xexa;
	uint16_t xexb;
	uint16_t ya;
	uint16_t yb;
	uint16_t yexa;
	uint16_t yexb;
};

#define CY5_IOCTL_GRIP_SET_DATA		_IOW(TOUCH_MAGIC_NUMBER, 0x30, struct cyttsp5_grip_suppression_data)
#define CY5_IOCTL_GRIP_GET_DATA		_IOR(TOUCH_MAGIC_NUMBER, 0x31, struct cyttsp5_grip_suppression_data)


#endif //__CYTTSP5_IO_H__
