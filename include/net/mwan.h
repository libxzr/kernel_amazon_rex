/*
 * mwan.h  --  Mario WAN hardware control driver
 *
 * Copyright 2005-2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 */

#ifndef __MWAN_H__
#define __MWAN_H__

typedef enum {
	WAN_INVALID  = -1,
	WAN_OFF,     // 0
	WAN_ON,      // 1
	WAN_OFF_KILL,// 2
	WAN_ON_FORCE // 3
} wan_status_t;

extern void wan_set_power_status(wan_status_t);
extern wan_status_t wan_get_power_status(void);

#define MODEM_TYPE_UNKNOWN      (-1)
#define MODEM_TYPE_AD_DTM       (0)
#define MODEM_TYPE_NVTL_EVDO    (1)
#define MODEM_TYPE_NVTL_HSPA    (2)
#define MODEM_TYPE_AD_DTP       (3)
#define MODEM_TYPE_LAB126_ELMO  (4)
#define MODEM_TYPE_USI_WHISTLER (5)
#define MODEM_TYPE_FBC_BANFF    (6)
#define MODEM_TYPE_FBC_SOLDEN   (7)

extern int wan_get_modem_type(void);

typedef enum {
	WAN_DPDT_RELEASE = 0,
	WAN_DPDT_MAIN,
	WAN_DPDT_DIV
} wan_dpdt_state_t;

typedef enum {
	WAN_SPST_RELEASE = 0,
	WAN_SPST_ENABLE,
	WAN_SPST_DISABLE
} wan_spst_state_t;

typedef void (*wan_usb_wake_callback_t)(void *);

extern wan_usb_wake_callback_t usb_wake_callback;
extern void *usb_wake_callback_data;

#endif // __MWAN_H__
