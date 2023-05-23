// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023 NXP
 */
#ifndef __DISPLAY_RPMSG_H__
#define __DISPLAY_RPMSG_H__

#define LPD_FB_FORMAT_NONE 		0x00
#define LPD_FB_FORMAT_RGB565 		0x01
#define LPD_FB_FORMAT_XRGB888 		0x02

#define LPD_FB_MOD_LINEAR 		0x00
#define LPD_FB_MOD_TILE4x4 		0x01

#define LPD_DRM_CALLBACK_PM 		0x01
#define LPD_DRM_CALLBACK_APP 		0x02

#define LPD_DEVICE_POWER_ON 		0x01
#define LPD_DEVICE_POWER_OFF 		0x02

// application domain power state
enum apd_power_state {
        DISP_APD_SUSPEND = 0U,
        DISP_APD_RESUME,
        DISP_APD_REBOOT,
        DISP_APD_SHUTDOWN,
        DISP_APD_BLANK,
        DISP_APD_UNBLANK,
};

enum apd_application_cmd {
	DISP_APP_SHOW_UI,
	DISP_APP_OPEN_APK,
};

typedef int (*lpd_drm_power_t)(struct device *dev, int state);
typedef int (*lpd_drm_application_t)(struct device *dev, int cmd, void *para);


extern int lpd_display_register(struct device *dev, bool enable);
extern int lpd_api_register(int func_id, void *cb);
extern int lpd_update_display_mode(u16 width, u16 height, u16 format, u16 modifier);
extern int lpd_send_power_state(unsigned char state);

#endif
