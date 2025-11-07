#ifndef __LINUX_FT_GESTURE_LIB_H__
#define __LINUX_FT_GESTURE_LIB_H__

int fetch_object_sample(unsigned char *buf, short pointnum);

void init_para(int x_pixel, int y_pixel, int time_slot, int cut_x_pixel,
	       int cut_y_pixel);

#endif
