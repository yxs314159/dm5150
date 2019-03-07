#ifndef __VEHICLE_AD_DM5150_H__
#define __VEHICLE_AD_DM5150_H__

extern struct ad_dev *g_addev;

int dm5150_ad_init(struct ad_dev *ad);
int dm5150_ad_deinit(void);
int dm5150_ad_get_cfg(struct vehicle_cfg **cfg);
void dm5150_ad_check_cif_error(struct ad_dev *ad, int last_line);
int dm5150_check_id(struct ad_dev *ad);

#endif

