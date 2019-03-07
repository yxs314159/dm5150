#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <mach/board.h>
#include <plat/rk_camera.h>

#include "vehicle_ad.h"
#include "vehicle_ad_7181.h"
#include "vehicle_ad_tp2825.h"
#include "vehicle_ad_tvp5150.h"
#include "vehicle_ad_dm5886.h"
#include "vehicle_ad_dm5150.h"

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

struct ad_dev *g_addev;

struct vehicle_sensor_ops {
	const char *name;

	int (*sensor_init)(struct ad_dev *ad);
	int (*sensor_deinit)(void);
	int (*sensor_get_cfg)(struct vehicle_cfg **cfg);
	void (*sensor_check_cif_error)(struct ad_dev *ad, int last_line);
	int (*sensor_check_id_cb)(struct ad_dev *ad);
};
struct vehicle_sensor_ops *sensor_cb;

static struct vehicle_sensor_ops sensor_cb_series[] = {
/*	{
		.name = "adv7181",
		.sensor_init = adv7181_ad_init,
		.sensor_deinit = adv7181_ad_deinit,
		.sensor_get_cfg = adv7181_ad_get_cfg,
		.sensor_check_cif_error = adv7181_ad_check_cif_error,
		.sensor_check_id_cb = adv7181_check_id
	},
	{
		.name = STR(RK29_CAM_SENSOR_TP2825),
		.sensor_init = tp2825_ad_init,
		.sensor_deinit = tp2825_ad_deinit,
		.sensor_get_cfg = tp2825_ad_get_cfg,
		.sensor_check_cif_error = tp2825_ad_check_cif_error,
		.sensor_check_id_cb = tp2825_check_id
	},
*/	
#ifdef CONFIG_SOC_CAMERA_DM5886
	{
		.name = STR(RK29_CAM_SENSOR_DM5886),
		.sensor_init = dm5886_ad_init,
		.sensor_deinit = dm5886_ad_deinit,
		.sensor_get_cfg = dm5886_ad_get_cfg,
		.sensor_check_cif_error = dm5886_ad_check_cif_error,
		.sensor_check_id_cb = dm5886_check_id
	},
#else
#ifdef CONFIG_SOC_CAMERA_DM5150
	{
		.name = STR(RK29_CAM_SENSOR_DM5150),
		.sensor_init = dm5150_ad_init,
		.sensor_deinit = dm5150_ad_deinit,
		.sensor_get_cfg = dm5150_ad_get_cfg,
		.sensor_check_cif_error = dm5150_ad_check_cif_error,
		.sensor_check_id_cb = dm5150_check_id
	},
#endif
	{
		.name = STR(RK29_CAM_SENSOR_TVP5150),
		.sensor_init = tvp5150_ad_init,
		.sensor_deinit = tvp5150_ad_deinit,
		.sensor_get_cfg = tvp5150_ad_get_cfg,
		.sensor_check_cif_error = tvp5150_ad_check_cif_error,
		.sensor_check_id_cb = tvp5150_check_id
	},
#endif
};

int vehicle_generic_sensor_write(struct ad_dev *ad, char reg, char *pval)
{
	struct i2c_msg msg;
	int ret;

	char *tx_buf = kmalloc(2, GFP_KERNEL);

	if (!tx_buf)
		return -ENOMEM;

	memcpy(tx_buf, &reg, 1);
	memcpy(tx_buf+1, (char *)pval, 1);

	msg.addr = ad->i2c_add;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = (char *)tx_buf;
	msg.scl_rate = ad->i2c_rate;

	ret = i2c_transfer(ad->adapter, &msg, 1);
	kfree(tx_buf);

	return (ret == 1) ? 4 : ret;
}

int vehicle_generic_sensor_read(struct ad_dev *ad, char reg)
{
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf[2];
	char pval;

	memcpy(reg_buf, &reg, 1);

	msgs[0].addr =	ad->i2c_add;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = reg_buf;
	msgs[0].scl_rate = ad->i2c_rate;

	msgs[1].addr = ad->i2c_add;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &pval;
	msgs[1].scl_rate = ad->i2c_rate;

	ret = i2c_transfer(ad->adapter, msgs, 2);

	return pval;
}

#ifdef CONFIG_OF
int vehicle_parse_sensor(struct ad_dev *ad)
{
	struct device *dev = ad->dev;
	struct device_node *node = NULL;
	struct device_node *cp = NULL;
	enum of_gpio_flags flags;
	const char *status = NULL;
	int i;
	int ret = 0;

	if (of_property_read_u32(dev->of_node, "ad,fix-format",
				 &ad->fix_format))
		DBG("get fix-format failed!\n");

	if (of_property_read_u32(dev->of_node, "vehicle,rotate-mirror",
				 &ad->cfg.rotate_mirror))
		DBG("get rotate-mirror failed!\n");

	node = of_parse_phandle(dev->of_node, "rockchip,cif-sensor", 0);
	if (!node) {
		DBG("get cif-sensor dts failed\n");
		return -1;
	}

	for_each_child_of_node(node, cp) {
		of_property_read_string(cp, "status", &status);
		DBG("status: %s\n", status);
		if (status && strcmp(status, "okay"))
			continue;

		if (of_property_read_u32(cp, "i2c_rata", &ad->i2c_rate))
			DBG("Get %s i2c_rata failed!\n", cp->name);
		if (of_property_read_u32(cp, "i2c_chl", &ad->i2c_chl))
			DBG("Get %s i2c_chl failed!\n", cp->name);
		if (of_property_read_u32(cp, "ad_chl", &ad->ad_chl))
			DBG("Get %s ad_chl failed!\n", cp->name);

		if (ad->ad_chl > 4 || ad->ad_chl < 0) {
			DBG("error, ad_chl %d !\n", ad->ad_chl);
			ad->ad_chl = 0;
		}
		if (of_property_read_u32(cp, "mclk_rate", &ad->mclk_rate))
			DBG("Get %s mclk_rate failed!\n", cp->name);

		if (of_property_read_u32(cp, "pwr_active", &ad->pwr_active))
			DBG("Get %s pwdn_active failed!\n", cp->name);

		if (of_property_read_u32(cp, "pwdn_active", &ad->pwdn_active))
			DBG("Get %s pwdn_active failed!\n", cp->name);

		ad->power = of_get_named_gpio_flags(cp, "rockchip,power",
						    0, &flags);
		ad->powerdown = of_get_named_gpio_flags(cp,
							"rockchip,powerdown",
							0, &flags);

		if (of_property_read_u32(cp, "i2c_add", &ad->i2c_add))
			DBG("Get %s i2c_add failed!\n", cp->name);

		ad->i2c_add = (ad->i2c_add >> 1);

		if (of_property_read_u32(cp, "resolution", &ad->resolution))
			DBG("Get %s resolution failed!\n", cp->name);

		of_property_read_u32_array(
				cp,
				"rockchip,camera-module-defrect0",
				(unsigned int *)&ad->defrects[0],
				6);
		of_property_read_u32_array(
				cp,
				"rockchip,camera-module-defrect1",
				(unsigned int *)&ad->defrects[1],
				6);
		of_property_read_u32_array(
				cp,
				"rockchip,camera-module-defrect2",
				(unsigned int *)&ad->defrects[2],
				6);
		of_property_read_u32_array(
				cp,
				"rockchip,camera-module-defrect3",
				(unsigned int *)&ad->defrects[3],
				6);

		of_property_read_string(
				cp,
				"rockchip,camera-module-interface0",
				&ad->defrects[0].interface);
		of_property_read_string(
				cp,
				"rockchip,camera-module-interface1",
				&ad->defrects[1].interface);
		of_property_read_string(
				cp,
				"rockchip,camera-module-interface2",
				&ad->defrects[2].interface);
		of_property_read_string(
				cp,
				"rockchip,camera-module-interface3",
				&ad->defrects[3].interface);

		ad->ad_name = cp->name;
		for (i = 0; i < ARRAY_SIZE(sensor_cb_series); i++) {
			if (!strcmp(ad->ad_name, sensor_cb_series[i].name))
				sensor_cb = sensor_cb_series + i;
		}

		DBG("%s: ad_chl=%d,,ad_addr=%x,fix_for=%d\n", ad->ad_name,
		    ad->ad_chl, ad->i2c_add, ad->fix_format);
		DBG("gpio power:%d, active:%d\n", ad->power, ad->pwr_active);
		DBG("gpio powerdown:%d, active:%d\n",
		    ad->powerdown, ad->pwdn_active);
		break;
	}
	if (!ad->ad_name)
		ret = -EINVAL;

	return ret;
}
#else

bool vehicle_sensor_check(struct ad_dev *ad, struct rkcamera_platform_data *new_camera)
{
	int i;
	int ret;

	DBG("new camera: %s\n", new_camera->dev.device_info.dev.init_name);

	/*find sensor ops*/
	sensor_cb = NULL;
	for (i = 0; i < ARRAY_SIZE(sensor_cb_series); i++) {
		DBG("sensor_cb: %s\n", sensor_cb_series[i].name);
		if (strstr(new_camera->dev.device_info.dev.init_name, sensor_cb_series[i].name)) {
			sensor_cb = sensor_cb_series + i;
			break;
		}
	}
	if (!sensor_cb) {
		DBG("vehicle do not support %s yet\n", new_camera->dev.device_info.dev.init_name);
		return false;
	}
	/*sensor name match, check sensor id*/
	if (sensor_cb->sensor_check_id_cb) {
		ad->ad_name = sensor_cb->name;
		/* rotation */
		ad->cfg.rotate_mirror = 0;
		i = new_camera->orientation / 90;
		if (i-- > 0)
			ad->cfg.rotate_mirror = (1 << i) & 0xf;
		/* mirror */
		if (new_camera->mirror & 0x1)
			ad->cfg.rotate_mirror |= RGA_TRANSFORM_FLIP_H;
		else if (new_camera->mirror & 0x10)
			ad->cfg.rotate_mirror |= RGA_TRANSFORM_FLIP_V;
		ad->i2c_chl = new_camera->dev.link_info.i2c_adapter_id;
		ad->i2c_add = new_camera->dev.i2c_cam_info.addr;
		ad->i2c_rate = new_camera->i2c_rate;
		ad->ad_chl = 0;/*default ad select channel*/
		ad->mclk_rate = new_camera->mclk_rate;

		ad->pwdn_active = ((new_camera->io.gpio_flag & RK29_CAM_POWERDNACTIVE_MASK) >> RK29_CAM_POWERDNACTIVE_BITPOS);
		ad->powerdown = new_camera->io.gpio_powerdown;
		ad->power = -1;

		ret = sensor_cb->sensor_check_id_cb(ad);
		if (IS_ERR_VALUE(ret)) {
			DBG("%s check id failed!\n", ad->ad_name);
			return false;
		}
	}
	return true;
}
int vehicle_parse_sensor(struct ad_dev *ad)
{
	int ret = -1;
	struct vehicle_platform_data *pdata = ad->dev->platform_data;
	struct rkcamera_platform_data *new_camera = pdata->camera_data;

	/*try match platform sensors, if one checked just exit*/
	if (new_camera != NULL) {
		while (!strstr(new_camera->dev.device_info.dev.init_name, "end")) {
			if (vehicle_sensor_check(ad, new_camera)) {
				ret = 0;
				break;
			}
			new_camera++;
		}
	}
	return ret;
}

#endif
int vehicle_ad_init(struct ad_dev *ad)
{
	int ret = 0;

	if (sensor_cb->sensor_init) {
		ret = sensor_cb->sensor_init(ad);
		if (IS_ERR_VALUE(ret)) {
			DBG("%s sensor_init failed!\n", ad->ad_name);
			goto end;
		}
	} else {
		DBG("%s sensor_init is NULL!\n", ad->ad_name);
		ret = -1;
		goto end;
	}
end:
	return ret;
}

int vehicle_ad_deinit(void)
{
	if (sensor_cb->sensor_deinit)
		return sensor_cb->sensor_deinit();
	else
		return -1;
}

struct vehicle_cfg *ad_get_vehicle_cfg(void)
{
	struct vehicle_cfg *cfg;

	if (sensor_cb->sensor_get_cfg)
		sensor_cb->sensor_get_cfg(&cfg);

	return cfg;
}

void ad_check_cif_error(struct ad_dev *ad, int last_line)
{
	if (sensor_cb->sensor_get_cfg)
		sensor_cb->sensor_check_cif_error(ad, last_line);
}

