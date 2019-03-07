#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <media/v4l2-chip-ident.h>
#include <linux/videodev2.h>
#include "vehicle_cfg.h"
#include "vehicle_main.h"
#include "vehicle_ad.h"
#include "vehicle_ad_dm5150.h"

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif


enum {
	FORCE_PAL_WIDTH = 720,
	FORCE_PAL_HEIGHT = 576,
	FORCE_NTSC_WIDTH = 720,
	FORCE_NTSC_HEIGHT = 480,
	FORCE_CIF_OUTPUT_FORMAT = CIF_OUTPUT_FORMAT_420,
};

static v4l2_std_id std_old = V4L2_STD_NTSC;

#define SENSOR_REGISTER_LEN	1	/* sensor register address bytes*/
#define SENSOR_VALUE_LEN	1	/* sensor register value bytes*/

#define DM5150_SENSOR_BUS_PARAM	(SOCAM_MASTER |	\
					SOCAM_PCLK_SAMPLE_RISING |	\
					SOCAM_HSYNC_ACTIVE_HIGH |	\
					SOCAM_VSYNC_ACTIVE_HIGH |	\
					SOCAM_DATA_ACTIVE_HIGH |	\
					SOCAM_DATAWIDTH_8 |	\
					SOCAM_MCLK_24MHZ)

struct rk_sensor_reg {
	unsigned int reg;
	unsigned int val;
};

#define DM5150_REG_VD_STATUS		0x10
#define DM5150_REG_VD_STATUS_MASK	0xfc
#define DM5150_REG_VD_STATUS_PAL_NC	0X80
#define DM5150_REG_VD_STATUS_PAL_B_D_N 0X40
#define DM5150_REG_VD_STATUS_PAL_M 0X20
#define DM5150_REG_VD_STATUS_PAL_60 0X10
#define DM5150_REG_VD_STATUS_NTSC_4_43 0X08
#define DM5150_REG_VD_STATUS_NTSC_M_J 0X04

#define DM5150_REG_REVNUM 0x5F

#define SEQCMD_WAIT_MS	 0xFD000000
#define SEQCMD_END  0xFF000000

#define SensorWaitMs(a) 	   {SEQCMD_WAIT_MS, a}
#define SensorEnd   {SEQCMD_END, 0x00}

#define SENSOR_DG DBG
#define SENSOR_TR DBG
static struct ad_dev *pAd;

/* Preview resolution setting*/
static struct rk_sensor_reg sensor_preview_data_cvbs[] = {
	{0x51, 0x01},//bit[0]= '1' system reset
	{0x32, 0x00},//bit[0]= '0' set the IC for 720(720*480 or 576) mode
	{0x3b, 0x00},//disable auto detection line lock camera function
	{0x3c, 0x05},//set line lock auto detection threshold
	{0x08, 0x30},//set color burst difference threshold
	{0x14, 0x16},//always tracking clkoffset
	{0x17, 0x0a},//set Hsync track
	{0x05, 0x20},//set Hsync threshold
	{0x18, 0x23},//bit[0]= '1' auto update HSYNCTH
	{0x39, 0xd2},//increase color AGC threshold
	{0x3d, 0x40},//CCIR buffer reset
	{0x3a, 0x90},//CAGC enable
	{0x1a, 0x15},//burst detect option.
	{0x06, 0x0a},//drive black panel when no video signal
	{0x00, 0x8a},//bit[7] = '1' Reset, bit[3]= '1' Select VADC_A to VD. bit[1]= '1' Enable
	SensorEnd
};

static v4l2_std_id dm5150_std_to_v4l2(u8 status1)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!status1)
		return V4L2_STD_UNKNOWN;

	switch (status1 & DM5150_REG_VD_STATUS_MASK) {
	case DM5150_REG_VD_STATUS_NTSC_M_J:
		return V4L2_STD_NTSC;
	case DM5150_REG_VD_STATUS_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case DM5150_REG_VD_STATUS_PAL_B_D_N:
	case DM5150_REG_VD_STATUS_PAL_NC:
		return V4L2_STD_PAL;
	case DM5150_REG_VD_STATUS_PAL_M:
		return V4L2_STD_PAL_M;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 dm5150_status_to_v4l2(u8 status)
{
	if (!status)
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

int dm5150_ad_check_siganl(struct ad_dev *pAd)
{
    unsigned int pValue = -1,nRet = -1;

    pValue = vehicle_generic_sensor_read(pAd, DM5150_REG_VD_STATUS);
    if(IS_ERR_VALUE(pValue)){
        printk("\n status0 failed = 0x%x\n",pValue);
    } 
 
    if (!pValue) {
       printk("No video signal\n");  
       nRet = 0 ; 
    }else{
       printk("With video signal\n");
       nRet = 1 ; 
    }

    return nRet ;
}

static int dm5150_vehicle_status(struct ad_dev *ad,
				  u32 *status,
				  v4l2_std_id *std)
{
	char status1 = 0;
	static int laststate = -1;
	int state;

	status1 = vehicle_generic_sensor_read(ad, DM5150_REG_VD_STATUS);
	if (status1 < 0)
		return status1;

	if (status)
		*status = dm5150_status_to_v4l2(status1);

	if (std)
		*std = dm5150_std_to_v4l2(status1);

	state = dm5150_status_to_v4l2(status1);
	if (state != laststate){
		vehicle_ad_stat_change_notify();
		laststate = state;
	}
    
	return 0;
}

static void dm5150_reinit_parameter(struct ad_dev *ad, v4l2_std_id std)
{
	int i;

	ad->cfg.bus_param = DM5150_SENSOR_BUS_PARAM;
    printk("===CVSTD_720P60= std%d\n",(int)std);

	switch (std) {
	case V4L2_STD_PAL:
		ad->cfg.width = FORCE_PAL_WIDTH;
		ad->cfg.height = FORCE_PAL_HEIGHT;
		ad->cfg.start_x = 4;
		ad->cfg.start_y = 8;
		ad->cfg.input_format = CIF_INPUT_FORMAT_PAL;
		ad->cfg.output_format = FORCE_CIF_OUTPUT_FORMAT;
		ad->cfg.field_order = 0;
		ad->cfg.yuv_order = 0;
		ad->cfg.frame_rate = 25;
		ad->cfg.skip_frames = 4;
		break;
	case V4L2_STD_NTSC:
		ad->cfg.width = 720;
		ad->cfg.height = 462;
		ad->cfg.start_x = 4;
		ad->cfg.start_y = 8;
		ad->cfg.input_format = CIF_INPUT_FORMAT_NTSC;
		ad->cfg.output_format = FORCE_CIF_OUTPUT_FORMAT;
		ad->cfg.field_order = 0;
		ad->cfg.yuv_order = 0;
		ad->cfg.frame_rate = 30;
		ad->cfg.skip_frames = 4;
		break;
	default:
		ad->cfg.width = 720;
		ad->cfg.height = 480;
		ad->cfg.start_x = 8;
		ad->cfg.start_y = 20;
		ad->cfg.input_format = CIF_INPUT_FORMAT_NTSC;//CIF_INPUT_FORMAT_YUV;
		ad->cfg.output_format = FORCE_CIF_OUTPUT_FORMAT;
		ad->cfg.field_order = 0;
		ad->cfg.yuv_order = 0;/*00 - UYVY*/
		/*href:0,high;1,low*/
        if (ad->cfg.bus_param | SOCAM_HSYNC_ACTIVE_HIGH)
            ad->cfg.href = 0;
        else
            ad->cfg.href = 1;
		/*vsync:0,low;1,high*/
		if (ad->cfg.bus_param | SOCAM_VSYNC_ACTIVE_HIGH)
			ad->cfg.vsync = 1;
		else
			ad->cfg.vsync = 0;
		ad->cfg.frame_rate = 25;
		ad->cfg.skip_frames = 4;
		break;		
	}

	/*href:0,high;1,low*/
	if (ad->cfg.bus_param | SOCAM_HSYNC_ACTIVE_HIGH)
		ad->cfg.href = 0;
	else
		ad->cfg.href = 1;
	/*vsync:0,low;1,high*/
	if (ad->cfg.bus_param | SOCAM_VSYNC_ACTIVE_HIGH)
		ad->cfg.vsync = 1;
	else
		ad->cfg.vsync = 0;

	/* fix crop info from dts config */
	for (i = 0; i < 4; i++) {
		if ((ad->defrects[i].width == ad->cfg.width) &&
		    (ad->defrects[i].height == ad->cfg.height)) {
			ad->cfg.start_x = ad->defrects[i].crop_x;
			ad->cfg.start_y = ad->defrects[i].crop_y;
			ad->cfg.width  =  ad->defrects[i].crop_width;
			ad->cfg.height  = ad->defrects[i].crop_height;
		}
	}

	DBG("dm5150 size %dx%d, crop(%d,%d)\n",
	    ad->cfg.width, ad->cfg.height,
	    ad->cfg.start_x, ad->cfg.start_y);
}

static void dm5150_reg_init(struct ad_dev *ad, unsigned char cvstd)
{
	struct rk_sensor_reg *sensor;
	int i = 0;
	unsigned char val[2];

	sensor = sensor_preview_data_cvbs;
    pAd = ad;
	while ((sensor[i].reg != SEQCMD_END) && (sensor[i].reg != 0xFC000000)) {
		val[0] = sensor[i].val;
		vehicle_generic_sensor_write(ad, sensor[i].reg, val);
		//DBG("%s write:=0x%x ,read:=0x%x\n", ad->ad_name,sensor[i].val, vehicle_generic_sensor_read(ad, sensor[i].reg));
		i++;
	}
}

int dm5150_ad_get_cfg(struct vehicle_cfg **cfg)
{
	u32 status = 0;
  
	if (!g_addev)
		return -1;

	dm5150_vehicle_status(g_addev, &status, NULL);
	if (status) /* No signal */
		g_addev->cfg.ad_ready = false;
	else
		g_addev->cfg.ad_ready = true;

	*cfg = &g_addev->cfg;

	return 0;
}

void dm5150_ad_check_cif_error(struct ad_dev *ad, int last_line)
{
	DBG("%s, last_line %d\n\n\n", __func__, last_line);
	if (last_line < 1)
		return;

	ad->cif_error_last_line = last_line;
	if (V4L2_STD_PAL == std_old) {
		if (last_line == FORCE_NTSC_HEIGHT) {
			if (ad->state_check_work.state_check_wq)
				queue_delayed_work(
						   ad->state_check_work.
						   state_check_wq,
						   &ad->state_check_work.work,
						   msecs_to_jiffies(0));
		}
	} else if (V4L2_STD_NTSC == std_old) {
		if (last_line == FORCE_PAL_HEIGHT) {
			if (ad->state_check_work.state_check_wq)
				queue_delayed_work(
						   ad->state_check_work.
						   state_check_wq,
						   &ad->state_check_work.work,
						   msecs_to_jiffies(0));
		}
	}
}
static void power_on(struct ad_dev *ad);
static void power_off(struct ad_dev *ad);
int dm5150_check_id(struct ad_dev *ad)
{
	int i = 0;
	int check_id_ret = -1;
	int val;

	DBG("%s\n", __func__);

	/*  1. i2c init */
	ad->adapter = i2c_get_adapter(ad->i2c_chl);
	if (ad->adapter == NULL)
		return -1;

	if (!i2c_check_functionality(ad->adapter, I2C_FUNC_I2C)) {
		i2c_put_adapter(ad->adapter);
		ad->adapter = NULL;
		DBG("====%s i2c_check_functionality failed\n", __func__);
		return -1;
	}

	/*  2. ad power on*/
	power_on(ad);
	msleep(20);

	while (i++ < 5) {
		msleep(5);
		val = vehicle_generic_sensor_read(ad, DM5150_REG_REVNUM);
		DBG("%s read 0x%02x --> 0x%02x\n", ad->ad_name, DM5150_REG_REVNUM, val);
		if (0x3b == val){
			pr_info("dm5150 check id succeed\n");
			check_id_ret = 0;
			break;
		}
	}

	/*if sensor match, set raw params here*/
	if (check_id_ret == 0) {
		ad->fix_format = 0;
		ad->defrects[0].width = 720;
		ad->defrects[0].height = 462;//byfreehua 480;
		ad->defrects[0].crop_x = 4;
		ad->defrects[0].crop_y = 8;
		ad->defrects[0].crop_width = 712; 
		ad->defrects[0].crop_height = 462;
		ad->defrects[0].interface = "cvbs_ntsc";

		ad->defrects[1].width = 720;
		ad->defrects[1].height = 576;
		ad->defrects[1].crop_x = 8;
		ad->defrects[1].crop_y = 0;
		ad->defrects[1].crop_width = 712;
		ad->defrects[1].crop_height = 576;
		ad->defrects[1].interface = "cvbs_pal";

		DBG("%s: ad_chl=%d,,ad_addr=%x,fix_for=%d\n", ad->ad_name,
		    ad->ad_chl, ad->i2c_add, ad->fix_format);
		DBG("gpio power:%d, active:%d\n", ad->power, ad->pwr_active);
		DBG("gpio powerdown:%d, active:%d\n",
		    ad->powerdown, ad->pwdn_active);
	}

	/* if check failed, release resources*/
	if (check_id_ret == -1) {
		power_off(ad);
		i2c_put_adapter(ad->adapter);
		ad->adapter = NULL;
	}
	return check_id_ret;
}

static int dm5150_check_std(struct ad_dev *ad, v4l2_std_id *std)
{
	u32 status;
	static bool is_first = true;

	dm5150_vehicle_status(ad, &status, std);

	if (status && is_first) { /* No signal */
		mdelay(30);
		dm5150_vehicle_status(ad, &status, std);
		DBG("status 0x%x\n", status);
	}

	if (status)
		*std = V4L2_STD_NTSC;
    
	return 0;
}

static void power_on(struct ad_dev *ad)
{
	/* gpio_direction_output(ad->power, ad->pwr_active); */

	if (gpio_is_valid(ad->powerdown)) {
		gpio_request(ad->powerdown, "ad_powerdown");
		gpio_direction_output(ad->powerdown, !ad->pwdn_active);
		/* gpio_set_value(ad->powerdown, !ad->pwdn_active); */
	}

	if (gpio_is_valid(ad->power)) {
		gpio_request(ad->power, "ad_power");
		gpio_direction_output(ad->power, ad->pwr_active);
		/* gpio_set_value(ad->power, ad->pwr_active); */
	}
}

static void power_off(struct ad_dev *ad)
{
	if (gpio_is_valid(ad->powerdown))
		gpio_free(ad->powerdown);

	if (gpio_is_valid(ad->power))
		gpio_free(ad->power);
}

void dm5150_check_state_work(struct work_struct *work)
{
	struct ad_dev *ad;
	v4l2_std_id std;

	ad = g_addev;

	if (ad->cif_error_last_line > 0)
		ad->cif_error_last_line = 0;

	dm5150_check_std(ad, &std);

	if (std != std_old) {
		std_old = std;
		dm5150_reinit_parameter(ad, std);
		vehicle_ad_stat_change_notify();
	}    

	queue_delayed_work(ad->state_check_work.state_check_wq,
			   &ad->state_check_work.work, msecs_to_jiffies(1000));
}

int dm5150_ad_deinit(void)
{
	struct ad_dev *ad;

	ad = g_addev;

	if (!ad)
		return -1;

	if (ad->state_check_work.state_check_wq) {
		cancel_delayed_work_sync(&ad->state_check_work.work);
		flush_delayed_work(&ad->state_check_work.work);
		flush_workqueue(ad->state_check_work.state_check_wq);
		destroy_workqueue(ad->state_check_work.state_check_wq);
	}
	if (ad->irq)
		free_irq(ad->irq, ad);
	power_off(ad);
	return 0;
}

static int tvp5150_write_reg(struct i2c_adapter *adap,
				u8 reg, u8 val)
{
	struct i2c_msg msgs[1];
	u8 buf[2];
	int ret;
	
	buf[0] = reg;
	buf[1] = val;

	msgs[0].addr = 0x5d;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	msgs[0].scl_rate = 100*1024;
	
	ret = i2c_transfer(adap, msgs, 1);
	 
	return (ret == 1)? 0 : ret;
}

int dm5150_ad_init(struct ad_dev *ad)
{
	int ret;
	v4l2_std_id std;

	if (!ad)
		return -1;

	g_addev = ad;

	/*  1. i2c init */
	if (ad->adapter == NULL) {
		ad->adapter = i2c_get_adapter(ad->i2c_chl);
        DBG("===dm5150_ad_init i2c channel=%d\n",ad->i2c_chl);
		if (ad->adapter == NULL)
			return -1;

		if (!i2c_check_functionality(ad->adapter, I2C_FUNC_I2C)){
			return -1;
		}
	}

	/* for debug with DM5150 demo board */
	tvp5150_write_reg(ad->adapter, 0x3, 0xd);

	/*  2. ad power on, already power on when checkid*/
	/* fix mode */
	msleep(500);
	dm5150_check_std(ad, &std);
	std_old = std;
	DBG("std: %s\n", (std == V4L2_STD_NTSC) ? "ntsc" : "pal");

	/*  3 .init default format params */
	dm5150_reg_init(ad, std);
	dm5150_reinit_parameter(ad, std);
	vehicle_ad_stat_change_notify();

	/*  4. ad register signal detect irq */
	if(0){
		ad->irq = gpio_to_irq(ad->cvstd);
		ret = request_irq(ad->irq, NULL, IRQF_TRIGGER_FALLING,
				  "vehicle ad_dm5150", ad);
	}

	/*  5. create workqueue to detect signal change */
	INIT_DELAYED_WORK(&ad->state_check_work.work, dm5150_check_state_work);
	ad->state_check_work.state_check_wq =
		create_singlethread_workqueue("vehicle-ad-dm5150");

	queue_delayed_work(ad->state_check_work.state_check_wq,
			   &ad->state_check_work.work, msecs_to_jiffies(1000));

	return 0;
}


