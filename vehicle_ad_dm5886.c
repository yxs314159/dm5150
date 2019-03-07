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
#include "vehicle_ad_dm5886.h"

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

#define DM5886_SENSOR_BUS_PARAM	(SOCAM_MASTER |	\
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

#define DM5886_REG_VD_STATUS		0x10
#define DM5886_REG_VD_STATUS_MASK	0xfc
#define DM5886_REG_VD_STATUS_PAL_NC	0X80
#define DM5886_REG_VD_STATUS_PAL_B_D_N 0X40
#define DM5886_REG_VD_STATUS_PAL_M 0X20
#define DM5886_REG_VD_STATUS_PAL_60 0X10
#define DM5886_REG_VD_STATUS_NTSC_4_43 0X08
#define DM5886_REG_VD_STATUS_NTSC_M_J 0X04

#define DM5886_REG_REVNUM 0xFF

#define SEQCMD_WAIT_MS	 0xFD000000
#define SEQCMD_END  0xFF000000

#define SensorWaitMs(a) 	   {SEQCMD_WAIT_MS, a}
#define SensorEnd   {SEQCMD_END, 0x00}

#define SENSOR_DG DBG
#define SENSOR_TR DBG
static struct ad_dev *pAd;

/* Preview resolution setting*/
static struct rk_sensor_reg sensor_preview_data_cvbs[] = {
	{0x92, 0x0e},// set the PLL parameter M for 108MHZ
	{0x93, 0x00},// set the PLL parameter N for 108MHZ
	{0x96, 0x01},// set the PLL parameter OD for 108MHZ.
	{0x90, 0x81},//bit[7]= '1' software PLL reset, wait for 1 ms before issue another command
//bit[0]= '1' software PLL_1
	SensorWaitMs(2),
	{0x65, 0x03},//bit[1:0]='b11' system reset
	{0x64, 0x0f},//bit[3:0]='b1111' select 4 VD(Video decoder) to set
	{0x00, 0x01},//bit[0]= '1' software reset VD
	{0x32, 0x00},//bit[0]=’0’ set the IC for 720(720*480 or 576) mode (Only valid in DM5886)
	{0x4b, 0xdc},//set line Blank sample position
	{0x06, 0x02},//bit[1]= '1' VD will out ccir656
	{0x04, 0x03},//set AGCDOWNTH
	{0x14, 0x16},//always tracking clkoffset
	{0x17, 0x0a},//set Hsync track
	{0x05, 0x20},//set Hsync threshold
	{0x18, 0x23},//bit[0]= '1' auto update HSYNCTH
	{0x39, 0xd2},//increase color AGC threshold
	{0x3d, 0x40},//set CCIR buffer reset
	{0x3a, 0x90},//CAGC enable
	{0x00, 0x8a},// bit[7] = '1' Reset VD (video decoder) and the CSR (Control signal register)
//does not change.
//bit[3]= '1' Select VADC (video analog to digital converter) to VD.
//bit[1]= '1' Enable video decoding function.
	{0x64, 0x0a},//bit[3:0]='b1010' select VD_1 & VD_3 to set
	{0x00, 0x82},// bit[7] = '1' Reset VD (video decoder) and the CSR (Control signal register)
//does not change.
//bit[3]= '0' Select VADC (video analog to digital converter) to VD.
//bit[1]= '1' Enable video decoding function.
	{0x80, 0x14},//set software fix gain
	{0x82, 0x00},//set software fix gain value
	{0x83, 0x00},//set software fix gain value
	{0x86, 0x04},//set software fix gain
	{0x88, 0x00},//set software fix gain value
	{0x89, 0x00},//set software fix gain value
	{0x75, 0x10},//IO/Clock Configuration
	{0x6a, 0x0C},//IC Mode Control
	{0x71, 0x54},//CCIROUT_0 Otdm Configuration 1
	{0xd0, 0x37},//CCIR Input Line Length
	{0xd1, 0xe0},//SDRAM Config 1
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xd0},//CCIR Input Line Length
	{0xa6, 0x0f},//Mixer Output partition Config
	{0xa7, 0x10},//Mixer Output partition Coordinate
	{0xa8, 0x00},
	{0xa9, 0x00},
	{0xaa, 0x68},
	{0xab, 0x00},
	{0xac, 0x54},
	{0xad, 0x00},
	{0xae, 0x20},
	{0xaf, 0x68},
	{0xb0, 0x20},
	{0xb1, 0x22},//Mixer Output partition Coordinate
	{0xb2, 0xd0},//Mixer Out Region End Point Config 2
	{0xb3, 0x40},//Mixer Out Region End Point Config 3
	{0xa3, 0x02},//mixer out blue panel when no valid signal
	{0xd3, 0x10},//Blue Panel Y Config, RGB(0, 0, 0)
	{0xd4, 0x80},//Blue Panel CB Config,RGB(0, 0, 0)
	{0xd5, 0x80},//Blue Panel CR Config,RGB(0, 0, 0)
	{0xb9, 0x13},//bit[2:0] select VD_0 signal to Mixer channel 0
//bit[6:4] select VD_1 signal to Mixer channel 1
	{0xba, 0x32},//bit[2:0] select VD_2 signal to Mixer channel 2
//bit[6:4] select VD_3 signal to Mixer channel 3
	{0xbd, 0x00},//disable mirror function (only valid for SD mode)
	{0xb8, 0x01},//channel enable 0
	{0xa0, 0xc4},//bit[3:0]='b0000' out_sel (SD mode), 4-CVBS bypass
	{0xa2, 0x90},//Mixer Mode detect, 0x90 PAL, 0x10 NTSC
	{0xa1, 0x01},//Mixer Out Enable
	{0x68, 0x01},//CCIR656 IO Control
	{0x6d, 0x10},//VD Power On Rstz
	{0x8f, 0x07},//VADC Digcore Config
	SensorEnd
};

static v4l2_std_id dm5886_std_to_v4l2(u8 status1)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!status1)
		return V4L2_STD_UNKNOWN;

	switch (status1 & DM5886_REG_VD_STATUS_MASK) {
	case DM5886_REG_VD_STATUS_NTSC_M_J:
		return V4L2_STD_NTSC;
	case DM5886_REG_VD_STATUS_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case DM5886_REG_VD_STATUS_PAL_B_D_N:
	case DM5886_REG_VD_STATUS_PAL_NC:
		return V4L2_STD_PAL;
	case DM5886_REG_VD_STATUS_PAL_M:
		return V4L2_STD_PAL_M;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 dm5886_status_to_v4l2(u8 status)
{
	if (!status)
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

int dm5886_ad_check_siganl(struct ad_dev *pAd)
{
    unsigned int pValue = -1,nRet = -1;

    pValue = vehicle_generic_sensor_read(pAd, DM5886_REG_VD_STATUS);
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

static int dm5886_vehicle_status(struct ad_dev *ad,
				  u32 *status,
				  v4l2_std_id *std)
{
	char status1 = 0;
	static int laststate = -1;
	int state;

	status1 = vehicle_generic_sensor_read(ad, DM5886_REG_VD_STATUS);
	if (status1 < 0)
		return status1;

	if (status)
		*status = dm5886_status_to_v4l2(status1);

	if (std)
		*std = dm5886_std_to_v4l2(status1);

	state = dm5886_status_to_v4l2(status1);
	if (state != laststate){
		vehicle_ad_stat_change_notify();
		laststate = state;
	}
    
	return 0;
}

static void dm5886_reinit_parameter(struct ad_dev *ad, v4l2_std_id std)
{
	int i;

	ad->cfg.bus_param = DM5886_SENSOR_BUS_PARAM;
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

	DBG("dm5886 size %dx%d, crop(%d,%d)\n",
	    ad->cfg.width, ad->cfg.height,
	    ad->cfg.start_x, ad->cfg.start_y);
}

static void dm5886_reg_init(struct ad_dev *ad, unsigned char cvstd)
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

int dm5886_ad_get_cfg(struct vehicle_cfg **cfg)
{
	u32 status = 0;
  
	if (!g_addev)
		return -1;

	dm5886_vehicle_status(g_addev, &status, NULL);
	if (status) /* No signal */
		g_addev->cfg.ad_ready = false;
	else
		g_addev->cfg.ad_ready = true;

	*cfg = &g_addev->cfg;

	return 0;
}

void dm5886_ad_check_cif_error(struct ad_dev *ad, int last_line)
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
int dm5886_check_id(struct ad_dev *ad)
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
		val = vehicle_generic_sensor_read(ad, DM5886_REG_REVNUM);
		DBG("%s read 0x%02x --> 0x%02x\n", ad->ad_name, DM5886_REG_REVNUM, val);
		if (0x65 == val){
			pr_info("dm5886 check id succeed\n");
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

static int dm5886_check_std(struct ad_dev *ad, v4l2_std_id *std)
{
	u32 status;
	static bool is_first = true;

	dm5886_vehicle_status(ad, &status, std);

	if (status && is_first) { /* No signal */
		mdelay(30);
		dm5886_vehicle_status(ad, &status, std);
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

void dm5886_check_state_work(struct work_struct *work)
{
	struct ad_dev *ad;
	v4l2_std_id std;

	ad = g_addev;

	if (ad->cif_error_last_line > 0)
		ad->cif_error_last_line = 0;

	dm5886_check_std(ad, &std);

	if (std != std_old) {
		std_old = std;
		dm5886_reinit_parameter(ad, std);
		vehicle_ad_stat_change_notify();
	}    

	queue_delayed_work(ad->state_check_work.state_check_wq,
			   &ad->state_check_work.work, msecs_to_jiffies(1000));
}

int dm5886_ad_deinit(void)
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

int dm5886_ad_init(struct ad_dev *ad)
{
	int ret;
	v4l2_std_id std;

	if (!ad)
		return -1;

	g_addev = ad;

	/*  1. i2c init */
	if (ad->adapter == NULL) {
		ad->adapter = i2c_get_adapter(ad->i2c_chl);
        DBG("===dm5886_ad_init i2c channel=%d\n",ad->i2c_chl);
		if (ad->adapter == NULL)
			return -1;

		if (!i2c_check_functionality(ad->adapter, I2C_FUNC_I2C)){
			return -1;
		}
	}

	/* for debug with DM5886 demo board */
	tvp5150_write_reg(ad->adapter, 0x3, 0xd);

	/*  2. ad power on, already power on when checkid*/
	/* fix mode */
	msleep(500);
	dm5886_check_std(ad, &std);
	std_old = std;
	DBG("std: %s\n", (std == V4L2_STD_NTSC) ? "ntsc" : "pal");

	/*  3 .init default format params */
	dm5886_reg_init(ad, std);
	dm5886_reinit_parameter(ad, std);
	vehicle_ad_stat_change_notify();

	/*  4. ad register signal detect irq */
	if(0){
		ad->irq = gpio_to_irq(ad->cvstd);
		ret = request_irq(ad->irq, NULL, IRQF_TRIGGER_FALLING,
				  "vehicle ad_dm5886", ad);
	}

	/*  5. create workqueue to detect signal change */
	INIT_DELAYED_WORK(&ad->state_check_work.work, dm5886_check_state_work);
	ad->state_check_work.state_check_wq =
		create_singlethread_workqueue("vehicle-ad-dm5886");

	queue_delayed_work(ad->state_check_work.state_check_wq,
			   &ad->state_check_work.work, msecs_to_jiffies(1000));

	return 0;
}


