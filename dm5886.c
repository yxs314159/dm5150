#include "generic_sensor.h"
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "dm5886.h"

static int version = KERNEL_VERSION(8,8,8);
module_param(version, int, S_IRUGO);

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

static int debug = 7;
module_param(debug, int, S_IRUGO|S_IWUSR);

#define dprintk(level, fmt, arg...) do {		\
	if (debug >= level)				\
		printk(KERN_WARNING fmt, ## arg);	\
} while (0)
#define debug_printk(format, ...) dprintk(1, format, ## __VA_ARGS__)
/* Sensor Driver Configuration Begin */
#define SENSOR_NAME RK29_CAM_SENSOR_DM5886
#define SENSOR_V4L2_IDENT V4L2_IDENT_DM5886
#define SENSOR_ID 0x65
#define SENSOR_BUS_PARAM  (SOCAM_MASTER |\
                           SOCAM_PCLK_SAMPLE_RISING|\
                           SOCAM_HSYNC_ACTIVE_HIGH|\
					       SOCAM_VSYNC_ACTIVE_HIGH |	\
                           SOCAM_DATA_ACTIVE_HIGH |\
                           SOCAM_DATAWIDTH_8  |\
                           SOCAM_MCLK_24MHZ)

static int SENSOR_PREVIEW_W = 720;
static int SENSOR_PREVIEW_H = 576;
static volatile v4l2_std_id std_last_g = V4L2_STD_PAL;

static struct rk_camera_device_signal_config dev_info[] = {
	{
		.type = RK_CAMERA_DEVICE_CVBS_PAL,
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.crop = {
			.top = 0,   
			.left = 0, 
			.width = 720,
			.height = 576, 
		}
	}
};

static struct rk_camera_device_defrect defrects[] = {
	{
		.width = 720,
		.height = 480,
		.defrect = {
			.top = 2,	
			.left = 0,	
			.width = 720,
			.height = 480,
		},
		.interface = "cvbs_ntsc"
	},
	{
		.width = 720,
		.height = 576,
		.defrect = {
			.top = 0,
			.left = 0,
			.width = 720,
			.height = 576
		},
		.interface = "cvbs_pal"
	},
};

static struct rk_camera_device_channel_info channel_infos = {
	.channel_total = 1,
	.default_id = 0,
	.channel_info = {
		"cvbs 1",
	}
};

static char input_mode[10] = "PAL";
#define SENSOR_PREVIEW_FPS                   30000     // 30fps
#define SENSOR_FULLRES_L_FPS                 7500      // 7.5fps
#define SENSOR_FULLRES_H_FPS                 7500      // 7.5fps
#define SENSOR_720P_FPS                      0
#define SENSOR_1080P_FPS                     0

#define SENSOR_REGISTER_LEN                  1         // sensor register address bytes
#define SENSOR_VALUE_LEN                     1         // sensor register value bytes

static unsigned int SensorConfiguration = (CFG_Effect | CFG_Scene);
static unsigned int SensorChipID[] = {SENSOR_ID};
/* Sensor Driver Configuration End */


#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

#define SensorRegVal(a,b) CONS4(SensorReg,SENSOR_REGISTER_LEN,Val,SENSOR_VALUE_LEN)(a,b)
#define sensor_write(client,reg,v) CONS4(sensor_write_reg,SENSOR_REGISTER_LEN,val,SENSOR_VALUE_LEN)(client,(reg),(v))
#define sensor_read(client,reg,v) CONS4(sensor_read_reg,SENSOR_REGISTER_LEN,val,SENSOR_VALUE_LEN)(client,(reg),(v))
#define sensor_write_array generic_sensor_write_array
static void dm5886_reinit_parameter(v4l2_std_id std,
				     struct generic_sensor *sensor);
static void dm5886_send_uevent(struct generic_sensor *sensor);

struct sensor_parameter {
	unsigned int PreviewDummyPixels;
	unsigned int CaptureDummyPixels;
	unsigned int preview_exposure;
	unsigned short int preview_line_width;
	unsigned short int preview_gain;

	unsigned short int PreviewPclk;
	unsigned short int CapturePclk;
	char awb[6];
};

struct specific_sensor{
	struct generic_sensor common_sensor;
	//define user data below
	struct sensor_parameter parameter;
};

static struct rk_sensor_reg sensor_check_id_data[] = {
	SensorRegVal(0xff, 0x0),
	SensorEnd
};


/* Sensor initial setting */
static struct rk_sensor_reg sensor_init_data[] = {
	SensorEnd
};
/* Senor full resolution setting: recommand for capture */
static struct rk_sensor_reg sensor_fullres_lowfps_data[] = {
	SensorEnd
};
/* Senor full resolution setting: recommand for video */
static struct rk_sensor_reg sensor_fullres_highfps_data[] = {
	SensorEnd
};
/* Preview resolution setting*/


/* 1280x720 */
static struct rk_sensor_reg sensor_720p[] = {
	SensorEnd
};

/* 1920x1080 */
static struct rk_sensor_reg sensor_1080p[] = {
	SensorEnd
};

static struct rk_sensor_reg pal_coordinate[] = {
	{0xc3, 0x00},//channel 0 capture interface cropping start point
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xd0},//CCIR Input Line Length
	{0xa6, 0x0f},//Mixer Output partition Config
	{0xa7, 0x10},//Mixer Output partition Coordinate y1 x1 y0 x0
	{0xa8, 0x00},//Mixer Output partition Coordinate x0
	{0xa9, 0x00},//Mixer Output partition Coordinate y0
	{0xaa, 0x68},//Mixer Output partition Coordinate x1
	{0xab, 0x00},//Mixer Output partition Coordinate y1
	{0xac, 0x54},//Mixer Output partition Coordinate y3 x3 y2 x2
	{0xad, 0x00},//Mixer Output partition Coordinate x2
	{0xae, 0x20},//Mixer Output partition Coordinate y2
	{0xaf, 0x68},//Mixer Output partition Coordinate x3
	{0xb0, 0x20},//Mixer Output partition Coordinate y3
	{0xb1, 0x22},//Mixer Output partition Coordinate ybr xbr
	{0xb2, 0xd0},//Mixer Output partition Coordinate xbr
	{0xb3, 0x40},//Mixer Output partition Coordinate ybr
	SensorEnd
};

static struct rk_sensor_reg pal_quad_coordinate[] = {
	{0xc3, 0x00},//channel 0 capture interface cropping start point
	{0xc4, 0x10},//channel 1 capture interface cropping start point
	{0xc5, 0x00},//channel 2 capture interface cropping start point
	{0xc6, 0x10},//channel 3 capture interface cropping start point
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xc0},//CCIR Input Line Length
	{0xa6, 0x0f},
	{0xa7, 0x10},
	{0xa8, 0x08},
	{0xa9, 0x00},
	{0xaa, 0x68},
	{0xab, 0x00},
	{0xac, 0x54},
	{0xad, 0x08},
	{0xae, 0x20},
	{0xaf, 0x68},
	{0xb0, 0x20},
	{0xb1, 0x22},
	{0xb2, 0xd0},
	{0xb3, 0x40},
	SensorEnd
};

static struct rk_sensor_reg pal_biview_ud_coordinate[] = {
	{0xc3, 0x00},//channel 0 capture interface cropping start point
	{0xc4, 0x00},//channel 1 capture interface cropping start point
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xd0},//CCIR Input Line Length
	{0xa6, 0x03},
	{0xa7, 0x40},
	{0xa8, 0x00},
	{0xa9, 0x00},
	{0xaa, 0x00},
	{0xab, 0x20},
	//{0xac, 0x54},
	//{0xad, 0x00},
	//{0xae, 0x20},
	//{0xaf, 0x68},
	//{0xb0, 0x20},
	{0xb1, 0x22},
	{0xb2, 0xd0},
	{0xb3, 0x40},
	SensorEnd
};

static struct rk_sensor_reg pal_biview_lr_coordinate[] = {
	{0xc3, 0x00},//channel 0 capture interface cropping start point
	{0xc4, 0x10},//channel 1 capture interface cropping start point
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xc0},//CCIR Input Line Length
	{0xa6, 0x03},
	{0xa7, 0x10},
	{0xa8, 0x08},
	{0xa9, 0x00},
	{0xaa, 0x68},
	{0xab, 0x00},
	//{0xac, 0x54},
	//{0xad, 0x00},
	//{0xae, 0x20},
	//{0xaf, 0x68},
	//{0xb0, 0x20},
	{0xb1, 0x22},
	{0xb2, 0xd0},
	{0xb3, 0x40},
	SensorEnd
};

static struct rk_sensor_reg pal_tview_coordinate[] = {
	{0xc3, 0x00},//channel 0 capture interface cropping start point
	{0xc4, 0x00},//channel 1 capture interface cropping start point
	{0xc5, 0x00},//channel 2 capture interface cropping start point
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xd0},//CCIR Input Line Length
	{0xa6, 0x07},
	{0xa7, 0x40},
	{0xa8, 0x00},
	{0xa9, 0x00},
	{0xaa, 0x00},
	{0xab, 0x20},
	{0xac, 0x55},
	{0xad, 0x68},
	{0xae, 0x20},
	{0xaf, 0x68},
	{0xb0, 0x20},
	{0xb1, 0x22},
	{0xb2, 0xd0},
	{0xb3, 0x40},
	SensorEnd
};

static struct rk_sensor_reg pal_hview_coordinate[] = {
	{0xc3, 0x00},//channel 0 capture interface cropping start point
	{0xc4, 0x00},//channel 1 capture interface cropping start point
	{0xc5, 0x00},//channel 2 capture interface cropping start point
	{0xc6, 0x00},//channel 3 capture interface cropping start point
	{0xa4, 0x02},//CCIR Input Line Length
	{0xa5, 0xd0},//CCIR Input Line Length
	{0xa6, 0x0f},
	{0xa7, 0x00},
	{0xa8, 0xb4},
	{0xa9, 0x00},
	{0xaa, 0x00},
	{0xab, 0x00},
	{0xac, 0x24},
	{0xad, 0xb4},
	{0xae, 0x20},
	{0xaf, 0x1c},
	{0xb0, 0x00},
	{0xb1, 0x22},
	{0xb2, 0xd0},
	{0xb3, 0x40},
	SensorEnd
};

static struct rk_sensor_reg sensor_softreset_data[] = {
	{0x75, 0x10},//IO/Clock Configuration
	{0x6a, 0x0C},//IC Mode Control
	{0x71, 0x54},//CCIROUT_0 Otdm Configuration 1
	{0xd0, 0x37},//CCIR Input Line Length
	{0xd1, 0xe0},//SDRAM Config 1
	{0xa3, 0x02},//Mixer Output format Configuration, BLUEOUT
	{0xd3, 0x10},//Blue Panel Y Config, RGB(0, 0, 0)
	{0xd4, 0x80},//Blue Panel CB Config,RGB(0, 0, 0)
	{0xd5, 0x80},//Blue Panel CR Config,RGB(0, 0, 0)
	{0xd6, 0x10},//Split Line Y Config, RGB(0, 0, 0)
	{0xd7, 0x80},//Split Line CB Config,RGB(0, 0, 0)
	{0xd8, 0x80},//Split Line CR Config,RGB(0, 0, 0)
	{0xbd, 0x00},//Mirror
	{0xa2, 0x90},//Mixer Mode detect, 0x90 PAL, 0x10 NTSC
	{0xa1, 0x01},//Mixer Out Enable
	{0x68, 0x01},//CCIR656 IO Control
	{0x6d, 0x10},//VD Power On Rstz
	{0x8f, 0x07},//VADC Digcore Config
	SensorEnd
};

static struct rk_sensor_reg sensor_preview_data[] = {
	{DM5886_REG_SYSTEM_CONTROL, 0x01},
	{DM5886_REG_VADC0_CONF1, 0x00},
	{DM5886_REG_VADC1_CONF1, 0x00},
	{DM5886_REG_CHA01_SELECT, 0x10},
	{DM5886_REG_CHA23_SELECT, 0x32},
	{DM5886_REG_MIXER_CHA_ENABLE, 0x0f},
	{DM5886_REG_MIXER_CONF, 0xc0},
	{DM5886_REG_VD_POWER_ON_RSTZ, 0x10},
	SensorWaitMs(200),
	SensorEnd
};

static struct rk_sensor_reg sensor_WhiteB_Auto[] = {
	SensorEnd
};
static	struct rk_sensor_reg sensor_WhiteB_Cloudy[] = {
	SensorEnd
};

/* ClearDay Colour Temperature : 5000K - 6500K	*/
static	struct rk_sensor_reg sensor_WhiteB_ClearDay[] = {
	SensorEnd
};
/* Office Colour Temperature : 3500K - 5000K  */
static	struct rk_sensor_reg sensor_WhiteB_TungstenLamp1[] = {
	SensorEnd
};
/* Home Colour Temperature : 2500K - 3500K	*/
static	struct rk_sensor_reg sensor_WhiteB_TungstenLamp2[] = {
	SensorEnd
};
static struct rk_sensor_reg *sensor_WhiteBalanceSeqe[] = {
	sensor_WhiteB_Auto,
	sensor_WhiteB_TungstenLamp1,
	sensor_WhiteB_TungstenLamp2,
	sensor_WhiteB_ClearDay,
	sensor_WhiteB_Cloudy,
	NULL,
};

static struct rk_sensor_reg sensor_Brightness0[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Brightness1[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Brightness2[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Brightness3[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Brightness4[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Brightness5[] = {
	SensorEnd
};

static struct rk_sensor_reg *sensor_BrightnessSeqe[] = {
	sensor_Brightness0,
	sensor_Brightness1,
	sensor_Brightness2,
	sensor_Brightness3,
	sensor_Brightness4,
	sensor_Brightness5,
	NULL,
};

static struct rk_sensor_reg sensor_Effect_Normal[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Effect_WandB[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Effect_Sepia[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Effect_Negative[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Effect_Bluish[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Effect_Green[] = {
	SensorEnd
};

static struct rk_sensor_reg *sensor_EffectSeqe[] = {
	sensor_Effect_Normal,
	sensor_Effect_WandB,
	sensor_Effect_Negative,
	sensor_Effect_Sepia,
	sensor_Effect_Bluish,
	sensor_Effect_Green,
	NULL,
};

static struct rk_sensor_reg sensor_Exposure0[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Exposure1[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Exposure2[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Exposure3[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Exposure4[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Exposure5[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Exposure6[] = {
	SensorEnd
};

static struct rk_sensor_reg *sensor_ExposureSeqe[] = {
	sensor_Exposure0,
	sensor_Exposure1,
	sensor_Exposure2,
	sensor_Exposure3,
	sensor_Exposure4,
	sensor_Exposure5,
	sensor_Exposure6,
	NULL,
};

static struct rk_sensor_reg sensor_Saturation0[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Saturation1[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Saturation2[] = {
	SensorEnd
};
static struct rk_sensor_reg *sensor_SaturationSeqe[] = {
	sensor_Saturation0,
	sensor_Saturation1,
	sensor_Saturation2,
	NULL,
};

static struct rk_sensor_reg sensor_Contrast0[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Contrast1[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Contrast2[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Contrast3[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Contrast4[] = {
	SensorEnd
};


static struct rk_sensor_reg sensor_Contrast5[] = {
	SensorEnd
};

static struct rk_sensor_reg sensor_Contrast6[] = {
	SensorEnd
};
static struct rk_sensor_reg *sensor_ContrastSeqe[] = {
	sensor_Contrast0,
	sensor_Contrast1,
	sensor_Contrast2,
	sensor_Contrast3,
	sensor_Contrast4,
	sensor_Contrast5,
	sensor_Contrast6,
	NULL,
};
static	struct rk_sensor_reg sensor_SceneAuto[] =
{
	SensorEnd
};

static	struct rk_sensor_reg sensor_SceneNight[] =
{
	SensorEnd
};
static struct rk_sensor_reg *sensor_SceneSeqe[] = {sensor_SceneAuto, sensor_SceneNight,NULL,};

static struct rk_sensor_reg sensor_Zoom0[] =
{
	SensorEnd
};

static struct rk_sensor_reg sensor_Zoom1[] =
{
	SensorEnd
};

static struct rk_sensor_reg sensor_Zoom2[] =
{
	SensorEnd
};


static struct rk_sensor_reg sensor_Zoom3[] =
{
	SensorEnd
};
static struct rk_sensor_reg *sensor_ZoomSeqe[] = {sensor_Zoom0, sensor_Zoom1, sensor_Zoom2, sensor_Zoom3, NULL,};

/*
* User could be add v4l2_querymenu in sensor_controls by new_usr_v4l2menu
*/
static struct v4l2_querymenu sensor_menus[] =
{
};
/*
* User could be add v4l2_queryctrl in sensor_controls by new_user_v4l2ctrl
*/
static inline int dm5886_channel_set(struct i2c_client *client,
				      struct sensor_v4l2ctrl_info_s *ctrl_info,
				      struct v4l2_ext_control *ext_ctrl)
{
	struct generic_sensor *sensor = to_generic_sensor(client);
	struct rk_sensor_sequence *sensor_series =
		sensor->info_priv.sensor_series;
	int series_num = sensor->info_priv.num_series;
	int ret = 0;
	int i;
	int id;
	int channel_ain;

	DBG("\n====%s\n",__FUNCTION__);

	if ((ext_ctrl->value < ctrl_info->qctrl->minimum) ||
	    (ext_ctrl->value > ctrl_info->qctrl->maximum)) {
		SENSOR_TR("%s(%d):channel(%d) is not support\n",__func__, __LINE__, ext_ctrl->value);
		ret = -EINVAL;
		goto end;
	}
	if (sensor->channel_id != ext_ctrl->value) {
		SENSOR_TR("%s(%d):set channel(%d)!\n", __func__, __LINE__, ext_ctrl->value);
		sensor->channel_id = ext_ctrl->value;
		id = sensor->channel_id;
		for (i = 0; i < series_num; i++)
			if ((sensor_series[i].property == SEQUENCE_PREVIEW) && (sensor_series[i].data[0].reg != SEQCMD_END))
				break;
		if (strstr(channel_infos.channel_info[id], "cvbs")) {
			ret = sscanf(channel_infos.channel_info[id], "%*s %d",&channel_ain);
			if (IS_ERR_VALUE(ret)) {
				SENSOR_TR("%s(%d): channel_infos err!\n", __func__, __LINE__);
				ret = -EINVAL;
				goto end;
			}

			sensor_series[i].data = sensor_preview_data;
			sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_NTSC;
			sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;
			dm5886_reinit_parameter(V4L2_STD_NTSC, sensor);
		}

		generic_sensor_write_array(client, sensor_series[i].data);
	}

end:
	return ret;
}

static inline int sensor_v4l2ctrl_inside_cb(struct soc_camera_device *icd,
					    struct sensor_v4l2ctrl_info_s *ctrl_info,
					    struct v4l2_ext_control *ext_ctrl,
					    bool is_set)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct generic_sensor *sensor = to_generic_sensor(client);
	int ret = 0;

	DBG("\n====%s\n",__FUNCTION__);

	switch (ctrl_info->qctrl->id) {
	case V4L2_CID_DEINTERLACE:
		{
			if (is_set) {
				SENSOR_TR("deinterlace isn't support set!\n");
				ret = -EINVAL;
				goto cb_end;
			} else {
				if ((RK_CAMERA_DEVICE_BT601_8 == sensor->info_priv.dev_sig_cnf.type)) {
					/* don't need deinterlace process */
					ext_ctrl->value = 0;
					ctrl_info->cur_value = 0;
				} else {
					ext_ctrl->value = 1;
					ctrl_info->cur_value = 1;
				}
			}
			break;
		}
	case V4L2_CID_CHANNEL:
	{
		if (is_set) {
			ret = dm5886_channel_set(client, ctrl_info, ext_ctrl);
			if (ret)
				goto cb_end;
		} else {
			ext_ctrl->value = sensor->channel_id;
			ctrl_info->cur_value = sensor->channel_id;
		}
		break;
	}
	case V4L2_CID_BRIGHTNESS:
	{
		sensor_write(client, DM5886_REG_BRIGHTNESS, ext_ctrl->value);
		break;
	}
	case V4L2_CID_CONTRAST:
	{
		sensor_write(client, DM5886_REG_CONTRAST, ext_ctrl->value);
		break;
	}
	case V4L2_CID_SATURATION:
	{
		sensor_write(client, DM5886_REG_SATURATION, ext_ctrl->value);
		break;
	}
	case V4L2_CID_HUE:
	{
		sensor_write(client, DM5886_REG_HUE, ext_ctrl->value);
		break;
	}
	default:
		{
			SENSOR_TR("%s(%d): cmd(0x%x) is unknown !\n",__func__, __LINE__, ctrl_info->qctrl->id);
			ret = -EINVAL;
		}
	}

cb_end:
	return ret;
}
static struct sensor_v4l2ctrl_usr_s sensor_controls[] = {
	{
		{
			V4L2_CID_DEINTERLACE,
			V4L2_CTRL_TYPE_BOOLEAN,
			"deinterlace",
			0,
			1,
			1,
			0
		},
		sensor_v4l2ctrl_inside_cb,
		NULL
	},
	{
		{
			V4L2_CID_CHANNEL,
			V4L2_CTRL_TYPE_INTEGER,
			"channel",
			0,
			4,
			1,
			0
		},
		sensor_v4l2ctrl_inside_cb,
		NULL
	},
	{
		{
			V4L2_CID_BRIGHTNESS,
			V4L2_CTRL_TYPE_INTEGER,
			"brightness",
			0,
			255,
			1,
			128
		},
		sensor_v4l2ctrl_inside_cb,
		NULL
	},
	{
		{
			V4L2_CID_CONTRAST,
			V4L2_CTRL_TYPE_INTEGER,
			"contrast",
			0,
			255,
			1,
			128
		},
		sensor_v4l2ctrl_inside_cb,
		NULL
	},
	{
		{
			V4L2_CID_SATURATION,
			V4L2_CTRL_TYPE_INTEGER,
			"saturation",
			0,
			255,
			1,
			128
		},
		sensor_v4l2ctrl_inside_cb,
		NULL
	},
	{
		{
			V4L2_CID_HUE,
			V4L2_CTRL_TYPE_INTEGER,
			"hue",
			-128,
			127,
			1,
			0
		},
		sensor_v4l2ctrl_inside_cb,
		NULL
	},
};

//MUST define the current used format as the first item
static struct rk_sensor_datafmt sensor_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG}
};
//static struct soc_camera_ops sensor_ops;

/*
**********************************************************
* Following is local code:
*
* Please codeing your program here
**********************************************************
*/
static int sensor_parameter_record(struct i2c_client *client)
{
	DBG("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_ae_transfer(struct i2c_client *client)
{
	DBG("\n====%s\n",__FUNCTION__);

	return 0;
}
static void dm5886_reinit_parameter(v4l2_std_id std,
				     struct generic_sensor *sensor)
{
	struct rk_sensor_sequence *series = sensor->info_priv.sensor_series;
	int num_series = sensor->info_priv.num_series;
	int i;

	switch (std) {
	case V4L2_STD_NTSC:
	case V4L2_STD_NTSC_443:
		SENSOR_PREVIEW_W = 720;
		SENSOR_PREVIEW_H = 480;
		sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_NTSC;
		sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;		
		strcpy(input_mode, "NTSC");
		break;
	case V4L2_STD_PAL:
	case V4L2_STD_PAL_M:
	case V4L2_STD_PAL_60:
		SENSOR_PREVIEW_W = 720;
		SENSOR_PREVIEW_H = 576;
		sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_PAL;
		sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;
		strcpy(input_mode, "PAL");
		break;
	default:
		SENSOR_PREVIEW_W = 720;
		SENSOR_PREVIEW_H = 576;
		sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_NTSC;
		sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;
		strcpy(input_mode, "YUV");
		break;
	}

	for (i = 0; i < ARRAY_SIZE(defrects); i++) {
		if ((defrects[i].width == SENSOR_PREVIEW_W) &&
		    (defrects[i].height == SENSOR_PREVIEW_H)) {
			SENSOR_PREVIEW_W = defrects[i].defrect.width;
			SENSOR_PREVIEW_H = defrects[i].defrect.height;

			printk("=============match %dx%d\n", SENSOR_PREVIEW_W, SENSOR_PREVIEW_H);
			memcpy(&sensor->info_priv.dev_sig_cnf.crop,&defrects[i].defrect,sizeof(defrects[i].defrect));

			if (!defrects[i].interface) {
				SENSOR_TR("%s(%d): interface is NULL\n",
					  __func__, __LINE__);
				printk("=============continue\n");
				continue;
			}

			SENSOR_TR("%s(%d): type 0x%x\n", __func__, __LINE__,sensor->info_priv.dev_sig_cnf.type);
		}
	}
		
	/*update sensor info_priv*/
	for (i = 0; i < num_series; i++) {
		series[i].gSeq_info.w = SENSOR_PREVIEW_W;
		series[i].gSeq_info.h = SENSOR_PREVIEW_H;
	}
	 
	generic_sensor_get_max_min_res(sensor->info_priv.sensor_series,sensor->info_priv.num_series,&(sensor->info_priv.max_real_res),&(sensor->info_priv.max_res),&(sensor->info_priv.min_res));

	printk("===========max_res.h=%d, max_res.w=%d ,max_real_res.h=%d, max_real_res.w=%d ,\n", sensor->info_priv.max_res.h, sensor->info_priv.max_res.w, sensor->info_priv.max_real_res.h, sensor->info_priv.max_real_res.w );
}

static void dm5886_send_uevent(struct generic_sensor *sensor)
{
	char *event_msg = NULL;
	char *envp[2];
	DBG("\n====%s\n",__FUNCTION__);

	event_msg = kasprintf(GFP_KERNEL,
			      "CVBS_NAME=DM5886 NOW_INPUT_MODE=%s RESOLUTION=%dx%d",
			      input_mode, SENSOR_PREVIEW_W, SENSOR_PREVIEW_H);

	SENSOR_TR("%s(%d): event_msg: %s\n", __func__, __LINE__, event_msg);
	envp[0] = event_msg;
	envp[1] = NULL;
	kobject_uevent_env(&(sensor->subdev.v4l2_dev->dev->kobj),KOBJ_CHANGE, envp);
}

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
	case DM5886_REG_VD_STATUS_PAL_60:
		return V4L2_STD_PAL_60;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 dm5886_status_to_v4l2(u8 status1)
{
	if (!status1)
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

static int dm5886_status(struct i2c_client *client, u32 *status,
			  v4l2_std_id *std)
{
	unsigned char status1 = 0;
	int ret = 0;

	ret = sensor_read(client, DM5886_REG_VD_STATUS, &status1);
	if (IS_ERR_VALUE(ret)) {
		SENSOR_TR("%s(%d): register read failed: 0x%x\n",__func__, __LINE__, DM5886_REG_VD_STATUS);
		return status1;
	}

	if (status1 < 0)
		return status1;

	if (status)
		*status = dm5886_status_to_v4l2(status1);
	if (std)
		*std = dm5886_std_to_v4l2(status1);

	return 0;
}

#define VIDEO_SIGNAL 1
#define VIDEO_NOSIGAL 0

static struct i2c_client *mClientSignal;

static int g_outsel;
static int g_curcha;

static void update_sequence(struct rk_sensor_reg *seq, unsigned int reg, unsigned int val)
{
	for (; seq->reg != SEQCMD_END; seq++) {
		if (reg == seq->reg) {
			seq->val = val;
			break;
		}
	}

	return;
}

int dm5886_get_outsel(void)
{
	return g_outsel;
}

int dm5886_set_outsel(int outsel)
{
	int rval;
	unsigned char mixer_cfg = RST_VD_SIGNAL_LOSS|RST_VM_SIGNAL_LOSS;
	unsigned char mixer_cha_enable;
	unsigned char mixer_cha_sel01 = CHSEL_1(1) | CHSEL_0(0);
	unsigned char mixer_cha_sel23 = CHSEL_3(2) | CHSEL_2(3);
	unsigned char vadc0_cfg1 = SWGAIN_0|SW_SEL1_VIN1A|SW_SEL2_VIN2A;
	unsigned char vadc1_cfg1 = SWGAIN_1|SW_SEL3_VIN3A|SW_SEL4_VIN4A;
	unsigned char sc_page = 0x1;
	struct rk_sensor_reg *coordinate = pal_coordinate;

#if 0
	static int count;
	int outsels[14] = {CHANNEL_NONE, CHANNEL1, CHANNEL2, CHANNEL3, CHANNEL4,
		CHANNEL5, CHANNEL6, CHANNEL7, CHANNEL8,
		QUAD_VIEW, BIVIEW_UD, T_VIEW, H_VIEW, BIVIEW_LR};

	count++;
	outsel = outsels[count%14];
#endif

	if (outsel == g_outsel) {
		return 0;
	}

	g_outsel = outsel;
	if (!mClientSignal) {
		//pr_err("invalid i2c client\n");
		return 0;
	}

	g_curcha = -1;
	if (outsel <= 0) {
		mixer_cfg |= OUT_SEL_0(SD_QUAD_VIEW);
		mixer_cha_enable = CHENNONE;
		sensor_write(mClientSignal, DM5886_REG_MIXER_CHA_ENABLE, CHENNONE);
		return 0;
	} else if (outsel <= CHANNEL8) {
		/* SD mode CVBS view */
		mixer_cfg |= OUT_SEL_0(SD_CVBS_VIEW);
		mixer_cha_enable = CHEN0;
		mixer_cha_sel01 &= 0xf0;
		mixer_cha_sel01 |= CHSEL_0((outsel - 1) & 0x3);
		g_curcha = outsel;
		sc_page = 1 << ((outsel - 1) & 0x3);
	} else if (BIVIEW_LR == outsel) {
		/* SD mode Bi-view left right, righit front and right rear camera */
		mixer_cfg |= OUT_SEL_0(SD_BIVIEW_LR);
		mixer_cha_enable = CHEN0 | CHEN1;
		mixer_cha_sel01 = CHSEL_1(2) | CHSEL_0(1);
		coordinate = pal_biview_lr_coordinate;
	} else if (BIVIEW_UD == outsel) {
		/* SD mode Bi-view up down */
		mixer_cfg |= OUT_SEL_0(SD_BIVIEW_UD);
		mixer_cha_enable = CHEN0|CHEN1;
		coordinate = pal_biview_ud_coordinate;
	} else if (T_VIEW == outsel) {
		/* SD mode T view */
		mixer_cfg |= OUT_SEL_0(SD_T_VIEW);
		mixer_cha_enable = CHEN0|CHEN1|CHEN2;
		coordinate = pal_tview_coordinate;
	} else if (H_VIEW == outsel) {
		/* SD mode H view */
		mixer_cfg |= OUT_SEL_0(SD_H_VIEW);
		mixer_cha_enable = CHEN0|CHEN1|CHEN2|CHEN3;
		coordinate = pal_hview_coordinate;
	} else {
		/* SD mode quad view */
		mixer_cfg |= OUT_SEL_0(SD_QUAD_VIEW);
		mixer_cha_enable = CHEN0|CHEN1|CHEN2|CHEN3;
		coordinate = pal_quad_coordinate;
	}

	switch (outsel) {
	case CHANNEL5:
		vadc0_cfg1 = SW_SEL1_VIN1B;
		break;
	case CHANNEL6:
		vadc0_cfg1 = SW_SEL2_VIN2B;
		break;
	case CHANNEL7:
		vadc1_cfg1 = SW_SEL3_VIN3B;
		break;
	case CHANNEL8:
		vadc1_cfg1 = SW_SEL4_VIN4B;
		break;
	default:
		break;
	}

	update_sequence(sensor_preview_data, DM5886_REG_SYSTEM_CONTROL, sc_page);
	update_sequence(sensor_preview_data, DM5886_REG_MIXER_CHA_ENABLE, mixer_cha_enable);
	update_sequence(sensor_preview_data, DM5886_REG_CHA01_SELECT, mixer_cha_sel01);
	update_sequence(sensor_preview_data, DM5886_REG_CHA23_SELECT, mixer_cha_sel23);
	update_sequence(sensor_preview_data, DM5886_REG_MIXER_CONF, mixer_cfg);
	update_sequence(sensor_preview_data, DM5886_REG_VADC0_CONF1, vadc0_cfg1);
	update_sequence(sensor_preview_data, DM5886_REG_VADC1_CONF1, vadc1_cfg1);
	rval = sensor_write_array(mClientSignal, coordinate);
	rval = sensor_write_array(mClientSignal, sensor_preview_data);
	pr_info("outsel %d, current channel %d\n", outsel, g_curcha);

	return 0;
}

int dm5886_check_signal(struct i2c_client *client,bool isCheck)
{
    int nRet = -EINVAL, i ;
    unsigned char status;
    int state = VIDEO_NOSIGAL;
	struct generic_sensor *sensor;
	struct rk_sensor_sequence *sensor_series;
	int series_num;

    if (!client)
        return nRet;

    sensor = to_generic_sensor(client);
    sensor_series = sensor->info_priv.sensor_series;
    series_num = sensor->info_priv.num_series;

    for( i = 0; i < series_num ;i++){
        if((sensor_series[i].property == SEQUENCE_INIT)&&(sensor_series[i].data[0].reg != SEQCMD_END ))
            break;
    }

    nRet = sensor_read(client, DM5886_REG_VD_STATUS, &status);
    if(IS_ERR_VALUE(nRet)){
        printk("\n status0 failed = 0x%x\n",status);
    }

	if(status){
		state = VIDEO_SIGNAL;
		sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOCKED;
	}
	else{
		state = VIDEO_NOSIGAL;
		sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOSS;
	}

    return state;
}

typedef int (*fn_cselect_t)(int channel);
typedef unsigned char (*fn_csignal_t)(void);

void register_cselect_fn(fn_cselect_t cselect);
void register_csignal_fn(fn_csignal_t csignal);

static unsigned char dm5886_readSignalStatus(void)
{
    unsigned char nTemp ;

    nTemp = dm5886_check_signal(mClientSignal,true);

    return nTemp;
}

/*
* the function is called in open sensor
*/
static int sensor_activate_cb(struct i2c_client *client)
{
	struct generic_sensor *sensor = to_generic_sensor(client);
	v4l2_std_id std;

    mClientSignal = client;

	register_csignal_fn(dm5886_readSignalStatus);
	register_cselect_fn(NULL);

	std = std_last_g;
	dm5886_status(client, NULL, &std);
	dm5886_reinit_parameter(std, sensor);
	dm5886_send_uevent(sensor);
	if (std_last_g != std) {
		pr_info("STD changed 0x%llx -> 0x%llx\n", std_last_g, std);
		std_last_g = std;
	}
			
 //   sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOCKED;
	
	if (sensor->state_check_work.state_check_wq) {
		SENSOR_DG("sensor_activate_cb: queue_delayed_work 1000ms\n");
		printk("sensor_activate_cb: queue_delayed_work 1000ms\n");
        queue_delayed_work(sensor->state_check_work.state_check_wq,&sensor->state_check_work.work,200);
	}

	return 0;
}

/*
* the function is called in close sensor
*/
static int sensor_deactivate_cb(struct i2c_client *client)
{
	int ret = 0;
	struct generic_sensor *sensor = to_generic_sensor(client);

	ret = cancel_delayed_work_sync(&sensor->state_check_work.work);
	return ret;
}
/*
* the function is called before sensor register setting in VIDIOC_S_FMT
*/
static int sensor_s_fmt_cb_th(struct i2c_client *client,struct v4l2_mbus_framefmt *mf, bool capture)
{
    if (capture) {
        sensor_parameter_record(client);
    }

    return 0;
}
/*
* the function is called after sensor register setting finished in VIDIOC_S_FMT
*/
static int sensor_s_fmt_cb_bh (struct i2c_client *client,struct v4l2_mbus_framefmt *mf, bool capture)
{
	printk("%s, fmt.width=%u, fmt.height=%u, code=%u, field=%u, colorspace=%u\n", __FUNCTION__,  mf->width, mf->height, mf->code, mf->field, mf->colorspace);

    if (capture) {
        sensor_ae_transfer(client);
    }
    return 0;
}
static int sensor_try_fmt_cb_th(struct i2c_client *client,struct v4l2_mbus_framefmt *mf)
{
	return 0;
}

static int sensor_softrest_usr_cb(struct i2c_client *client,struct rk_sensor_reg *series)
{
	return 0;
}
static int sensor_check_id_usr_cb(struct i2c_client *client,struct rk_sensor_reg *series)
{
	return 0;
}

static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
	//struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

	if (pm_msg.event == PM_EVENT_SUSPEND) {
		DBG("Suspend\n");

	} else {
		SENSOR_TR("pm_msg.event(0x%x) != PM_EVENT_SUSPEND\n",pm_msg.event);
		return -EINVAL;
	}
	return 0;
}

static int sensor_resume(struct soc_camera_device *icd)
{
	printk("\n====%s\n",__FUNCTION__);

	return 0;
}
static int sensor_mirror_cb (struct i2c_client *client, int mirror)
{

	return 0;
}
/*
* the function is v4l2 control V4L2_CID_HFLIP callback
*/
static int sensor_v4l2ctrl_mirror_cb(struct soc_camera_device *icd, struct sensor_v4l2ctrl_info_s *ctrl_info,
                                                     struct v4l2_ext_control *ext_ctrl)
{

	return 0;
}

static int sensor_flip_cb(struct i2c_client *client, int flip)
{

	return 0;
}
/*
* the function is v4l2 control V4L2_CID_VFLIP callback
*/
static int sensor_v4l2ctrl_flip_cb(struct soc_camera_device *icd, struct sensor_v4l2ctrl_info_s *ctrl_info,
                                                     struct v4l2_ext_control *ext_ctrl)
{
	return 0;
}
/*
* the functions are focus callbacks
*/
static int sensor_focus_init_usr_cb(struct i2c_client *client){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_focus_af_single_usr_cb(struct i2c_client *client){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_focus_af_near_usr_cb(struct i2c_client *client){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_focus_af_far_usr_cb(struct i2c_client *client){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_focus_af_specialpos_usr_cb(struct i2c_client *client,int pos){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_focus_af_const_usr_cb(struct i2c_client *client){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}
static int sensor_focus_af_const_pause_usr_cb(struct i2c_client *client)
{
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}
static int sensor_focus_af_close_usr_cb(struct i2c_client *client){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

static int sensor_focus_af_zoneupdate_usr_cb(struct i2c_client *client, int *zone_tm_pos)
{
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

/*
face defect call back
*/
static int 	sensor_face_detect_usr_cb(struct i2c_client *client,int on){
    printk("\n====%s\n",__FUNCTION__);

	return 0;
}

/*
*   The function can been run in sensor_init_parametres which run in sensor_probe, so user can do some
* initialization in the function.
*/
static void dm5886_check_signal_work(struct work_struct *work)
{
	struct rk_state_check_work *state_check_work =
		container_of(work, struct rk_state_check_work, work.work);
	struct generic_sensor *sensor =
		container_of(state_check_work,
			     struct generic_sensor,
			     state_check_work);
	struct i2c_client *client = sensor->client;
	v4l2_std_id std;

	if ((sensor->info_priv.dev_sig_cnf.type ==
	     RK_CAMERA_DEVICE_BT601_8))
		return;

	dm5886_status(client, NULL, &std);
	if (std_last_g != std) {
		dm5886_reinit_parameter(std, sensor);
		dm5886_send_uevent(sensor);
		std_last_g = std;
		SENSOR_TR("%s(%d):now mode %s\n",
			  __func__, __LINE__, input_mode);
	}
	
	queue_delayed_work(sensor->state_check_work.state_check_wq,
			   &sensor->state_check_work.work, 300);
	

}
static void sensor_init_parameters_user(struct specific_sensor* spsensor,struct soc_camera_device *icd)
{
	/* init work_queue for state_check */
	INIT_DELAYED_WORK(&spsensor->common_sensor.state_check_work.work,
			  dm5886_check_signal_work);
	spsensor->common_sensor.state_check_work.state_check_wq =
		create_singlethread_workqueue(SENSOR_NAME_STRING(_state_check_workqueue));
	if (spsensor->common_sensor.state_check_work.state_check_wq == NULL) {
		SENSOR_TR("%s(%d): %s create failed.\n", __func__, __LINE__,
			  SENSOR_NAME_STRING(_state_check_workqueue));
		BUG();
	}

	memcpy(&spsensor->common_sensor.info_priv.dev_sig_cnf,
	       &dev_info[0], sizeof(dev_info));
	spsensor->common_sensor.crop_percent = 0;
	spsensor->common_sensor.channel_id = 0;
}

/*
**It is not allowed to modify the following code
*/
sensor_init_parameters_default_code();

sensor_v4l2_struct_initialization();

sensor_probe_default_code();

sensor_remove_default_code();

sensor_driver_default_module_code();

