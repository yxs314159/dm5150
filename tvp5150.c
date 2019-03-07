
#include "generic_sensor.h"
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "tvp5150_reg.h"

static int version = KERNEL_VERSION(0,1,3);
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
#define SENSOR_NAME RK29_CAM_SENSOR_TVP5150
#define SENSOR_V4L2_IDENT V4L2_IDENT_TVP5150
#define SENSOR_ID 0X51
#define SENSOR_BUS_PARAM  (SOCAM_MASTER |\
                           SOCAM_PCLK_SAMPLE_RISING|\
                           SOCAM_HSYNC_ACTIVE_HIGH|\
					       SOCAM_VSYNC_ACTIVE_HIGH |	\
                           SOCAM_DATA_ACTIVE_HIGH |\
                           SOCAM_DATAWIDTH_8  |\
                           SOCAM_MCLK_24MHZ)
#define TVP5150_STATUS5_REG		0x8c
#define TVP5150_STATUS5_IN_LOCK		0x01
#define TVP5150_STATUS5_AUTOD_MASK	0x0f
#define TVP5150_STATUS5_AUTOD_SWITCH	0x80
#define TVP5150_STATUS5_AUTOD_STD		0x00
#define TVP5150_STATUS5_AUTOD_NTSC_M_J	0x01
#define TVP5150_STATUS5_AUTOD_PAL_B_D_N 0X03
#define TVP5150_STATUS5_AUTOD_PAL_M		0X05
#define TVP5150_STATUS5_AUTOD_PAL_NC	0X07
#define TVP5150_STATUS5_AUTOD_NTSC_4_43 0x09
#define TVP5150_STATUS5_AUTOD_SECAM		0x0b

//ADD TVP5150 Register
#define TVP5150_VD_IN_SRC_SEL_1      0x00 /* Video input source selection #1 */
#define TVP5150_ANAL_CHL_CTL         0x01 /* Analog channel controls */
#define TVP5150_OP_MODE_CTL          0x02 /* Operation mode controls */
#define TVP5150_MISC_CTL             0x03 /* Miscellaneous controls */
#define TVP5150_AUTOSW_MSK           0x04 /* Autoswitch mask: TVP5150A / TVP5150AM */

/* Reserved 05h */
#define TVP5150_COLOR_KIL_THSH_CTL   0x06 /* Color killer threshold control */
#define TVP5150_LUMA_PROC_CTL_1      0x07 /* Luminance processing control #1 */
#define TVP5150_LUMA_PROC_CTL_2      0x08 /* Luminance processing control #2 */
#define TVP5150_BRIGHT_CTL           0x09 /* Brightness control */
#define TVP5150_SATURATION_CTL       0x0a /* Color saturation control */
#define TVP5150_HUE_CTL              0x0b /* Hue control */
#define TVP5150_CONTRAST_CTL         0x0c /* Contrast control */
#define TVP5150_DATA_RATE_SEL        0x0d /* Outputs and data rates select */
#define TVP5150_LUMA_PROC_CTL_3      0x0e /* Luminance processing control #3 */
#define TVP5150_CONF_SHARED_PIN      0x0f /* Configuration shared pins */

/* Reserved 10h */
#define TVP5150_ACT_VD_CROP_ST_MSB   0x11 /* Active video cropping start MSB */
#define TVP5150_ACT_VD_CROP_ST_LSB   0x12 /* Active video cropping start LSB */
#define TVP5150_ACT_VD_CROP_STP_MSB  0x13 /* Active video cropping stop MSB */
#define TVP5150_ACT_VD_CROP_STP_LSB  0x14 /* Active video cropping stop LSB */
#define TVP5150_GENLOCK              0x15 /* Genlock/RTC */
#define TVP5150_HORIZ_SYNC_START     0x16 /* Horizontal sync start */

/* Reserved 17h */
#define TVP5150_VERT_BLANKING_START 0x18 /* Vertical blanking start */
#define TVP5150_VERT_BLANKING_STOP  0x19 /* Vertical blanking stop */
#define TVP5150_CHROMA_PROC_CTL_1   0x1a /* Chrominance processing control #1 */
#define TVP5150_CHROMA_PROC_CTL_2   0x1b /* Chrominance processing control #2 */
#define TVP5150_INT_RESET_REG_B     0x1c /* Interrupt reset register B */
#define TVP5150_INT_ENABLE_REG_B    0x1d /* Interrupt enable register B */
#define TVP5150_INTT_CONFIG_REG_B   0x1e /* Interrupt configuration register B */

/* Reserved 1Fh-27h */
#define TVP5150_VIDEO_STD           0x28 /* Video standard */

/* Reserved 29h-2bh */
#define TVP5150_CB_GAIN_FACT        0x2c /* Cb gain factor */
#define TVP5150_CR_GAIN_FACTOR      0x2d /* Cr gain factor */
#define TVP5150_MACROVISION_ON_CTR  0x2e /* Macrovision on counter */
#define TVP5150_MACROVISION_OFF_CTR 0x2f /* Macrovision off counter */
#define TVP5150_REV_SELECT          0x30 /* revision select (TVP5150AM1 only) */

/* Reserved	31h-7Fh */
#define TVP5150_MSB_DEV_ID          0x80 /* MSB of device ID */
#define TVP5150_LSB_DEV_ID          0x81 /* LSB of device ID */
#define TVP5150_ROM_MAJOR_VER       0x82 /* ROM major version */
#define TVP5150_ROM_MINOR_VER       0x83 /* ROM minor version */
#define TVP5150_VERT_LN_COUNT_MSB   0x84 /* Vertical line count MSB */
#define TVP5150_VERT_LN_COUNT_LSB   0x85 /* Vertical line count LSB */
#define TVP5150_INT_STATUS_REG_B    0x86 /* Interrupt status register B */
#define TVP5150_INT_ACTIVE_REG_B    0x87 /* Interrupt active register B */
#define TVP5150_STATUS_REG_1        0x88 /* Status register #1 */
#define TVP5150_STATUS_REG_2        0x89 /* Status register #2 */
#define TVP5150_STATUS_REG_3        0x8a /* Status register #3 */
#define TVP5150_STATUS_REG_4        0x8b /* Status register #4 */
//#define TVP5150_STATUS_REG_5        0x8c /* Status register #5 */
/* Reserved	8Dh-8Fh */
 /* Closed caption data registers */
#define TVP5150_CC_DATA_INI         0x90
#define TVP5150_CC_DATA_END         0x93

 /* WSS data registers */
#define TVP5150_WSS_DATA_INI        0x94
#define TVP5150_WSS_DATA_END        0x99

/* VPS data registers */
#define TVP5150_VPS_DATA_INI        0x9a
#define TVP5150_VPS_DATA_END        0xa6

/* VITC data registers */
#define TVP5150_VITC_DATA_INI       0xa7
#define TVP5150_VITC_DATA_END       0xaf

#define TVP5150_VBI_FIFO_READ_DATA  0xb0 /* VBI FIFO read data */

/* Teletext filter 1 */
#define TVP5150_TELETEXT_FIL1_INI  0xb1
#define TVP5150_TELETEXT_FIL1_END  0xb5

/* Teletext filter 2 */
#define TVP5150_TELETEXT_FIL2_INI  0xb6
#define TVP5150_TELETEXT_FIL2_END  0xba

#define TVP5150_TELETEXT_FIL_ENA    0xbb /* Teletext filter enable */
/* Reserved	BCh-BFh */
#define TVP5150_INT_STATUS_REG_A    0xc0 /* Interrupt status register A */
#define TVP5150_INT_ENABLE_REG_A    0xc1 /* Interrupt enable register A */
#define TVP5150_INT_CONF            0xc2 /* Interrupt configuration */
#define TVP5150_VDP_CONF_RAM_DATA   0xc3 /* VDP configuration RAM data */
#define TVP5150_CONF_RAM_ADDR_LOW   0xc4 /* Configuration RAM address low byte */
#define TVP5150_CONF_RAM_ADDR_HIGH  0xc5 /* Configuration RAM address high byte */
#define TVP5150_VDP_STATUS_REG      0xc6 /* VDP status register */
#define TVP5150_FIFO_WORD_COUNT     0xc7 /* FIFO word count */
#define TVP5150_FIFO_INT_THRESHOLD  0xc8 /* FIFO interrupt threshold */
#define TVP5150_FIFO_RESET          0xc9 /* FIFO reset */
#define TVP5150_LINE_NUMBER_INT     0xca /* Line number interrupt */
#define TVP5150_PIX_ALIGN_REG_LOW   0xcb /* Pixel alignment register low byte */
#define TVP5150_PIX_ALIGN_REG_HIGH  0xcc /* Pixel alignment register high byte */
#define TVP5150_FIFO_OUT_CTRL       0xcd /* FIFO output control */
/* Reserved	CEh */
#define TVP5150_FULL_FIELD_ENA      0xcf /* Full field enable 1 */

/* Line mode registers */
#define TVP5150_LINE_MODE_INI       0xd0
#define TVP5150_LINE_MODE_END       0xfb

#define TVP5150_FULL_FIELD_MODE_REG 0xfc /* Full field mode register */
static int SENSOR_PREVIEW_W = 720;
static int SENSOR_PREVIEW_H = 576;
volatile v4l2_std_id std_last_g = V4L2_STD_PAL;



//by freehua
#if 0
static struct rk_camera_device_signal_config dev_info[] = {
	{
		.type = RK_CAMERA_DEVICE_CVBS_NTSC,
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.crop = {
			.top = 8,
			.left = 4,
			.width = 720,
			.height = 462
		}
	}
};
#else
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

#endif 

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
	//by freehua
	#if 0
	{
		.width = 720,
		.height = 480,
		.defrect = {
			.top = 0,
			.left = 0,
			.width = 720,
			.height = 480
		},
		.interface = "bt601_8"
	}
	#endif
};

static struct rk_camera_device_channel_info channel_infos = {
	.channel_total = 1,
	.default_id = 0,
	.channel_info = {
		"cvbs 1",
	}
};

static int now_channel_value = 0x00;
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
static void tvp5150_reinit_parameter(v4l2_std_id std,
				     struct generic_sensor *sensor);
static void tvp5150_send_uevent(struct generic_sensor *sensor);

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
	SensorRegVal(0x80, 0x0),
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

static struct rk_sensor_reg sensor_softreset_data[] = {
	/*SensorRegVal(0x0f, 0x00 | 0x80),*/
	SensorEnd
};

static struct rk_sensor_reg sensor_preview_data[] = {
	/* autodetect cvbs in ntsc/pal/secam 8-bit 422 encode */
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x0d
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x60
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x00
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0x80
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x80
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x47
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x00
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x14
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x28 */
		TVP5150_VIDEO_STD,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},

	SensorEnd
};

static struct rk_sensor_reg sensor_WhiteB_Auto[] = {
	SensorEnd
};
static	struct rk_sensor_reg sensor_WhiteB_Cloudy[] = {
	SensorEnd
};
static int tvp5150_log_status(struct i2c_client *sd)
{
    u8 value,value1, value2;
    sensor_read(sd, TVP5150_VD_IN_SRC_SEL_1,&value);
	printk("tvp5150: Video input source selection #1 = 0x%02x\n",value);
    sensor_read(sd, TVP5150_ANAL_CHL_CTL,&value);
	printk("tvp5150: Analog channel controls = 0x%02x\n",value);
    sensor_read(sd, TVP5150_OP_MODE_CTL,&value);
	printk("tvp5150: Operation mode controls = 0x%02x\n",value);
    sensor_read(sd, TVP5150_MISC_CTL,&value);
	printk("tvp5150: Miscellaneous controls = 0x%02x\n",value);
    sensor_read(sd, TVP5150_AUTOSW_MSK,&value);
	printk("tvp5150: Autoswitch mask= 0x%02x\n",value);
    sensor_read(sd, TVP5150_COLOR_KIL_THSH_CTL,&value);
	printk("tvp5150: Color killer threshold control = 0x%02x\n",value);
    sensor_read(sd, TVP5150_LUMA_PROC_CTL_1,&value);
    sensor_read(sd, TVP5150_LUMA_PROC_CTL_2,&value1);
    sensor_read(sd, TVP5150_LUMA_PROC_CTL_3,&value2);
	printk("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",value,value1,value2);
    sensor_read(sd, TVP5150_BRIGHT_CTL,&value);
    printk("tvp5150: Brightness control = 0x%02x\n",value);
    sensor_read(sd, TVP5150_SATURATION_CTL,&value);
	printk("tvp5150: Color saturation control = 0x%02x\n",value);
    sensor_read(sd, TVP5150_HUE_CTL,&value);
	printk("tvp5150: Hue control = 0x%02x\n",value);
    sensor_read(sd, TVP5150_CONTRAST_CTL,&value);
	printk("tvp5150: Contrast control = 0x%02x\n",value);
#if 0
	printk("tvp5150: Outputs and data rates select = 0x%02x\n",sensor_read(sd, TVP5150_DATA_RATE_SEL));
	printk("tvp5150: Configuration shared pins = 0x%02x\n",sensor_read(sd, TVP5150_CONF_SHARED_PIN));
	printk("tvp5150: Active video cropping start = 0x%02x%02x\n",sensor_read(sd, TVP5150_ACT_VD_CROP_ST_MSB),sensor_read(sd, TVP5150_ACT_VD_CROP_ST_LSB));
    printk("tvp5150: Active video cropping stop  = 0x%02x%02x\n",sensor_read(sd, TVP5150_ACT_VD_CROP_STP_MSB),sensor_read(sd, TVP5150_ACT_VD_CROP_STP_LSB));
    printk("tvp5150: Genlock/RTC = 0x%02x\n",sensor_read(sd, TVP5150_GENLOCK));
    printk("tvp5150: Horizontal sync start = 0x%02x\n",sensor_read(sd, TVP5150_HORIZ_SYNC_START));
	printk("tvp5150: Vertical blanking start = 0x%02x\n",sensor_read(sd, TVP5150_VERT_BLANKING_START));
	printk("tvp5150: Vertical blanking stop = 0x%02x\n",sensor_read(sd, TVP5150_VERT_BLANKING_STOP));
	printk("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",sensor_read(sd, TVP5150_CHROMA_PROC_CTL_1),sensor_read(sd, TVP5150_CHROMA_PROC_CTL_2));
	printk("tvp5150: Interrupt reset register B = 0x%02x\n",sensor_read(sd, TVP5150_INT_RESET_REG_B));
	printk("tvp5150: Interrupt enable register B = 0x%02x\n",sensor_read(sd, TVP5150_INT_ENABLE_REG_B));
	printk("tvp5150: Interrupt configuration register B = 0x%02x\n",sensor_read(sd, TVP5150_INTT_CONFIG_REG_B));
	printk("tvp5150: Video standard = 0x%02x\n",sensor_read(sd, TVP5150_VIDEO_STD));
	printk("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",sensor_read(sd, TVP5150_CB_GAIN_FACT),sensor_read(sd, TVP5150_CR_GAIN_FACTOR));
	printk("tvp5150: Macrovision on counter = 0x%02x\n",sensor_read(sd, TVP5150_MACROVISION_ON_CTR));
	printk("tvp5150: Macrovision off counter = 0x%02x\n",sensor_read(sd, TVP5150_MACROVISION_OFF_CTR));
	printk("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",(sensor_read(sd, TVP5150_REV_SELECT) & 1) ? 3 : 4);
	printk("tvp5150: Device ID = %02x%02x\n",sensor_read(sd, TVP5150_MSB_DEV_ID),sensor_read(sd, TVP5150_LSB_DEV_ID));
	printk("tvp5150: ROM version = (hex) %02x.%02x\n",sensor_read(sd, TVP5150_ROM_MAJOR_VER),sensor_read(sd, TVP5150_ROM_MINOR_VER));
	printk("tvp5150: Vertical line count = 0x%02x%02x\n",sensor_read(sd, TVP5150_VERT_LN_COUNT_MSB),sensor_read(sd, TVP5150_VERT_LN_COUNT_LSB));
	printk("tvp5150: Interrupt status register B = 0x%02x\n",sensor_read(sd, TVP5150_INT_STATUS_REG_B));
	printk("tvp5150: Interrupt active register B = 0x%02x\n",sensor_read(sd, TVP5150_INT_ACTIVE_REG_B));
	printk("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",sensor_read(sd, TVP5150_STATUS_REG_1),sensor_read(sd, TVP5150_STATUS_REG_2),sensor_read(sd, TVP5150_STATUS_REG_3),sensor_read(sd, TVP5150_STATUS_REG_4),sensor_read(sd, TVP5150_STATUS_REG_5));
	printk("tvp5150: Teletext filter enable = 0x%02x\n",sensor_read(sd, TVP5150_TELETEXT_FIL_ENA));
	printk("tvp5150: Interrupt status register A = 0x%02x\n",sensor_read(sd, TVP5150_INT_STATUS_REG_A));
	printk("tvp5150: Interrupt enable register A = 0x%02x\n",sensor_read(sd, TVP5150_INT_ENABLE_REG_A));
	printk("tvp5150: Interrupt configuration = 0x%02x\n",sensor_read(sd, TVP5150_INT_CONF));
	printk("tvp5150: VDP status register = 0x%02x\n",sensor_read(sd, TVP5150_VDP_STATUS_REG));
	printk("tvp5150: FIFO word count = 0x%02x\n",sensor_read(sd, TVP5150_FIFO_WORD_COUNT));
	printk("tvp5150: FIFO interrupt threshold = 0x%02x\n",sensor_read(sd, TVP5150_FIFO_INT_THRESHOLD));
	printk("tvp5150: FIFO reset = 0x%02x\n",sensor_read(sd, TVP5150_FIFO_RESET));
	printk("tvp5150: Line number interrupt = 0x%02x\n",sensor_read(sd, TVP5150_LINE_NUMBER_INT));
	printk("tvp5150: Pixel alignment register = 0x%02x%02x\n",sensor_read(sd, TVP5150_PIX_ALIGN_REG_HIGH),sensor_read(sd, TVP5150_PIX_ALIGN_REG_LOW));
	printk("tvp5150: FIFO output control = 0x%02x\n",sensor_read(sd, TVP5150_FIFO_OUT_CTRL));
	printk("tvp5150: Full field enable = 0x%02x\n",sensor_read(sd, TVP5150_FULL_FIELD_ENA));
	printk("tvp5150: Full field mode register = 0x%02x\n",sensor_read(sd, TVP5150_FULL_FIELD_MODE_REG));
#endif
	return 0;
}
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
static inline int tvp5150_channel_set(struct i2c_client *client,
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

			now_channel_value=0x00;
			sensor_series[i].data = sensor_preview_data;
			sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_NTSC;
			sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;
			tvp5150_reinit_parameter(V4L2_STD_NTSC, sensor);
		}

		sensor_write(client, 0x00, now_channel_value);
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
			ret = tvp5150_channel_set(client, ctrl_info, ext_ctrl);
			if (ret)
				goto cb_end;
		} else {
			ext_ctrl->value = sensor->channel_id;
			ctrl_info->cur_value = sensor->channel_id;
		}
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
	}
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
static void tvp5150_reinit_parameter(v4l2_std_id std,
				     struct generic_sensor *sensor)
{
	struct rk_sensor_sequence *series = sensor->info_priv.sensor_series;
	int num_series = sensor->info_priv.num_series;
	int i;

	if (std == V4L2_STD_NTSC || std == V4L2_STD_NTSC_443) {
		SENSOR_PREVIEW_W = 720;
		SENSOR_PREVIEW_H = 480;
		sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_NTSC;
		sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;
					
		strcpy(input_mode, "NTSC");
	} else if (std == V4L2_STD_PAL || std == TVP5150_STATUS5_AUTOD_PAL_B_D_N || std == V4L2_STD_PAL_M) {
		SENSOR_PREVIEW_W = 720;
		SENSOR_PREVIEW_H = 576;
		sensor->info_priv.dev_sig_cnf.type = RK_CAMERA_DEVICE_CVBS_PAL;
		sensor->info_priv.dev_sig_cnf.code =V4L2_MBUS_FMT_UYVY8_2X8;
		strcpy(input_mode, "PAL");
	} else {
		SENSOR_PREVIEW_W = 720;
		SENSOR_PREVIEW_H = 480;
		strcpy(input_mode, "YUV");
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

static void tvp5150_send_uevent(struct generic_sensor *sensor)
{
	char *event_msg = NULL;
	char *envp[2];
	DBG("\n====%s\n",__FUNCTION__);

	event_msg = kasprintf(GFP_KERNEL,
			      "CVBS_NAME=TVP5150 NOW_INPUT_MODE=%s RESOLUTION=%dx%d",
			      input_mode, SENSOR_PREVIEW_W, SENSOR_PREVIEW_H);

	SENSOR_TR("%s(%d): event_msg: %s\n", __func__, __LINE__, event_msg);
	envp[0] = event_msg;
	envp[1] = NULL;
	kobject_uevent_env(&(sensor->subdev.v4l2_dev->dev->kobj),KOBJ_CHANGE, envp);
}

static v4l2_std_id tvp5150_std_to_v4l2(u8 status1)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!(status1 & TVP5150_STATUS5_IN_LOCK))
		return V4L2_STD_UNKNOWN;

	switch (status1 & TVP5150_STATUS5_AUTOD_MASK) {
	case TVP5150_STATUS5_AUTOD_NTSC_M_J:
		return V4L2_STD_NTSC;
	case TVP5150_STATUS5_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case TVP5150_STATUS5_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	case TVP5150_STATUS5_AUTOD_PAL_B_D_N:
		return V4L2_STD_PAL;
	case TVP5150_STATUS5_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static u32 tvp5150_status_to_v4l2(u8 status1)
{
	if (!(status1 & TVP5150_STATUS5_IN_LOCK))
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

static int tvp5150_status(struct i2c_client *client, u32 *status,
			  v4l2_std_id *std)
{
	unsigned char status1 = 0;
	int ret = 0;

	ret = sensor_read(client, TVP5150_STATUS5_REG, &status1);

	if (IS_ERR_VALUE(ret)) {
		SENSOR_TR("%s(%d): register read failed: 0x%x\n",__func__, __LINE__, TVP5150_STATUS5_REG);
		return status1;
	}

	if (status1 < 0)
		return status1;

	if (status)
		*status = tvp5150_status_to_v4l2(status1);
	if (std)
		*std = tvp5150_std_to_v4l2(status1);

	return 0;
}
#define VIDEO_SIGNAL 1
#define VIDEO_NOSIGAL 0
unsigned char nSignalFlag = 0;

struct i2c_client *mClientSignal;

int tvp5150_check_signal(struct i2c_client *client,bool isCheck)
{
#if 1
    int nRet = -EINVAL, i ;
    unsigned char status,lastSatus;
    int state = VIDEO_NOSIGAL;

    if (!client)
        return nRet;

    struct generic_sensor *sensor = to_generic_sensor(client);
    struct rk_sensor_sequence *sensor_series = sensor->info_priv.sensor_series;
    int series_num = sensor->info_priv.num_series;

    for( i = 0; i < series_num ;i++){
        if((sensor_series[i].property == SEQUENCE_INIT)&&(sensor_series[i].data[0].reg != SEQCMD_END ))
            break;
    }

    nRet = sensor_read(client,0x88,&status);
    if(IS_ERR_VALUE(nRet)){
        printk("\n status0 failed = 0x%x\n",status);
    }

	//by freehua modified
	#if 0
//    DBG("\n start state 0x%x := 0x%x \n",status,(status>>4));
    if((status & 0x0f) == 0x0){
        state = VIDEO_NOSIGAL;
       // DBG("\n VIDEO_NOSIGAL state=%d \n",state);

        sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOSS;
    }else{
        state = VIDEO_SIGNAL;
      //  DBG("\n VIDEO_SIGNAL state=%d \n",state);
        sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOCKED;
    }
	#else
	  //add by freehua
	  if((status & 0x06) == 0x06){
	  	state = VIDEO_SIGNAL;
		sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOCKED;
	  }
	  else{
		state = VIDEO_NOSIGAL;
		sensor->info_priv.video_state = RK_CAM_INPUT_VIDEO_STATE_LOSS;
	  }
	  	
	  
	#endif
#endif
    return state;
}

typedef int (*fn_cselect_t)(int channel);
typedef unsigned char (*fn_csignal_t)(void);

void register_cselect_fn(fn_cselect_t cselect);
void register_csignal_fn(fn_csignal_t csignal);

#define CVBS_BACK 5
static int tvp5150_channel_select(int channel)
{
    int rval;
    int val;

    if (!mClientSignal) {
		return -1;
	}

    val = CVBS_BACK==channel?0x0:0x2;
    sensor_preview_data[0].val = val;
    rval = sensor_write(mClientSignal, TVP5150_VD_IN_SRC_SEL_1, val);

    return rval;
}

static unsigned char tvp5150_readSignalStatus()
{
    unsigned char nTemp ;

    nTemp = tvp5150_check_signal(mClientSignal,true);

    return nTemp;
}

/*
* the function is called in open sensor
*/
static int sensor_activate_cb(struct i2c_client *client)
{
	struct generic_sensor *sensor = to_generic_sensor(client);
	DBG("\n==== open%s\n",__FUNCTION__);
	printk("\n====%s\n",__FUNCTION__);
    mClientSignal = client;

	register_csignal_fn(tvp5150_readSignalStatus);
	register_cselect_fn(tvp5150_channel_select);
	//by freehua
	sensor_write(client, 0x00, 0x00);
	
	//by freehua
	v4l2_std_id std = std_last_g;
	tvp5150_status(client, NULL, &std);
	tvp5150_reinit_parameter(std, sensor);
	tvp5150_send_uevent(sensor);
	std_last_g = std;
			
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
	DBG("\n==== close %s\n",__FUNCTION__);
	printk("\n====%s\n",__FUNCTION__);

	ret = cancel_delayed_work_sync(&sensor->state_check_work.work);
	return ret;
}
/*
* the function is called before sensor register setting in VIDIOC_S_FMT
*/
static int sensor_s_fmt_cb_th(struct i2c_client *client,struct v4l2_mbus_framefmt *mf, bool capture)
{
	DBG("\n====%s\n",__FUNCTION__);
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
	printk("\n====%s\n",__FUNCTION__);

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
static void tvp5150_check_signal_work(struct work_struct *work)
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

	tvp5150_status(client, NULL, &std);
	if (std_last_g != std) {
		tvp5150_reinit_parameter(std, sensor);
		tvp5150_send_uevent(sensor);
		std_last_g = std;
		SENSOR_TR("%s(%d):now mode %s\n",
			  __func__, __LINE__, input_mode);
	}
	
	queue_delayed_work(sensor->state_check_work.state_check_wq,
			   &sensor->state_check_work.work, 300);
	

}
static void sensor_init_parameters_user(struct specific_sensor* spsensor,struct soc_camera_device *icd)
{
    DBG("\n====%s\n",__FUNCTION__);
	printk("\n====%s\n",__FUNCTION__);

	/* init work_queue for state_check */
	INIT_DELAYED_WORK(&spsensor->common_sensor.state_check_work.work,
			  tvp5150_check_signal_work);
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

