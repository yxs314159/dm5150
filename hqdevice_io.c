#include <linux/fb.h>
#include <linux/gpio_detection.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include<linux/input.h>
#include <linux/device.h>
#include <linux/watchdog.h>
#include "hqdevice_io.h"

struct gpio_data {
	struct gpio_detection *parent;
	const char *name;
	struct device dev;
	int notify;
	int gpio;
	int atv_val;
	int val;
	int irq;
	int wakeup;
};

struct gpio_detection {
	struct class_attribute cls_attr;
	struct device *dev;
	int num;
	struct gpio_data *data;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct notifier_block fb_notifier;
	struct wake_lock wake_lock;
	int mirror;
	int type;
	int info;
};

static int uart1_rx_det;
static int bt_num = 0;//bt wifi  bug
static struct class *gpio_detection_class;
static BLOCKING_NOTIFIER_HEAD(gpio_det_notifier_list);
static int system_suspend;
#define CSTR(x) #x

#ifdef CONFIG_SOC_CAMERA_DM5886
extern int dm5886_get_outsel(void);
extern int dm5886_set_outsel(int outsel);
#endif

#ifdef CONFIG_HQDEVICE_CM
extern unsigned char cameraFlag;
#else
unsigned char cameraFlag;
#endif
extern unsigned char audioFlag;
extern unsigned int  adc_value;
extern int hqdevice_set_eq(unsigned *pValue);
extern int hqdevice_set_lrf_volume(unsigned char *pValue);
extern int hqdevice_tda7719_channel(unsigned short channel);
extern int hqdevice_set_volume(unsigned char *pValue);
extern int hq_channel_video_select(int nValue);
extern int hqdevice_set_mix(unsigned char *pValue);
extern int hqdevice_set_second(unsigned char *pValue);

//add hq_debug
static ssize_t hq_debug_state_show(struct device *pdev, struct device_attribute *attr,char *buf);
static ssize_t hq_debug_state_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(hq_debug, S_IRUGO | S_IWUSR, hq_debug_state_show, hq_debug_state_store);
static struct device_attribute *hq_debug_attributes[] = {
    &dev_attr_hq_debug,
    NULL
};

static ssize_t hq_debug_state_show(struct device *pdev, struct device_attribute *attr,char *buf){
    return sprintf(buf, "0\n");
}

static ssize_t hq_debug_state_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size){

	printk("state_store:%d\r\n",*buf);
	//open
	if(*buf == 49){
		printk("debug open\n");
     }
	//close
	if(*buf == 48)
        printk("debug close\n");

	return size;
}


static void hq_debug_sysfs(struct miscdevice *dev){
	struct device_attribute **attrs = hq_debug_attributes;
    struct device_attribute *attr;
    int err;

	dev_set_drvdata(dev->this_device, dev);

    while ((attr = *attrs++)) {
    err = device_create_file(dev->this_device, attr);
    if (err) {
        return err;
    }
    }
}


static int hope_read(struct file * file,char * buf,size_t count,loff_t * f_ops)
{
	int nRet = 0;

	return nRet;
}

static int hope_write(struct file * file,const char * buf,size_t count,loff_t * f_ops)
{
	int nRet = 0;

	return nRet;
}

#define HQ_BT_CTRL_ONOFF              0X3000          //bt power on of
#define HQ_GET_AUDIO_CHANNEL          0x3001          //GET audio chanel:0 radio 1 MIC 4 arm 5 aux
#define HQ_USB1_CTRL_ONOFF            0x3002          //SET USB1 power off
#define HQ_SET_LRF_CHANNEL_VOL        0X3003          //LRF_CHANNEL SET VLOUME
#define HQ_BACKLIGH_ENABLE_ONOFF      0X3008          //blacklight onoff
#define HQ_VEHICLE_GET_STATUS         0x3004          //0 close all cvbs1 cvbs2 cvbs3 cvbs4 ccd5 reserver6 channel
#define HQ_AUDOI_STANDY_CTRL_ONOFF    0x3005          //Audio standy ctrl
#define HQ_AUDIO_MUTE_CTRL_ONOFF      0x3006          //Mute on off
#define HQ_VEHICLE_CAMERA_CTRL_ONOFF  0x3007          //Vehicle camera onoff
#define HQ_USB_MODE_SWITCH_ONOFF      0x3009          //Usb mode switch onoff
#define HQ_CAMERA_CTRL_ONOFF          0x300a          //camera power on off
#define HQ_COOLING_FAN_CTRL_ONOFF     0x300c          //Cooling Fan power on off
#define HQ_GPS_CTRL_ONOFF             0x300d          //GPS power on off
#define HQ_RADIO_CTRL_ONOFF           0x300e          //Radio power on off
//#define HQ_SET_VIDEO_CHANNEL        0x300f          ////0 close all cvbs1 cvbs2 cvbs3 cvbs4 ccd5 reserver6 channel
#define HQ_GET_HQDEVIEIO_STATUS       0x300f          //set GET_HQDEVIEIO_STATUS
#define HQ_SET_AUDIO_VOLUME           0x3010          //set 7719 audio volume
#define HQ_SET_AUDIO_MIX_CHANNEL      0x3011          //set audio mix channel
#define HQ_BT_HOST_WAKE		      0x3012	      //set BT_REG provided to the manufacturer to use

#define HQ_SET_DERECTION_OUT          0X5000          //set direction out
#define HQ_SET_DERECTION_INPUT        0x5001          //set direction input
#define HQ_SET_OUTPUT_HIGH            0x5002          //set output high
#define HQ_SET_OUTPUT_LOW             0x5003          //set output low
#define HQ_SET_CAMERA_CHANNEL         0x6000          //set camera channel 1-4· 5 ���� 0 close
#define HQ_SET_IMAGE_CAMERA           0x6001          //set camera Image:0 not,1 horizen 2 ve 3 h_v
#define HQ_SET_AUDIOEQ_VALE           0x6002          //set audioEq value TRE 0x00-0x10[-15db 15db]
#define HQ_GET_ADC_VAL                0x7000          //get ADC VALUE
#define HQ_GET_CAMERA_SIGNAL          0x7001          //get camera signal 1:siganl 0:nosignal

//6106AM
//#define HQ_GPS_CTRL_ONOFF			  0x300d
#define HQ_GET_UART1_RX_TEST_DET	  0x7101		//测试机UART接入检测，上升沿或下降沿有效。
#define HQ_GET_IO1_RADAR			  0x7102		//雷达板预留IO1。
#define HQ_SET_IO1_RADAR			  0x7103		//雷达板预留IO1。
#define HQ_SET_UART1_SW				  0x7104		//UART1切换开关控制。
#define HQ_SET_LCD_RESET			  0x7105		//LCD屏复位控制，低电平有效。
#define HQ_SET_IO2_RADAR			  0x7106		//雷达板预留IO2。
#define HQ_SET_12V_RADAR			  0x7107		//雷达板12V电源开关控制，高电平打开，低电平关闭。
#define HQ_SET_12V_MAIN_CTL			  0x7108		//24V转12V电源控制，高电平打开。
#define HQ_SET_5V_RADAR				  0x7109		//雷达板5V电源开关控制，高电平打开，低电平关闭。
#define HQ_SET_RST_DM5885			  0x710a		//DM5885 RESET控制，低电平有效。
#define HQ_SET_12V_CAM_MONITOR		  0x710b		//盲区监控摄像头的12V电源控制，高电平打开，低电平关闭。
#define HQ_SET_USB2_POWER			  0x710c		//USB HOST接口（预留），5V电源开关控制脚，高电平有效。
//#define HQ_USB1_CTRL_ONOFF			  0x3002		//USB OTG接口（预留），5V电源开关控制脚，高电平有效。



//audio channel
#define HQ_SET_RADIO_CHANNEL          0x8000

#define HQ_GET_BACK_STATUS            100             ///vehicle STATUS
#define HQ_GET_BT_STATUS              101             //BT STATUS
#define HQ_GET_ACC_STATUS             102             //ACC STATUS
#define HQ_GET_LIGHT1_STATUS          103             //LIGHT1 STSTUS
#define HQ_GET_LIGHT2_STATUS          104             //LIGHT2 STATUS
#define HQ_GET_AMPMUTE_STATUS         105             //AMP MUTE STATUS
//#define HQ_GET_BACK_STATUS          106             //BACK TATUS
#define HQ_GET_FACE_KEY_STATUS        107             //FACE STATUS
#define HQ_GET_SHORTCUT_KEY_STATUS    108             //HOMEKEY STATUS
#define HQ_GET_USB_MODE_STATUS        109             //USB MODE STATUS
//#define HQ_GET_AD_VAL               110
#define HQ_GET_BACKLIGHT_ENABLE       111             //0 close 1 open
#define HQ_GET_FAN_STATUS             112             //GET FAN STATUS
#define HQ_GET_RADIO_STATUS           113             //GET RADIO STATUS

static int get_gpio_name(int pin)
{
    int nRet = -1;
    switch(pin)
    {
        case HQ_GET_BACK_STATUS://vehicle gpio
        #ifdef HQDEVICE_VEHICLE_GPIO
            nRet = HQDEVICE_VEHICLE_GPIO;
            //printk("\n back:%d\n",gpio_get_value(HQDEVICE_VEHICLE_GPIO));
        #endif
            break;
        case HQ_GET_BT_STATUS://bt gpio
        #ifdef HQDEVICE_BT_GPIO
            nRet = HQDEVICE_BT_GPIO;
		#endif
            break;
        case HQ_GET_ACC_STATUS://acc gpio
        #ifdef HQDEVICE_ACC_GPIO
            nRet = HQDEVICE_ACC_GPIO;
            //printk("====get HQDEVICE_ACC_GPIO====gpio_get_value(HQDEVICE_ACC_GPIO)= %d \n",gpio_get_value(HQDEVICE_ACC_GPIO));
		#endif
			break;
        case HQ_GET_LIGHT1_STATUS://left gpio
        #ifdef HQDEVICE_LEFT_GPIO
            nRet = HQDEVICE_LEFT_GPIO;
		#endif
            break;
        case HQ_GET_LIGHT2_STATUS://right gpio
        #ifdef HQDEVICE_RIGHT_GPIO
            nRet = HQDEVICE_RIGHT_GPIO;
		#endif
            break;
        case HQ_GET_BACKLIGHT_ENABLE://blackght
        #ifdef HQDEVICE_BL_EN_PIN_GPIO
            nRet = HQDEVICE_BL_EN_PIN_GPIO;
		#endif
            break;
        case HQ_GET_FAN_STATUS:
		#ifdef HQDEVICE_FAN_GPIO
            nRet = HQDEVICE_FAN_GPIO;
		#endif
            break;
        case  HQ_GET_USB_MODE_STATUS:
		#ifdef HQDEVICE_USB_MODE_SWITCH
            nRet = HQDEVICE_USB_MODE_SWITCH;
		#endif
            break;
        case HQ_GET_FACE_KEY_STATUS:
		#ifdef HQDEVICE_FACEKEY_GPIO
            nRet = HQDEVICE_FACEKEY_GPIO;
		#endif
            break;
       case HQ_GET_SHORTCUT_KEY_STATUS:
	   	#ifdef HQDEVIVE_HOMEKEY_GPIO
            nRet = HQDEVIVE_HOMEKEY_GPIO;
		#endif
            break;
       case HQ_GET_AMPMUTE_STATUS:
	   	#ifdef HQDEVICE_AUDIO_AMP_MUTE
           nRet = HQDEVICE_AUDIO_AMP_MUTE;
		#endif
		break;
    }
//    printk("hq status pin=%d\n",nRet);
    return nRet;
}

typedef int (*fn_cselect_t)(int channel);
typedef unsigned char (*fn_csignal_t)(void);

static fn_cselect_t hq_cselect;
static fn_csignal_t hq_csignal;

void register_cselect_fn(fn_cselect_t cselect)
{
	hq_cselect = cselect;
	return;
}

void register_csignal_fn(fn_csignal_t csignal)
{
	hq_csignal = csignal;
	return;
}

static int video_channel_select(int channel)
{
	if (hq_cselect) {
		return hq_cselect(channel);
	}

	return 0;
}

static unsigned int readSignalStatus(void)
{
	if (hq_csignal) {
		return hq_csignal();
	}

	return 0;
}

static long hope_unlocked_ioctl(struct file *file, unsigned int  cmd, unsigned long arg)
{
	int nRet = 0;
    int bt_onoff = -1,left_onoff = -1,right_onoff =-1 ,acc_onoff=-1,backlight_onoff=-1;
    int usb_switch = -1,camera_onoff =-1;
    int pin,get_gpio,fan_poweronof=-1,audio_flag = -1,fm_onoff = -1 ,video_channel = -1,audio_channel=-1;
    int usb1_onoff = -1,ampmute_onoff = -1;
	int onoff = -1;

//	printk("=====enter hope_unlocked_ioctl\n");
	switch(cmd)
	{
		case HQ_BT_CTRL_ONOFF:
			#ifdef HQDEVICE_BT_GPIO
    		if(copy_from_user(&bt_onoff, (int __user *)arg, sizeof(int)))
    		{
    					return -EFAULT;
    		}

    		if(bt_onoff)
    	    {
    		      gpio_direction_output(HQDEVICE_BT_GPIO, 1);//on
    		 	  gpio_set_value(HQDEVICE_BT_GPIO, 1);//on
    		 	  printk("bluetooth----bt_poweron\n\n");
    		 }
    		 else
    		 {
                  gpio_direction_output(HQDEVICE_BT_GPIO, 0);//off
    			  gpio_set_value(HQDEVICE_BT_GPIO, 0);//off
    			  printk("bluetooth----bt_poweroff\n\n");
    		}
			 #endif
    		break;
	    case HQ_BT_HOST_WAKE:
			#ifdef 	HQDEVICE_BT_REG_GPIO
			bt_num ++;
			if(bt_num > 2){
				gpio_direction_output(HQDEVICE_BT_REG_GPIO, (int)arg);
				gpio_set_value(HQDEVICE_BT_REG_GPIO, (int)arg);
			}
			#endif
		break;
	    case HQ_COOLING_FAN_CTRL_ONOFF:
			#ifdef HQDEVICE_FAN_GPIO
		    if(copy_from_user(&fan_poweronof, (int __user *)arg, sizeof(int)))
		    {
					return -EFAULT;
			}


			 if(fan_poweronof)
			 {
                  gpio_direction_output(HQDEVICE_FAN_GPIO, 1);
			 	  gpio_set_value(HQDEVICE_FAN_GPIO, 1);//on
			 	  printk("fan----fan_poweron\n\n");
			 }
			 else
			 {
			     gpio_direction_output(HQDEVICE_FAN_GPIO, 0);
			     gpio_set_value(HQDEVICE_FAN_GPIO, 0);//off
			 	 printk("fan----fan_poweroff\n\n");
			 }
			 #endif
			 break;

	    case HQ_USB1_CTRL_ONOFF:
			#ifdef HQDEVICE_USB_GPIO
		    if(copy_from_user(&usb1_onoff, (int __user *)arg, sizeof(int)))
		    {
					return -EFAULT;
			}


			 if(usb1_onoff)
			 {
                  gpio_direction_output(HQDEVICE_USB_GPIO, 1);
			 	  gpio_set_value(HQDEVICE_USB_GPIO, 1);//on
			 	  printk("usb----usb1_poweron\n\n");
			 }
			 else
			 {
			     gpio_direction_output(HQDEVICE_USB_GPIO, 0);
			     gpio_set_value(HQDEVICE_USB_GPIO, 0);//off
			 	 printk("usb----usb1_poweroff\n\n");
			 }
			 #endif
			 break;
        case HQ_AUDIO_MUTE_CTRL_ONOFF:
			#ifdef HQDEVICE_AUDIO_AMP_MUTE
		    if(copy_from_user(&ampmute_onoff, (int __user *)arg, sizeof(int)))
		    {
                return -EFAULT;
			}


			if(ampmute_onoff)
			{
                gpio_direction_output(HQDEVICE_AUDIO_AMP_MUTE, 1);
                gpio_set_value(HQDEVICE_AUDIO_AMP_MUTE, 1);//on
                printk("ampmute----ampmute_poweron\n\n");
			 }
			 else
			 {
                gpio_direction_output(HQDEVICE_AUDIO_AMP_MUTE, 0);
                gpio_set_value(HQDEVICE_AUDIO_AMP_MUTE, 0);//off
                printk("ampmute----ampmute_poweroff\n\n");
			 }
			#endif
            break;
        case HQ_BACKLIGH_ENABLE_ONOFF:
			#ifdef HQDEVICE_BL_EN_PIN_GPIO
            if(copy_from_user(&backlight_onoff, (int __user *)arg, sizeof(int)))
			{
					return -EFAULT;
			 }

			 if(backlight_onoff)
			 {
			    gpio_direction_output(HQDEVICE_BL_EN_PIN_GPIO, 1);
			    gpio_set_value(HQDEVICE_BL_EN_PIN_GPIO, 1);//on
			 	printk("backlight_poweron----backlight_poweron\n\n");
			 }
			 else
			 {
			     gpio_direction_output(HQDEVICE_BL_EN_PIN_GPIO, 0);
			     gpio_set_value(HQDEVICE_BL_EN_PIN_GPIO, 0);//off
			 	 printk("backlight_poweroff----backlight_poweroff\n\n");
			 }
			 #endif
			 break;

        case HQ_AUDOI_STANDY_CTRL_ONOFF:
			#ifdef HQDEVICE_AUDIO_STANDY_GPIO
            if(copy_from_user(&audio_flag, (int __user *)arg, sizeof(int)))
			{
					return -EFAULT;
			 }

			 if(audio_flag)
			 {
			      gpio_direction_output(HQDEVICE_AUDIO_STANDY_GPIO, 1);
			 	  gpio_set_value(HQDEVICE_AUDIO_STANDY_GPIO, 1);//on
			 	  printk("audio_flag----audio_poweron\n\n");
			 }
			 else
			 {
                 gpio_direction_output(HQDEVICE_AUDIO_STANDY_GPIO, 0);
                 gpio_set_value(HQDEVICE_AUDIO_STANDY_GPIO, 0);//off
			 	 printk("audio_flag----audio_poweroff\n\n");
			 }
			 #endif
			 break;

        case HQ_USB_MODE_SWITCH_ONOFF:
			#ifdef HQDEVICE_USB_MODE_SWITCH
            if(copy_from_user(&usb_switch, (int __user *)arg, sizeof(int)))
			{
				return -EFAULT;
			 }

			 if(usb_switch)
			 {
			     gpio_direction_output(HQDEVICE_USB_MODE_SWITCH, 1);
			 	 gpio_set_value(HQDEVICE_USB_MODE_SWITCH, 1);//device
			 	 printk("usb_switch----usb device mode\n\n");
			 }
			 else
			 {
			     gpio_direction_output(HQDEVICE_USB_MODE_SWITCH, 0);
			     gpio_set_value(HQDEVICE_USB_MODE_SWITCH, 0);//host
			 	 printk("usb_switch----usb host mode\n\n");
			 }
			 #endif
			 break;

        case HQ_CAMERA_CTRL_ONOFF:
			#ifdef HQDEVICE_CCD_CONTROL_GPIO
            if(copy_from_user(&camera_onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
             }


             if(camera_onoff)
             {
			     gpio_direction_output(HQDEVICE_CCD_CONTROL_GPIO, 1);
                 gpio_set_value(HQDEVICE_CCD_CONTROL_GPIO, 1);//device
                 printk("camera_onoff----camera_on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_CCD_CONTROL_GPIO, 0);
                 gpio_set_value(HQDEVICE_CCD_CONTROL_GPIO, 0);
                 printk("camera_onoff----camera_off\n\n");
             }
			 #endif
             break;

         case HQ_RADIO_CTRL_ONOFF:
		 	#ifdef HQDEVICE_FM_GPIO
            if(copy_from_user(&fm_onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }


             if(fm_onoff)
             {
			     gpio_direction_output(HQDEVICE_FM_GPIO, 1);
                 gpio_set_value(HQDEVICE_FM_GPIO, 1);//device
                 printk("fm_onoff----fm_on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_FM_GPIO, 0);
                 gpio_set_value(HQDEVICE_FM_GPIO, 0);
                 printk("fm_onoff----fm_off\n\n");
             }
			 #endif
             break;
         case HQ_SET_DERECTION_OUT:
             if(copy_from_user(&pin, (int __user *)arg, sizeof(int)))
    			return -EFAULT;

    		 get_gpio	= get_gpio_name(pin);
			 if(get_gpio < 0)
			 	return -EFAULT;
    		 gpio_free(get_gpio);
    		 if(gpio_request(get_gpio, NULL)){

                printk("request gpio%d output fail\n",get_gpio);

    		 }else{
                gpio_direction_output(get_gpio, 1);
                gpio_set_value(get_gpio, 1);
             }
             break;

		//6106AM
		case HQ_SET_IO1_RADAR:
		 	#ifdef HQDEVICE_IO1_RADAR
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_IO1_RADAR, 1);
                 gpio_set_value(HQDEVICE_IO1_RADAR, 1);//device
                 printk("IO1_RADAR----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_IO1_RADAR, 0);
                 gpio_set_value(HQDEVICE_IO1_RADAR, 0);
                 printk("IO1_RADAR----off\n\n");
             }
			 #endif
             break;

		case HQ_SET_UART1_SW:
		 	#ifdef HQDEVICE_UART1_SW
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_UART1_SW, 1);
                 gpio_set_value(HQDEVICE_UART1_SW, 1);//device
                 printk("UART1_SW----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_UART1_SW, 0);
                 gpio_set_value(HQDEVICE_UART1_SW, 0);
                 printk("UART1_SW----off\n\n");
             }
			 #endif
             break;

		case HQ_SET_LCD_RESET:
		 	#ifdef HQDEVICE_LCD_RESET
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_LCD_RESET, 1);
                 gpio_set_value(HQDEVICE_LCD_RESET, 1);//device
                 printk("LCD_RESET----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_LCD_RESET, 0);
                 gpio_set_value(HQDEVICE_LCD_RESET, 0);
                 printk("LCD_RESET----off\n\n");
             }
			 #endif
             break;

		case HQ_SET_IO2_RADAR:
		 	#ifdef HQDEVICE_IO2_RADAR
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_IO2_RADAR, 1);
                 gpio_set_value(HQDEVICE_IO2_RADAR, 1);//device
                 printk("IO2_RADAR----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_IO2_RADAR, 0);
                 gpio_set_value(HQDEVICE_IO2_RADAR, 0);
                 printk("IO2_RADAR----off\n\n");
             }
			 #endif
             break;

		case HQ_SET_12V_RADAR:
		 	#ifdef HQDEVICE_12V_RADAR
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_12V_RADAR, 1);
                 gpio_set_value(HQDEVICE_12V_RADAR, 1);//device
                 printk("12V_RADAR----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_12V_RADAR, 0);
                 gpio_set_value(HQDEVICE_12V_RADAR, 0);
                 printk("12V_RADAR----off\n\n");
             }
			 #endif
             break;

		case HQ_SET_12V_MAIN_CTL:
		 	#ifdef HQDEVICE_12V_MAIN_CTL
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_12V_MAIN_CTL, 1);
                 gpio_set_value(HQDEVICE_12V_MAIN_CTL, 1);//device
                 printk("12V_MAIN_CTL----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_12V_MAIN_CTL, 0);
                 gpio_set_value(HQDEVICE_12V_MAIN_CTL, 0);
                 printk("12V_MAIN_CTL----off\n\n");
             }
			 #endif
             break;

		case HQ_SET_5V_RADAR:
		 	#ifdef HQDEVICE_5V_RADAR
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_5V_RADAR, 1);
                 gpio_set_value(HQDEVICE_5V_RADAR, 1);//device
                 printk("5V_RADAR----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_5V_RADAR, 0);
                 gpio_set_value(HQDEVICE_5V_RADAR, 0);
                 printk("5V_RADAR----off\n\n");
             }
			 #endif
             break;		

		case HQ_SET_RST_DM5885:
		 	#ifdef HQDEVICE_RST_DM5885
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_RST_DM5885, 1);
                 gpio_set_value(HQDEVICE_RST_DM5885, 1);//device
                 printk("RST_DM5885----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_RST_DM5885, 0);
                 gpio_set_value(HQDEVICE_RST_DM5885, 0);
                 printk("RST_DM5885----off\n\n");
             }
			 #endif
             break;			 

		case HQ_SET_12V_CAM_MONITOR:
		 	#ifdef HQDEVICE_12V_CAM_MONITOR
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_12V_CAM_MONITOR, 1);
                 gpio_set_value(HQDEVICE_12V_CAM_MONITOR, 1);//device
                 printk("12V_CAM_MONITOR----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_12V_CAM_MONITOR, 0);
                 gpio_set_value(HQDEVICE_12V_CAM_MONITOR, 0);
                 printk("12V_CAM_MONITOR----off\n\n");
             }
			 #endif
             break;		
			 
		case HQ_SET_USB2_POWER:
		 	#ifdef HQDEVICE_USB2_POWER
            if(copy_from_user(&onoff, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

             if(onoff)
             {
			     gpio_direction_output(HQDEVICE_USB2_POWER, 1);
                 gpio_set_value(HQDEVICE_USB2_POWER, 1);//device
                 printk("USB2_POWER----on\n\n");
             }
             else
             {
			     gpio_direction_output(HQDEVICE_USB2_POWER, 0);
                 gpio_set_value(HQDEVICE_USB2_POWER, 0);
                 printk("USB2_POWER----off\n\n");
             }
			 #endif
             break;

          case HQ_SET_DERECTION_INPUT:
            if(copy_from_user(&pin, (int __user *)arg, sizeof(int)))
				return -EFAULT;
		    get_gpio	= get_gpio_name(pin);
			if(get_gpio < 0)
			 	return -EFAULT;
		    if(gpio_request(get_gpio, NULL)){
                printk("request gpio:%d input fail\n",get_gpio);
		    }else{
				gpio_direction_input(get_gpio);
            }
		    break;

          case HQ_VEHICLE_GET_STATUS:
#ifdef CONFIG_SOC_CAMERA_DM5886
			*((unsigned int*)arg) = dm5886_get_outsel();
#else
			*((unsigned int*)arg) = cameraFlag;
#endif
            break;
          case HQ_GET_AUDIO_CHANNEL:
			*((unsigned int*)arg) = audioFlag;
            break;
          case HQ_SET_RADIO_CHANNEL:
            if(copy_from_user(&audio_channel, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

            if(audio_channel<0||audio_channel>7){
                pr_info("invalid audio channel %d\n", audio_channel);
            }else{
#if defined(CONFIG_BOARD_HQG710WS)
                extern void audio_switch(int channel);
                audioFlag = audio_channel;
                audio_switch(audio_channel);
#else
                hqdevice_tda7719_channel(audio_channel);//0 radio 1 arm 2
#endif
            }

            break;
         case HQ_SET_CAMERA_CHANNEL://0 close all cvbs1 cvbs2 cvbs3 cvbs4 ccd5 reserver6 channel
            if(copy_from_user(&video_channel, (int __user *)arg, sizeof(int)))
            {
                 return -EFAULT;
            }

            if(video_channel<0){
                printk("video_channel error ----video_channel ==0x%x\n\n",video_channel);

            }else{
#ifdef CONFIG_SOC_CAMERA_DM5886
                dm5886_set_outsel(video_channel);
#else
#ifdef CONFIG_HQDEVICE_CM
                hq_channel_video_select(video_channel);
#else
				cameraFlag = video_channel;
				video_channel_select(video_channel);
#endif
#endif
            }
            break;
         case HQ_SET_AUDIOEQ_VALE:
            printk("===enter HQ_SET_AUDIOEQ_VALE \n");
            unsigned char pEqValue[4];
            if(copy_from_user(pEqValue, (void *)arg, sizeof(int)))
            {
                return -EFAULT;
            }
            hqdevice_set_eq(pEqValue);
            break;
         case HQ_SET_LRF_CHANNEL_VOL:
            printk("===enter HQ_SET_LRF_CHANNEL_VOL \n");
            unsigned char pLrfValue[4];
            if(copy_from_user(pLrfValue, (void *)arg, sizeof(int)))
            {
                return -EFAULT;
            }
            hqdevice_set_lrf_volume(pLrfValue);
            break;
         case HQ_SET_AUDIO_VOLUME:
            printk("===enter HQ_SET_AUDIO_VOLUME \n");
            unsigned char pAudioValue[4];
            if(copy_from_user(pAudioValue, (void *)arg, sizeof(int)))
            {
                return -EFAULT;
            }
            hqdevice_set_volume(pAudioValue);
            break;
         case HQ_SET_AUDIO_MIX_CHANNEL:
            printk("===enter HQ_SET_AUDIO_MIX_CHANNEL \n");
            unsigned char pMixValue[4];
            if(copy_from_user(pMixValue, (void *)arg, sizeof(int)))
            {
                return -EFAULT;
            }
            hqdevice_set_mix(pMixValue);
            break;
		case HQ_GET_UART1_RX_TEST_DET:
			*((unsigned int*)arg) = uart1_rx_det;
			break;
         case HQ_GET_HQDEVIEIO_STATUS:
			 if(copy_from_user(&pin, (int __user *)arg, sizeof(int)))
					return -EFAULT;
#ifdef defined(CONFIG_BOARD_HQG710PA)
			extern int xcu_get_io(int which);
			switch (pin) {
			case HQ_GET_FACE_KEY_STATUS:
				*((unsigned int*)arg) = xcu_get_io(pin);
				return nRet;
			default:
				break;
			}
#endif
		    get_gpio	= get_gpio_name(pin);    
			if(get_gpio < 0)
			 	return -EFAULT;
#if defined(CONFIG_BOARD_HQG710WS)
			switch (pin) {
			case HQ_GET_FACE_KEY_STATUS:
			case HQ_GET_SHORTCUT_KEY_STATUS:
				*((unsigned int*)arg) = 1;
				return nRet;
			default:
				break;
			}
#endif
			*((unsigned int*)arg) = gpio_get_value(get_gpio);
            //printk("pin:%d value:%d\n",pin,*((unsigned int*)arg));
            break;
         case HQ_GET_ADC_VAL:
            *((unsigned int*)arg) = adc_value;
            break;

         case HQ_GET_CAMERA_SIGNAL:
           *((unsigned int*)arg) = readSignalStatus();
            //printk("Signal:%d\n",*((unsigned int*)arg));
            break;
	}
	return nRet;
}
static int hope_open(struct inode * inode,struct file * file)
{
	int nRet = 0;

	return 0;
}
static int  hope_release(struct inode * inode, struct file * file)
{
	int nRet = 0;

	return 0;
}

int hq_det_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&gpio_det_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hq_det_notifier_call_chain);

int hq_det_register_notifier(struct notifier_block *nb)
{
	int ret = blocking_notifier_chain_register(&gpio_det_notifier_list, nb);

	return ret;
}
EXPORT_SYMBOL(hq_det_register_notifier);

int hq_det_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&gpio_det_notifier_list, nb);
}
EXPORT_SYMBOL(hq_det_unregister_notifier);

static void gpio_det_report_event(struct gpio_data *gpiod)
{
	struct gpio_event event;
	struct gpio_detection *gpio_det = gpiod->parent;
	char *status = NULL;
	char *envp[2];

	event.val = gpiod->val ^ gpiod->atv_val;
	event.name = gpiod->name;
	status = kasprintf(GFP_KERNEL, "GPIO_NAME=%s GPIO_STATE=%s",
			   gpiod->name, event.val ? "over" : "on");
	envp[0] = status;
	envp[1] = NULL;
	wake_lock_timeout(&gpio_det->wake_lock, 5 * HZ);
	kobject_uevent_env(&gpiod->dev.kobj, KOBJ_CHANGE, envp);
	if (gpiod->notify)
		hq_det_notifier_call_chain(GPIO_EVENT, &event);
	if (event.val == 0) //on
		irq_set_irq_type(gpiod->irq, IRQF_TRIGGER_RISING);
	else
		irq_set_irq_type(gpiod->irq, IRQF_TRIGGER_FALLING);

}

static irqreturn_t gpio_det_interrupt(int irq, void *dev_id)
{
	struct gpio_data *gpiod = dev_id;
	int val = gpio_get_value(gpiod->gpio);

#if defined(HQDEVICE_UART1_RX_TEST_DET)
	if (0 == uart1_rx_det) {
		if (gpiod->name && 0==strcmp(gpiod->name, "uart1-rx-det")) {
			uart1_rx_det = 1;
			pr_info("UART1 RX detected\n");
		}
	}
#endif

	if (gpiod->val != val) {
		gpiod->val = val;
		gpio_det_report_event(gpiod);
	}

	return IRQ_HANDLED;
}

static int gpio_det_init_status_check(struct gpio_detection *gpio_det)
{
	struct gpio_data *gpiod;
	int i;

	for (i = 0; i < gpio_det->num; i++) {
		gpiod = &gpio_det->data[i];
		gpiod->val = gpio_get_value(gpiod->gpio);
		if (gpiod->atv_val == gpiod->val)
			gpio_det_report_event(gpiod);
	}

	return 0;
}

static int gpio_det_fb_notifier_callback(struct notifier_block *self,
					 unsigned long event,
					 void *data)
{
	struct gpio_detection *gpio_det;
	struct fb_event *evdata = data;
	int fb_blank;

	if (event != FB_EVENT_BLANK && event != FB_EVENT_CONBLANK)
		return 0;

	gpio_det = container_of(self, struct gpio_detection, fb_notifier);
	fb_blank = *(int *)evdata->data;
	if (fb_blank == FB_BLANK_UNBLANK)
		system_suspend = 0;
	else
		system_suspend = 1;

	return 0;
}

static int gpio_det_fb_notifier_register(struct gpio_detection *gpio)
{
	gpio->fb_notifier.notifier_call = gpio_det_fb_notifier_callback;

	return fb_register_client(&gpio->fb_notifier);
}

static ssize_t gpio_detection_info_show(struct class *class, struct class_attribute *attr,
				char *buf)
{
	struct gpio_detection *gpio_det = container_of(attr, struct gpio_detection, cls_attr);;

	return sprintf(buf, "%d\n", gpio_det->info);
}

static int status_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct gpio_data *gpiod = container_of(dev, struct gpio_data, dev);
	unsigned int val = gpio_get_value(gpiod->gpio);

	return sprintf(buf, "%d\n", val == gpiod->atv_val);
}

static ssize_t status_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct gpio_data *gpiod;
	int val;
	int ret;
	struct gpio_event event;

	gpiod = container_of(dev, struct gpio_data, dev);
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;
	if (val >= 0) {
		event.val = val;
		event.name = gpiod->name;
		hq_det_notifier_call_chain(GPIO_EVENT, &event);
	} else {
		gpiod->notify = 0;
	}

	return count;
}

static int __init gpio_deteciton_class_init(void)
{
	gpio_detection_class = class_create(THIS_MODULE, "hq_io");
	if (IS_ERR(gpio_detection_class)) {
		pr_err("create gpio_detection class failed (%ld)\n",
		       PTR_ERR(gpio_detection_class));
		return PTR_ERR(gpio_detection_class);
	}

	return 0;
}

static int gpio_detection_class_register(struct gpio_detection *gpio_det,
					 struct gpio_data *gpiod)
{
	int ret;

	gpiod->dev.class = gpio_detection_class;
	dev_set_name(&gpiod->dev, "%s", gpiod->name);
	dev_set_drvdata(&gpiod->dev, gpio_det);
	ret = device_register(&gpiod->dev);

	return ret;
}

static int gpio_det_get_platform_data(struct gpio_detection *gpio_det, struct px3_gpio_platform_data *pdata)
{
	struct gpio_data *data;
	struct gpio_data *gpiod;

	data = devm_kzalloc(gpio_det->dev, 2 * sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	if (pdata->camcap_type)
		gpio_det->type = pdata->camcap_type;
	else
		gpio_det->type = 0;
	if (pdata->camcap_mirror)
		gpio_det->mirror = pdata->camcap_mirror;
	else
		gpio_det->mirror = 0;
	gpio_det->info = (gpio_det->mirror << 4) | gpio_det->type;

	if (pdata->hq_left.gpio) {
		gpiod = &data[0];
		gpiod->parent = gpio_det;
		gpiod->notify = 1;
		gpiod->gpio = pdata->hq_left.gpio;
		gpiod->atv_val = pdata->hq_left.active;
		gpiod->name = pdata->hq_left.label;
		gpiod->wakeup = pdata->hq_left.wakeup;
		gpiod->irq = gpio_to_irq(gpiod->gpio);
	}

	if (pdata->hq_right.gpio) {
		gpiod = &data[1];
		gpiod->parent = gpio_det;
		gpiod->notify = 1;
		gpiod->gpio  = pdata->hq_right.gpio;
		gpiod->atv_val  = pdata->hq_right.active;
		gpiod->name    = pdata->hq_right.label;
		gpiod->wakeup  = pdata->hq_right.wakeup;
		gpiod->irq     = gpio_to_irq(gpiod->gpio);
	}

	gpio_det->num = 2;
	gpio_det->data = data;

	return 0;
}

void dvr_set_powerOnOff(unsigned int isOn)
{
#ifdef HQDEVICE_DVR_CONTROL_GPIO
    gpio_direction_output(HQDEVICE_DVR_CONTROL_GPIO, isOn);
    gpio_set_value(HQDEVICE_DVR_CONTROL_GPIO, isOn);
#endif
}

void ccd_set_powerOnOff(unsigned int isOn)
{
#ifdef HQDEVICE_CCD_CONTROL_GPIO
    gpio_direction_output(HQDEVICE_CCD_CONTROL_GPIO, isOn);
    gpio_set_value(HQDEVICE_CCD_CONTROL_GPIO, isOn);
#endif
}

static int init_hq_deviceIo(void)
{
	int nRet = 0;

#ifdef HQDEVICE_BT_GPIO
	nRet  = gpio_request(HQDEVICE_BT_GPIO, "bt power");
//	printk("bt power\n");
	if (nRet != 0)
	{
		printk("bt power fail \n");
		gpio_free(HQDEVICE_BT_GPIO);
	}
	gpio_direction_output(HQDEVICE_BT_GPIO, GPIO_HIGH);
#endif

#ifdef HQDEVICE_BT_REG_GPIO
	//HQDEVICE_BT_RTG_GPIO is Provided to the manufacturer to use
	gpio_free(HQDEVICE_BT_REG_GPIO);
	nRet  = gpio_request(HQDEVICE_BT_REG_GPIO, "bt_reg_on");
	if (nRet != 0)
	{
	        printk("bt reg fail \n");
	        gpio_free(HQDEVICE_BT_REG_GPIO);
	}
	gpio_direction_output(HQDEVICE_BT_REG_GPIO, GPIO_HIGH);
	//gpio_set_value(HQDEVICE_BT_REG_GPIO, GPIO_HIGH);
#endif
#ifdef HQDEVICE_DVR_CONTROL_GPIO
    nRet  = gpio_request(HQDEVICE_DVR_CONTROL_GPIO, "Dvr power");
    if (nRet != 0)
    {
        printk("CAMERA_POWER_ENABLE fail \n");
        gpio_free(HQDEVICE_DVR_CONTROL_GPIO);
    }

    gpio_direction_output(HQDEVICE_DVR_CONTROL_GPIO, GPIO_HIGH);
#endif

#ifdef HQDEVICE_CCD_CONTROL_GPIO
    nRet  = gpio_request(HQDEVICE_CCD_CONTROL_GPIO, "Ccd power");
    if (nRet != 0)
    {
        printk("Dvr_POWER_ENABLE fail \n");
        gpio_free(HQDEVICE_CCD_CONTROL_GPIO);
    }

    gpio_direction_output(HQDEVICE_CCD_CONTROL_GPIO, GPIO_HIGH);
#endif

#ifdef HQDEVICE_12V_CONTOL_GPIO
    nRet  = gpio_request(HQDEVICE_12V_CONTOL_GPIO, "V12 power");
    if (nRet != 0)
    {
        printk("V12_POWER_ENABLE fail \n");
        gpio_free(HQDEVICE_12V_CONTOL_GPIO);
    }

    gpio_direction_output(HQDEVICE_12V_CONTOL_GPIO, GPIO_HIGH);
#endif

#ifdef HQDEVICE_AUDIO_STANDY_GPIO
	nRet  = gpio_request(HQDEVICE_AUDIO_STANDY_GPIO, "AMP");

	if (nRet != 0)
	{
		printk("AUDIO_AMP_STANDY fail \n");
		gpio_free(HQDEVICE_AUDIO_STANDY_GPIO);
	}
	gpio_direction_output(HQDEVICE_AUDIO_STANDY_GPIO, GPIO_HIGH);
#endif

#ifdef HQDEVICE_AUDIO_AMP_MUTE
	nRet  = gpio_request(HQDEVICE_AUDIO_AMP_MUTE, "AMP MUTE");

	if (nRet != 0)
	{
		printk("AUDIO_AMP_MUTE fail \n");
		gpio_free(HQDEVICE_AUDIO_AMP_MUTE);
	}
	gpio_direction_output(HQDEVICE_AUDIO_AMP_MUTE, GPIO_HIGH);
#endif

#ifdef HQDEVICE_FM_GPIO
	nRet = gpio_request(HQDEVICE_FM_GPIO, "fm");
	if (nRet) {
		pr_err("request GPIO %s for reset failed, ret = %d\n",	CSTR(HQDEVICE_FM_GPIO), nRet);
	}
	gpio_direction_output(HQDEVICE_FM_GPIO, 1);
#endif

#ifdef HQDEVICE_USB_MODE_SWITCH
    nRet = gpio_request(HQDEVICE_USB_MODE_SWITCH, "usb");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n", CSTR(HQDEVICE_USB_MODE_SWITCH), nRet);
    }
    gpio_direction_output(HQDEVICE_USB_MODE_SWITCH, 1);
#endif 

#ifdef HQDEVICE_USB_GPIO
    nRet = gpio_request(HQDEVICE_USB_GPIO, "usb power");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_USB_GPIO), nRet);
    }
    gpio_direction_output(HQDEVICE_USB_GPIO, 1);
#endif 

#ifdef HQDEVICE_FAN_GPIO
    nRet = gpio_request(HQDEVICE_FAN_GPIO, "fan power");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_FAN_GPIO), nRet);
    }
    gpio_direction_output(HQDEVICE_FAN_GPIO, 1);
#endif

//6016AM
#ifdef HQDEVICE_GPS_EN
	nRet = gpio_request(HQDEVICE_GPS_EN, "gps en");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_GPS_EN), nRet);
    }
    gpio_direction_output(HQDEVICE_GPS_EN, 1);
#endif

#ifdef HQDEVICE_IO1_RADAR
	nRet = gpio_request(HQDEVICE_IO1_RADAR, "IO1_RADAR");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_IO1_RADAR), nRet);
    }
    gpio_direction_output(HQDEVICE_IO1_RADAR, 0);
#endif

#ifdef HQDEVICE_UART1_SW
	gpio_free(HQDEVICE_UART1_SW);
	nRet = gpio_request(HQDEVICE_UART1_SW, "uart1_sw");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_UART1_SW), nRet);
    }
    gpio_direction_output(HQDEVICE_UART1_SW, 0);
#endif

#ifdef HQDEVICE_UART1_RX_TEST_DET
	nRet = gpio_request(HQDEVICE_UART1_RX_TEST_DET, "uart1_rx_test_det");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_UART1_RX_TEST_DET), nRet);
    }
    gpio_direction_input(HQDEVICE_UART1_RX_TEST_DET);
#endif

#ifdef HQDEVICE_LCD_RESET
	nRet = gpio_request(HQDEVICE_LCD_RESET, "lcd reset");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_LCD_RESET), nRet);
    }
    gpio_direction_output(HQDEVICE_LCD_RESET, 1);
#endif

#ifdef HQDEVICE_IO2_RADAR
	nRet = gpio_request(HQDEVICE_IO2_RADAR, "IO2_RADAR");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_IO2_RADAR), nRet);
    }
    gpio_direction_output(HQDEVICE_IO2_RADAR, 0);
#endif

#ifdef HQDEVICE_12V_RADAR
	nRet = gpio_request(HQDEVICE_12V_RADAR, "12v radar");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_12V_RADAR), nRet);
    }
    gpio_direction_output(HQDEVICE_12V_RADAR, 1);
#endif


#ifdef HQDEVICE_12V_MAIN_CTL
	gpio_free(HQDEVICE_12V_MAIN_CTL);
	nRet = gpio_request(HQDEVICE_12V_MAIN_CTL, "12v main ctrl");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n", CSTR(HQDEVICE_12V_MAIN_CTL), nRet);
    }
    gpio_direction_output(HQDEVICE_12V_MAIN_CTL, 1);
#endif

#ifdef HQDEVICE_5V_RADAR
	gpio_free(HQDEVICE_5V_RADAR);
	nRet = gpio_request(HQDEVICE_5V_RADAR, "5v radar");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_5V_RADAR), nRet);
    }
    gpio_direction_output(HQDEVICE_5V_RADAR, 1);
#endif


#ifdef HQDEVICE_RST_DM5885
	nRet = gpio_request(HQDEVICE_RST_DM5885, "rst dm5885");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_RST_DM5885), nRet);
    }
    gpio_direction_output(HQDEVICE_RST_DM5885, 1);
#endif

#ifdef HQDEVICE_12V_CAM_MONITOR
	nRet = gpio_request(HQDEVICE_12V_CAM_MONITOR, "12v cam monitor");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_12V_CAM_MONITOR), nRet);
    }
    gpio_direction_output(HQDEVICE_12V_CAM_MONITOR, 1);
#endif

#ifdef HQDEVICE_USB2_POWER
	nRet = gpio_request(HQDEVICE_USB2_POWER, "usb2 power");
    if (nRet) {
        pr_err("request GPIO %s for reset failed, ret = %d\n",CSTR(HQDEVICE_USB2_POWER), nRet);
    }
    gpio_direction_output(HQDEVICE_USB2_POWER, 1);
#endif

    return 1;
}

static int gpio_det_probe(struct platform_device *pdev)
{
	struct gpio_detection *gpio_det;
	struct gpio_data *gpiod;
	unsigned long irqflags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
	int i;
	int ret;
	struct px3_gpio_platform_data *pdata = pdev->dev.platform_data;

	gpio_det = devm_kzalloc(&pdev->dev, sizeof(*gpio_det), GFP_KERNEL);
	if (!gpio_det)
		return -ENOMEM;
	gpio_det->dev = &pdev->dev;
	gpio_det->cls_attr.attr.name = "info";
	gpio_det->cls_attr.attr.mode = S_IRUGO;
	gpio_det->cls_attr.show = gpio_detection_info_show;


	dev_set_name(gpio_det->dev, "hq_io");
	gpio_det_get_platform_data(gpio_det, pdata);
	wake_lock_init(&gpio_det->wake_lock, WAKE_LOCK_SUSPEND, "hq_io");
	for (i = 0; i < gpio_det->num; i++) {
		gpiod = &gpio_det->data[i];

		ret = gpio_request(gpiod->gpio, gpiod->name);
		if (ret < 0)
			printk(KERN_ERR "gpio_det_probe request %s failed\n", gpiod->name);

		gpio_pull_updown(gpiod->gpio, GPIOPullUp);
        gpio_direction_input(gpiod->gpio);//2017-10-31
//		gpio_direction_output(gpiod->gpio,1);
		gpiod->val = gpio_get_value(gpiod->gpio);

        ret = gpio_detection_class_register(gpio_det, gpiod);
		if (ret < 0)
			return ret;

		if (gpiod->irq <=0)
			continue;
		if (gpiod->wakeup)
			irqflags |= IRQF_NO_SUSPEND;

		ret = devm_request_threaded_irq(gpio_det->dev, gpiod->irq,	NULL, gpio_det_interrupt,irqflags, gpiod->name, gpiod);
		if (ret < 0)
			dev_err(gpio_det->dev, "request irq(%s) failed\n",gpiod->name);
	}

	if (gpio_det->info)
		ret = class_create_file(gpio_detection_class, &gpio_det->cls_attr);

	gpio_det_fb_notifier_register(gpio_det);
	gpio_det_init_status_check(gpio_det);

	init_hq_deviceIo();

	dev_info(gpio_det->dev, "gpio detection driver probe success\n");

	return 0;
}

static int gpio_det_remove(struct platform_device *dev)
{
	return 0;
}

static void gpio_det_shutdown(struct platform_device *dev)
{

}

static  struct file_operations hope_fops={
	.owner        	 = THIS_MODULE,
	.open    		 = hope_open,
	.read    		 = hope_read,
	.write    		 = hope_write,
	.unlocked_ioctl  = hope_unlocked_ioctl,
	.release     	 = hope_release,
};

static struct miscdevice hope_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "hq_io",
	.fops  = &hope_fops,
};
static struct platform_driver gpio_det_driver = {
	.driver     = {
		.name   = "hq_io",
		.owner  = THIS_MODULE,
	},
	.probe      = gpio_det_probe,
	.remove     = gpio_det_remove,
	.shutdown   = gpio_det_shutdown,
};


static int __init hope_init(void)
{
	int nRet = 0;

	nRet = misc_register(&hope_miscdev);
	if (nRet < 0)
		return nRet;
	nRet = 	gpio_deteciton_class_init();
	if (nRet){
		nRet = -1;
		return nRet;
	}

	platform_driver_register(&gpio_det_driver);
    hq_debug_sysfs(&hope_miscdev);

	return nRet;
}

static void __exit hope_exit(void)
{
	misc_deregister(&hope_miscdev);
	platform_driver_unregister(&gpio_det_driver);

}

module_init(hope_init);
module_exit(hope_exit);

MODULE_AUTHOR("hopechart");
MODULE_DESCRIPTION("hope Gpio set");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:hope-gpio");
