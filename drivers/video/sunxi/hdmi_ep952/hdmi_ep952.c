#include <linux/module.h>
#include <asm/uaccess.h>
#include <asm/memory.h>
#include <asm/unistd.h>
#include "asm-generic/int-ll64.h"
#include "linux/kernel.h"
#include "linux/mm.h"
#include "linux/semaphore.h"
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/switch.h> // switch_dev
#include <mach/sys_config.h>
#include <mach/platform.h>
#include <video/drv_display.h>
#include "EP952api.h"
#include "../disp/OSAL/OSAL.h"
#include "Edid.h"

#define HDMI720P_50         19
#define HDMI720P_60         4
#define HDMI1080P_50        31
#define HDMI1080P_60        16
#define HDMI480P            3
#define HDMI576P            17
#define EP952_DETECT_HPD
static char key_name[20] = "hdmi_ep952_para";
static u32 hdmi_used = 0;

int hpd_delay_enable = 0;
int hpd_delay = 0;
int g_ep952_enabled = 0;
static disp_tv_mode g_hdmi_mode = DISP_TV_MOD_720P_60HZ;
static unsigned char g_hdmi_vic = HDMI720P_60;
static unsigned char g_hpd_state = 0;
static unsigned char sunxi_edid[256] = {0};

static u32 hdmi_i2c_id = 0;
static u32 hdmi_i2c_used = 0;
static u32 ddc_i2c_id = 0;
static u32 ddc_i2c_used = 0;
static u32 hdmi_power_used = 0;
static char hdmi_power[16] = {0};

static bool hdmi_gpio_used[4];
static disp_gpio_set_t hdmi_gpio[4];
static struct i2c_adapter *ep952_i2c_adapter = NULL;
static struct i2c_adapter *ddc_i2c_adapter = NULL;
static struct i2c_client *ep952_i2c_client;

static struct task_struct *ep952_task = NULL;

static struct cdev *ep952_cdev;
static dev_t devid ;
static struct class *ep952_class;

#if defined(CONFIG_SWITCH) || defined(CONFIG_ANDROID_SWITCH)
static struct switch_dev hdmi_switch_dev = {
    .name = "hdmi",
};
#endif

s32 ddc_i2c_write(u32 client_addr, u8 * data,int size);
s32 ddc_i2c_read(u32 client_addr,u8 sub_addr,u8 * data,int size);
s32 ddc_i2c_send(struct i2c_msg *msgs, int msg_count);

s32 ep952_i2c_write(u32 client_addr, u8 * data,int size);
s32 ep952_i2c_read(u32 client_addr,u8 sub_addr,u8 * data,int size);
s32 ep952_i2c_send(struct i2c_msg *msgs, int msg_count);
extern struct i2c_adapter *i2c_get_adapter(int nr);

extern void EP952Controller_Timer(void);
extern void EP952Controller_Task(void);
extern unsigned char HDMI_Tx_hpd_state(void);
extern SMBUS_STATUS EP952_Reg_Read(unsigned char ByteAddr, unsigned char *Data, unsigned int Size);
extern int OSAL_Power_Enable(char *name);
extern int OSAL_Power_Disable(char *name);
extern __hdle OSAL_GPIO_Request(disp_gpio_set_t *gpio_list, u32 group_count_max);
extern s32 OSAL_GPIO_Release(__hdle p_handler, s32 if_release_to_default_status);

static disp_video_timing video_timing[] =
{
    /*vic           PCLK       AVI x      y      HT     HBP   HFP   HST  VT     VBP  VFP VST H_P  V_P  I    TRD */
    {HDMI720P_50,   74250000,  0,  1280,  720,   1980,  220,  440,  40,  750,   20,  5,  5,  1,   1,   0,   0,   0},
    {HDMI720P_60,   74250000,  0,  1280,  720,   1650,  220,  110,  40,  750,   20,  5,  5,  1,   1,   0,   0,   0},
    //{HDMI1080P_50,  148500000, 0,  1920,  1080, 2640,  148,  528,  44,  1125,  36,  4,  5,  1,   1,   0,   0,   0},
    //{HDMI1080P_60,  148500000, 0,  1920,  1080, 2200,  148,  88,   44,  1125,  36,  4,  5,  1,   1,   0,   0,   0},
    {HDMI480P,      27000000,  0,  720,  480,   858,   60,   16,   62,  525,   30,  9,  6,  0,   0,   0,   0,   0},
    {HDMI576P,      27000000,  0,  720,  576,   864,   68,   12,   64,  625,   39,  5,  5,  0,   0,   0,   0,   0},
};

static int hdmi_parse_config(void)
{
    printk("%s:%d\n", __func__, __LINE__);
    disp_gpio_set_t  *gpio_info;
    int i, ret;
    char io_name[32];
    script_item_u   val;
    for(i=0; i<2; i++) {
        gpio_info = &(hdmi_gpio[i]);
        sprintf(io_name, "gpio_%d", i);

        printk("%s:%d\n", __func__, __LINE__);
        ret = script_get_item(key_name, io_name, &val);
        if (ret == 3) {
            gpio_info->gpio = val.gpio.gpio;
            gpio_info->mul_sel = val.gpio.mul_sel;
            gpio_info->pull = val.gpio.pull;
            gpio_info->drv_level = val.gpio.drv_level;
            gpio_info->data = val.gpio.data;
            memcpy(gpio_info->gpio_name, io_name, strlen(io_name)+1);
            printk("gpio get success!\n");
            hdmi_gpio_used[i]= 1;
        }
        else
            continue;
    }

    printk("%s:%d\n", __func__, __LINE__);
    return 0;
}

// bon: 0-disable the gpio, 1-out as same as configed.
int hdmi_gpio_config(int gpio_id, int bon)
{
    int hdl;
    printk("%s:%d\n", __func__, __LINE__);

    if(gpio_id <= sizeof(hdmi_gpio_used) && hdmi_gpio_used[gpio_id]) {
        disp_gpio_set_t  gpio_info[1];

        memcpy(gpio_info, &(hdmi_gpio[gpio_id]), sizeof(disp_gpio_set_t));
        if(!bon) {
            gpio_info->mul_sel = 7;
        }
    printk("%s: gpio_info->mul_sel = %d\n", __func__, gpio_info->mul_sel);
        hdl = OSAL_GPIO_Request(gpio_info, 1);
        OSAL_GPIO_Release(hdl, 2);
    }
    return 0;
}

static int hdmi_power_enable(char *name)
{
    return OSAL_Power_Enable(name);
}

static int hdmi_power_disable(char *name)
{
    return OSAL_Power_Disable(name);
}


s32 ep952_hdmi_power_on(u32 on_off)
{
    if(hdmi_power_used == 0)
        return 0;

    if(on_off)
        hdmi_power_enable(hdmi_power);
    else;
        hdmi_power_disable(hdmi_power);

    return 0;
}

static unsigned char mode2vic(disp_tv_mode hdmi_mode)
{
    switch(hdmi_mode) {
    case DISP_TV_MOD_480P:
        return HDMI480P;
    case DISP_TV_MOD_576P:
        return HDMI576P;
    case DISP_TV_MOD_720P_50HZ:
        return HDMI720P_50;
    case DISP_TV_MOD_720P_60HZ:
        return HDMI720P_60;
    case DISP_TV_MOD_1080P_50HZ:
        return HDMI1080P_50;
    case DISP_TV_MOD_1080P_60HZ:
        return HDMI1080P_60;
    default:
        printk("fixme:%s:%d!!!", __func__, __LINE__);
        return HDMI720P_50;
    }
}

static void ep952_update_hpd_status(void)
{
    static unsigned char s_hpd_change_count = 0;
#if defined(EP952_DETECT_HPD)
    unsigned char current_state = HDMI_Tx_hpd_state() ? 1 : 0;
#else
    unsigned char current_state = 1;
#endif
    if(g_hpd_state != current_state) {
        if(s_hpd_change_count++ >= 10) {
            s_hpd_change_count = 0;
            g_hpd_state = current_state;
#if defined(CONFIG_SWITCH) || defined(CONFIG_ANDROID_SWITCH)
            switch_set_state(&hdmi_switch_dev, current_state);
            // only delay hpd while issue hpd kernel event
            if (hpd_delay_enable == 1) {
                hpd_delay = current_state;
            }
#endif
        }
    } else {
        s_hpd_change_count = 0;
    }
}

s32 ep952_get_hpd_status(void)
{
    return g_hpd_state;
}
EXPORT_SYMBOL(ep952_get_hpd_status);

int ep952_thread(void *parg)
{

    unsigned int timeout = HZ / 50; // 20ms
    printk("%s:%d\n", __func__, __LINE__);
    while(!kthread_should_stop()) {
        ep952_update_hpd_status();
        EP952Controller_Timer();
        EP952Controller_Task();
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(timeout);
    }
    return 0;
}

int ep952_thread_enable(void)
{
    int err;

    if(ep952_task)
        return 0;

    ep952_task = kthread_create(ep952_thread, NULL, "ep952_task");
    if(IS_ERR(ep952_task)){
        printk("Unable to start kernel thread./n");
        err = PTR_ERR(ep952_task);
        ep952_task = NULL;
        return err;
    } else {
        wake_up_process(ep952_task);
    }
    return 0;
}

int ep952_thread_disable(void)
{
    if(ep952_task) {
        kthread_stop(ep952_task);
        ep952_task = NULL;
    }
    return 0;
}

s32 ep952_get_mode()
{
    return g_hdmi_mode;
}
EXPORT_SYMBOL(ep952_get_mode);

s32 ep952_check_mode_support(disp_tv_mode hdmi_mode)
{
    unsigned char vic;
    s32 res;

    switch(hdmi_mode) {
    case DISP_TV_MOD_480P:
        vic = mode2vic(hdmi_mode);
        res = EDID_get_vic_support(vic);
        // 480p is assumed to be always supported, even in the
        // absence of EDID
        return res == 2 ? 2 : 1;
    case DISP_TV_MOD_576P:
    case DISP_TV_MOD_720P_50HZ:
    case DISP_TV_MOD_720P_60HZ:
    //case DISP_TV_MOD_1080P_50HZ:
    //case DISP_TV_MOD_1080P_60HZ:
        vic = mode2vic(hdmi_mode);
        return EDID_get_vic_support(vic);
    default:
        return 0;
    }
}
EXPORT_SYMBOL(ep952_check_mode_support);

s32 ep952_get_mode_support(disp_tv_mode hdmi_mode)
{
    switch(hdmi_mode) {
    case DISP_TV_MOD_480P:
    case DISP_TV_MOD_576P:
    case DISP_TV_MOD_720P_50HZ:
    case DISP_TV_MOD_720P_60HZ:
    //case DISP_TV_MOD_1080P_50HZ:
    //case DISP_TV_MOD_1080P_60HZ:
        return 1;
    default:
        return 0;
    }
}

s32 ep952_set_mode(disp_tv_mode hdmi_mode)
{

    printk("%s: ====hdmi_mode = %d\n", __func__, hdmi_mode);

    if(ep952_get_mode_support(hdmi_mode)) {
        g_hdmi_mode = hdmi_mode;
        g_hdmi_vic = mode2vic(hdmi_mode);
        printk("%s: ====g_hdmi_vic = %d\n", __func__, g_hdmi_vic);
        if(g_ep952_enabled) {
            EP_HDMI_Set_Video_Timing(g_hdmi_vic);
        }
    }

    return 0;
}
EXPORT_SYMBOL(ep952_set_mode);

s32 ep952_get_video_timing_info(disp_video_timing **video_info)
{
    disp_video_timing *info;
    int ret = -1;
    int i, list_num;
    info = video_timing;

    list_num = sizeof(video_timing)/sizeof(disp_video_timing);
    for(i=0; i<list_num; i++) {
        if(info->vic == g_hdmi_vic){
            *video_info = info;
            ret = 0;
            break;
        }
        info ++;
    }
    return ret;
}

EXPORT_SYMBOL(ep952_get_video_timing_info);

//0:rgb;  1:yuv
static s32 ep952_get_input_csc(void)
{
    return 0;
}

s32 ep952_open(void)
{
    printk("%s:%d\n", __func__, __LINE__);
    ep952_hdmi_power_on(1);
#if !defined(EP952_DETECT_HPD)
        EP_HDMI_Init();
#endif
    EP_HDMI_Set_Video_Timing(g_hdmi_vic);               // 720p50hz
    EP_HDMI_Set_Audio_Fmt(AUD_I2S, AUD_SF_48000Hz);     // IIS input , 48KHz
    g_ep952_enabled = 1;
    printk("%s:%d\n", __func__, __LINE__);
    return 0;
}
EXPORT_SYMBOL(ep952_open);

s32 ep952_close(void)
{
    printk("%s:%d\n", __func__, __LINE__);
    g_ep952_enabled = 0;
    ep952_hdmi_power_on(0);
    msleep(100);
    return 0;
}
EXPORT_SYMBOL(ep952_close);

static int ep952_suspend(void)
{
    ep952_close();
    return 0;
}

static int ep952_resume(void)
{
    ep952_open();
    return 0;
}

static int ep952_early_suspend(void)
{
    printk("%s:%d: fixme if neccessry.\n", __func__, __LINE__);
    return 0;
}

static int ep952_late_resume(void)
{
    printk("%s:%d: fixme if neccessry.\n", __func__, __LINE__);
    return 0;
}

extern int bsp_extern_hdmi_register();

ssize_t hdmi_i2c_show_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
    char regs[0x89];
    EP952_Reg_Read(0, regs, sizeof(regs));
    ssize_t written = 0;
    int i;
    int firstVal = 1;
    for (i = 0; i < sizeof(regs); ++i)
    {
        if (!firstVal) written += sprintf(buf + written, " ");
        firstVal = 0;
        written += sprintf(buf + written, "%.2x", regs[i]);
    }
    written += sprintf(buf + written, "\n");
    return written;
}

ssize_t hdmi_i2c_store_regs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //no-op
}

static DEVICE_ATTR(regs, S_IRUGO|S_IWUSR|S_IWGRP,
    hdmi_i2c_show_regs, hdmi_i2c_store_regs);

int (*ep952_hdmi_register)(void);
static int hdmi_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    printk("%s:%d\n", __func__, __LINE__);
    ep952_i2c_client = client;

    printk("%s:%d\n", __func__, __LINE__);
     hdmi_parse_config();

#if defined(EP952_DETECT_HPD)
    EP_HDMI_Init();
#endif

    #if defined(CONFIG_SWITCH) || defined(CONFIG_ANDROID_SWITCH)
    switch_dev_register(&hdmi_switch_dev);
    #endif
    ep952_thread_enable();
    ep952_hdmi_register = bsp_extern_hdmi_register;
    ep952_hdmi_register();

    int ret;
    ret = device_create_file(&client->dev, &dev_attr_regs);

    if (ret) goto err;

    return 0;

err:
    return -ENODEV;
}

static int __devexit hdmi_i2c_remove(struct i2c_client *client)
{
    switch_dev_unregister(&hdmi_switch_dev);
    return 0;
}

static const struct i2c_device_id hdmi_i2c_id_table[] = {
    { "hdmi_i2c", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, hdmi_i2c_id_table);

static int hdmi_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    printk("%s=============\n",__func__);
    if(hdmi_i2c_id == client->adapter->nr) {
        printk("%s==111111===========\n",__func__);
        const char *type_name = "hdmi_i2c";
        ep952_i2c_adapter = client->adapter;
        printk("[DISP_I2C] hdmi_i2c_detect, get right i2c adapter, id=%d, i2c adapter=%x\n",
            ep952_i2c_adapter->nr, ep952_i2c_adapter);
        strlcpy(info->type, type_name, I2C_NAME_SIZE);
        return 0;
    }
    printk("err.\n");
    return -ENODEV;
}

static  unsigned short normal_i2c[] = {0x29, I2C_CLIENT_END};

static struct i2c_driver hdmi_i2c_driver = {
    .class = I2C_CLASS_HWMON,
    .probe        = hdmi_i2c_probe,
    .remove        = __devexit_p(hdmi_i2c_remove),
    .id_table    = hdmi_i2c_id_table,
    .driver    = {
        .name    = "hdmi_i2c",
        .owner    = THIS_MODULE,
    },
    .detect        = hdmi_i2c_detect,
    .address_list    = normal_i2c,
};

static int  hdmi_i2c_init(void)
{
    int ret;
    script_item_u value;
    ret = script_get_item(key_name, "hdmi_twi_used", &value);
    if(1 == ret && value.val) {
        hdmi_i2c_used = value.val;
        if(hdmi_i2c_used == 1) {
            ret = script_get_item(key_name, "hdmi_twi_id", &value);
            hdmi_i2c_id = (ret == SCIRPT_ITEM_VALUE_TYPE_INT)? value.val : hdmi_i2c_id;
            printk("hdmi i2c id = %d\n", hdmi_i2c_id);
            return i2c_add_driver(&hdmi_i2c_driver);
        }
    }
    return 0;
}

static void hdmi_i2c_exit(void)
{
    i2c_del_driver(&hdmi_i2c_driver);
}

static  unsigned short ddc_i2c[] = {0x50, I2C_CLIENT_END};
static const struct i2c_device_id ddc_i2c_id_table[] = {{ "ddc_i2c", 0 },{}};
MODULE_DEVICE_TABLE(i2c, ddc_i2c_id_table);

static int ddc_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    printk("%s=============\n",__func__);
    if(ddc_i2c_id == client->adapter->nr) {
        const char *type_name = "ddc_i2c";
        ddc_i2c_adapter = client->adapter;
        printk("[DDC_I2C] ddc_i2c_detect, get right i2c adapter, id=%d\n",ddc_i2c_adapter->nr);
        strlcpy(info->type, type_name, I2C_NAME_SIZE);
        return 0;
    }
    printk("[ERR]:ddc_i2c_detect err.\n");
    return -ENODEV;
}

static struct i2c_driver ddc_i2c_driver = {
    .class = I2C_CLASS_HWMON,
    .id_table    = ddc_i2c_id_table,
    .driver    = {
        .name    = "ddc_i2c",
        .owner    = THIS_MODULE,
    },
    .detect        = ddc_i2c_detect,
    .address_list    = ddc_i2c,
};

static int  ddc_i2c_init(void)
{
    int ret;
    script_item_u value;
    ret = script_get_item(key_name, "ddc_twi_used", &value);
    if(1 == ret && value.val) {
        ddc_i2c_used = value.val;
        if(ddc_i2c_used == 1) {
            ret = script_get_item(key_name, "ddc_twi_id", &value);
            ddc_i2c_id = (ret == SCIRPT_ITEM_VALUE_TYPE_INT)? value.val : ddc_i2c_id;
            printk("ddc i2c id = %d\n", ddc_i2c_id);
            return i2c_add_driver(&ddc_i2c_driver);
        }
    }
    return 0;
}

static void ddc_i2c_exit(void)
{
    i2c_del_driver(&ddc_i2c_driver);
}

s32 ep952_i2c_write(u32 client_addr, u8 *data, int size)
{
    s32 ret = 0;
    struct i2c_msg msg;

    if(hdmi_i2c_used) {
        msg.addr = client_addr;
        msg.flags = 0;
        msg.len = size;
        msg.buf = data;
        ret = i2c_transfer(ep952_i2c_adapter, &msg, 1);
    }

    return (ret==1)?0:2;
}

s32 ep952_i2c_read(u32 client_addr, u8 sub_addr, u8 *data, int size)
{
    s32 ret = 0;
    struct i2c_msg msgs[] = {
        {
            .addr   = client_addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &sub_addr,
        },
        {
            .addr   = client_addr,
        .flags  = I2C_M_RD,
            .len    = size,
            .buf    = data,
        },
    };

    if(hdmi_i2c_used) {
        ret = i2c_transfer(ep952_i2c_adapter, msgs, 2);
    }

    return (ret==2)?0:2;
}

s32 ep952_i2c_send(struct i2c_msg *msgs, int msg_count)
{
    s32 ret = 0;


    if(hdmi_i2c_used) {
        ret = i2c_transfer(ep952_i2c_adapter, msgs, msg_count);
    }

    return (ret==msg_count)?0:1;
}

s32 ddc_i2c_write(u32 client_addr, u8 *data, int size)
{
    s32 ret = 0;
    struct i2c_msg msg;

    if(ddc_i2c_used) {
        msg.addr = client_addr;
        msg.flags = 0;
        msg.len = size;
        msg.buf = data;
        ret = i2c_transfer(ddc_i2c_adapter, &msg, 1);
    }

    return (ret==1)?0:2;
}

s32 ddc_i2c_read(u32 client_addr, u8 sub_addr, u8 *data, int size)
{
    s32 ret = 0;
    struct i2c_msg msgs[] = {
        {
            .addr   = client_addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &sub_addr,
        },
        {
            .addr   = client_addr,
        .flags  = I2C_M_RD,
            .len    = size,
            .buf    = data,
        },
    };

    if(ddc_i2c_used) {
        ret = i2c_transfer(ddc_i2c_adapter, msgs, 2);
    }

    return (ret==2)?0:2;
}

s32 ddc_i2c_send(struct i2c_msg *msgs, int msg_count)
{
    s32 ret = 0;


    if(ddc_i2c_used) {
        ret = i2c_transfer(ddc_i2c_adapter, msgs, msg_count);
    }

    return (ret==msg_count)?0:1;
}

long ep952_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long karg[4];
	unsigned long ubuffer[4] = {0};
	s32 ret = 0;

	if (copy_from_user((void*)karg,(void __user*)arg,4*sizeof(unsigned long))) {
		__wrn("copy_from_user fail\n");
		return -EFAULT;
	}

	ubuffer[0] = *(unsigned long*)karg;
	ubuffer[1] = (*(unsigned long*)(karg+1));
	ubuffer[2] = (*(unsigned long*)(karg+2));
	ubuffer[3] = (*(unsigned long*)(karg+3));

	switch(cmd)	{
		case DISP_CMD_HDMI_SUPPORT_MODE:
			// returns:
			// 0: mode not supported by connected display device
			// 1: mode supported by connected display device
			// 2: no connected display device
			ret = ep952_check_mode_support(ubuffer[0]);
			break;
	}

  return ret;
}

static const struct file_operations ep952_fops = {
	.owner    = THIS_MODULE,
	.unlocked_ioctl = ep952_ioctl,
};

static int __init ep952_module_init(void)
{
    int ret;
    script_item_u value;

    pr_info("[HDMI]ep952_module_init begin\n");

    ret = script_get_item(key_name, "hdmi_used", &value);
    if(1 == ret) {
        hdmi_used = value.val;
        if(hdmi_used) {
            printk("hdmi used.\n");
            ret = script_get_item(key_name, "hdmi_power", &value);
            if(2 == ret) {
            hdmi_power_used = 1;
            memcpy((void*)hdmi_power, (void*)value.str, strlen(value.str)+1);
            printk("[HDMI] hdmi_power: %s\n", hdmi_power);
            }

            ret = script_get_item(key_name, "hpd_delay_enable", &value);
            if(1 == ret) {
                hpd_delay_enable =  value.val;
            }

            ddc_i2c_init();
            hdmi_i2c_init();
            ep952_thread_enable();

            ep952_class = class_create(THIS_MODULE, "hdmi");
            if (IS_ERR(ep952_class))	{
                pr_err("failed to allocate class\n");
                return PTR_ERR(ep952_class);
            }
            ret = alloc_chrdev_region(&devid, 0, 1, "hdmi_ep952");
            if (ret < 0) {
                pr_err("failed to allocate char device region\n");
                goto remove_class;
            }
            ep952_cdev = cdev_alloc();
            if (!ep952_cdev) {
                ret = -ENOMEM;
                goto remove_class;
            }
            cdev_init(ep952_cdev, &ep952_fops);
            ep952_cdev->owner = THIS_MODULE;
            ret = cdev_add(ep952_cdev, devid, 1);
            if (ret) {
                pr_err("failed to add char device\n");
                goto remove_class;
            }
            if (device_create(ep952_class, NULL, devid, NULL, "hdmi_ep952") == NULL) {
                goto del_cdev;
            }
        }
    } else
        hdmi_used = 0;

    return 0;

del_cdev:
    cdev_del(ep952_cdev);

remove_class:
    class_destroy(ep952_class);

    return ret;
}

static void __exit ep952_module_exit(void)
{
    pr_info("ep952_module_exit\n");
    ep952_thread_disable();
    hdmi_i2c_exit();
    ddc_i2c_exit();

    device_destroy(ep952_class,  devid);
    class_destroy(ep952_class);
    cdev_del(ep952_cdev);
}

module_init(ep952_module_init);
module_exit(ep952_module_exit);

MODULE_AUTHOR("hezuyao");
MODULE_DESCRIPTION("ep952_driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ep952");
