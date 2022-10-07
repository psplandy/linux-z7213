#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <mach/sys_config.h>
#include <linux/gpio.h>
#include <linux/pinctrl/pinconf-sunxi.h>
#include <linux/pinctrl/consumer.h>

static char *ma8601_para = "ma8601_para";
struct ma8601_dev {
	struct device	*dev;
	struct cdev	*cdev;
	dev_t		devid;
	struct class	*class;
	int rst_used;
	int rst_pin;
	int enable_pin;
}ma8601;

static struct kobject *vbus_en_kobject;

static ssize_t vbus_en_get(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf)
{
        return sprintf(buf, "%d\n", gpio_get_value(ma8601.enable_pin));
}

static ssize_t vbus_en_set(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf, size_t count)
{
		int value;
        sscanf(buf, "%du", &value);
        gpio_set_value(ma8601.enable_pin, value);
        return count;
}

static struct kobj_attribute vbus_en_attribute =__ATTR(vbus_en, 0660, vbus_en_get, vbus_en_set);

static void ma8601_enable(void)
{
	gpio_set_value(ma8601.enable_pin, 1);
}

static void ma8601_disable(void)
{
	gpio_set_value(ma8601.enable_pin, 0);
}

static void ma8601_reset(void)
{
	gpio_set_value(ma8601.rst_pin, 0);
	msleep(20);
	gpio_set_value(ma8601.rst_pin, 1);
}

int sunxi_gpio_req(struct gpio_config *gpio)
{
	int            ret = 0;
	char           pin_name[8] = {0};
	unsigned long  config;

	sunxi_gpio_to_name(gpio->gpio, pin_name);
	config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
	ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
	if (ret) {
		printk("set gpio %s mulsel failed.\n",pin_name);
		return -1;
	}

	if (gpio->pull != GPIO_PULL_DEFAULT){
		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, gpio->pull);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			printk("set gpio %s pull mode failed.\n",pin_name);
			return -1;
		}
	}

	if (gpio->drv_level != GPIO_DRVLVL_DEFAULT){
		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, gpio->drv_level);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			printk("set gpio %s driver level failed.\n",pin_name);
			return -1;
		}
	}

	if (gpio->data != GPIO_DATA_DEFAULT) {
		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT, gpio->data);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			printk("set gpio %s initial val failed.\n",pin_name);
			return -1;
		}
	}

	return 0;
}

static int ma8601_get_res(void)
{
	script_item_value_type_e type;
	script_item_u val; 
	struct gpio_config  *gpio_rst = NULL;
	struct gpio_config  *gpio_vbus_en = NULL;

	type = script_get_item(ma8601_para, "ma8601_used", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		printk("failed to fetch ma8601 configuration!\n");
		return -1;
	}
	if (!val.val) {
		printk("no reset pin used in configuration\n");
		return -1;
	}
	ma8601.rst_used = val.val;

	type = script_get_item(ma8601_para, "enable_pin", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO!=type)
		printk("get ma8601 VBUS_EN gpio failed\n");
	else {
		gpio_vbus_en = &val.gpio;
		ma8601.enable_pin = gpio_vbus_en->gpio;
		sunxi_gpio_req(gpio_vbus_en);
	}

	type = script_get_item(ma8601_para, "rst_pin", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO!=type)
		printk("get ma8601 reset gpio failed\n");
	else {
		gpio_rst = &val.gpio;
		ma8601.rst_pin = gpio_rst->gpio;
		sunxi_gpio_req(gpio_rst);
		printk("get ma8601 reset gpio OK\n");
	}

	return 0;
}

long ma8601_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

int ma8601_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int __devinit ma8601_probe(struct platform_device *pdev)
{
	ma8601_disable();
	ma8601_reset();
	msleep(100);
	ma8601_enable();

	return 0;
}

static int __devexit ma8601_remove(struct platform_device *pdev)
{
	pr_info("ma8601 reset gpio is released !!\n");
	return 0;
}

#ifdef CONFIG_PM
static int ma8601_pm_suspend(struct device *dev)
{
	return 0;
}

static int ma8601_pm_resume(struct device *dev)
{
	ma8601_disable();
	ma8601_reset();
	msleep(100);
	ma8601_enable();

	return 0;
}

static struct dev_pm_ops ma8601_pm_ops = {
	.suspend	= ma8601_pm_suspend,
	.resume		= ma8601_pm_resume,
};
#endif

static struct platform_device ma8601_dev = {
	.name           = "ma8601",
};

static struct platform_driver ma8601_driver = {
	.driver.name    = "ma8601",
	.driver.owner   = THIS_MODULE,
#ifdef CONFIG_PM
	.driver.pm      = &ma8601_pm_ops,
#endif
	.probe          = ma8601_probe,
	.remove         = __devexit_p(ma8601_remove),
};

static const struct file_operations ma8601_fops =
{
	.owner          = THIS_MODULE,
	.open           = ma8601_open,
	.unlocked_ioctl = ma8601_ioctl,
};


int __init ma8601_init(void)
{
	int err;

	ma8601_get_res();
	if (!ma8601.rst_used)
		return 0;

	alloc_chrdev_region(&ma8601.devid, 0, 1, "ma8601");
	ma8601.cdev = cdev_alloc();
	cdev_init(ma8601.cdev, &ma8601_fops);
	ma8601.cdev->owner = THIS_MODULE;

	err = cdev_add(ma8601.cdev, ma8601.devid, 1);
	if (err) {
		printk("cdev_add fail.\n");
		return -1;
	}

	ma8601.class = class_create(THIS_MODULE, "ma8601");
	if (IS_ERR(ma8601.class)) {
		printk("class_create fail\n");
		return -1;
	}

	device_create(ma8601.class, NULL, ma8601.devid, NULL, "ma8601");

	err = platform_device_register(&ma8601_dev);
	if(err == 0) {
		err=platform_driver_register(&ma8601_driver);
	}

	vbus_en_kobject = kobject_create_and_add("ma8601", NULL);
	if(!vbus_en_kobject)
		return -ENOMEM;

	err = sysfs_create_file(vbus_en_kobject, &vbus_en_attribute.attr);
	if (err) {
			pr_debug("failed to create the vbus_en sysfs node\n");
	}

	printk("ma8601 init finish\n");
	return err;
}

static void __exit ma8601_exit(void)
{
	if (!ma8601.rst_used)
		return;
	
	kobject_put(vbus_en_kobject);
	platform_driver_unregister(&ma8601_driver);
	platform_device_unregister(&ma8601_dev);
	
	device_destroy(ma8601.class,  ma8601.devid);
	class_destroy(ma8601.class);
	cdev_del(ma8601.cdev);
}

module_init(ma8601_init);
module_exit(ma8601_exit);

