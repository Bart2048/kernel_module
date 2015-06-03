
#include <linux/init.h>
#include <linux/module.h>

#include <linux/platform_device.h>

int adc_probe(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	
	return 0;
}

int adc_remove(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	
	return 0;
}

// 设备列表(驱动多类相近设备时)
struct platform_device_id adc_ids[] = {
	[0] = {
		.name = "s3c-adc",
	},
	{/* end */}
};
MODULE_DEVICE_TABLE(platform, adc_ids);

struct platform_driver adc_driver = {
	.probe = adc_probe,
	.remove = adc_remove,

// 和BSP代码中的设备匹配
	.id_table = adc_ids,
	.driver = {
		.name = "adc",
		.owner = THIS_MODULE,
// 设备列表(和设备树中的设备列表进行匹配)
//		.of_match_table = ...
	},
};

int __init adc_init(void)
{
	printk("%s\n", __func__);
	
	platform_driver_register(&adc_driver);
	
	return 0;
}

void __exit adc_exit(void)
{
	printk("%s\n", __func__);
	platform_driver_unregister(&adc_driver);
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_LICENSE("GPL");
