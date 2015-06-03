
#include <linux/init.h>
#include <linux/module.h>

int __init adc_init(void)
{
	printk("%s\n", __func__);
	
	return 0;
}

void __exit adc_exit(void)
{
	printk("%s\n", __func__);
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_LICENSE("GPL");
