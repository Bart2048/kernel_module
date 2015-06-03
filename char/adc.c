
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/cdev.h>
#include <linux/fs.h>

// copy_to_user / copy_from_user
#include <linux/uaccess.h>

#include <linux/ioctl.h>

// kmalloc
#include <linux/slab.h>

#include "adc.h"


struct adc_cdev{
	struct cdev cdev;
	dev_t devno;
	
	// 增加设备属性
	
};

int adc_open(struct inode *inode, struct file *filp)
{
	struct adc_cdev *adc = container_of(inode->i_cdev, struct adc_cdev, cdev);
	
	printk("%s\n", __func__);
	
	filp->private_data = adc;
	
	
	return 0;
}

int adc_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;
}

/*
 * @brief						被read系统调用间接调用
 * @param[in]				filp												struct file结构体变量指针，跟read参数中的fd对应
 * @param[out]			buf													应用程序空间的buf
 * @param[in]				len													读取数据的长度
 * @param[out]			loff												修改文件位置指针(对于流设备来说，没有用)
 * @notes						__user											标识buf为应用程序空间buf，不能直接使用内存拷贝
 */
ssize_t adc_read(struct file *filp, char __user *buf, size_t len, loff_t *loff)
{
	ssize_t ret = 0;
	
	printk("%s\n", __func__);
	
/*
 * @brief						拷贝数据到应用程序空间
 * @param[out]			to													目标缓存区地址
 * @param[in]			 	from												源缓存区地址
 * @param[in]				n														拷贝的字节数
 * @return					未成功拷贝的字节数
 * @notes						__user											标识to为应用程序空间buf
 * unsigned long copy_to_user(void __user *to, const void *from, unsigned long n);
 */
	ret = copy_to_user(buf, "hello world\n", 13);
	if (ret != 0){
		ret = -EFAULT;
	}
	
	return ret;
}

long adc_unlocked_ioctl(struct file *filp, unsigned int requests, unsigned long arg)
{
	long ret = 0;
	
	printk("%s\n", __func__);
	
	switch (requests){
		case IOCTL_SET_RESOLUTION:
			printk("%s: IOCTL_SET_RESOLUTION(arg = %d)\n", __func__, (int)arg);
			break;
			
		default:
			ret = -EINVAL;
			break;
	}
	
	return ret;
}

struct file_operations fops = {
	.open = adc_open,
	.release = adc_release,
	.read = adc_read,
	.unlocked_ioctl = adc_unlocked_ioctl,
};

int adc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct adc_cdev *adc;
	
	printk("%s\n", __func__);
	
	adc = kmalloc(sizeof(struct adc_cdev), GFP_KERNEL);
	if (NULL == adc){
		ret = -ENOMEM;
		goto exit;
	}
	platform_set_drvdata(pdev, adc);
	
	cdev_init(&adc->cdev, &fops);
	adc->cdev.owner = THIS_MODULE;
	
/*
 * @brief						动态分配设备编号
 * @param[out]			dev													设备编号(第一个)
 * @param[in]				firstminor									分配第一个次编号
 * @param[in]				count												分配编号数量
 * @param[in]				name												设备名称(在/proc/devices文件中可见) 
 * @return					=0													分配成功
 * 									<0													错误码
 * int alloc_chrdev_region(dev_t *dev, unsigned int firstminor, unsigned int count, char *name);
 */
	ret = alloc_chrdev_region(&adc->devno, 0, 1, "adc");
	if (ret < 0){
		goto err_alloc_chrdev_region;
	}
	
	ret = cdev_add(&adc->cdev, adc->devno, 1);
	if (ret < 0){
		goto err_cdev_add;
	}
	
	goto exit;

err_cdev_add:
	unregister_chrdev_region(adc->devno, 1);
	
err_alloc_chrdev_region:
	kfree(adc);
	
exit:
	return ret;
}

int adc_remove(struct platform_device *pdev)
{
	struct adc_cdev *adc = platform_get_drvdata(pdev);
	
	printk("%s\n", __func__);
	
	cdev_del(&adc->cdev);
	unregister_chrdev_region(adc->devno, 1);
	kfree(adc);
	
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
