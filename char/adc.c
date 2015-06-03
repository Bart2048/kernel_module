
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
	
	// �����豸����
	
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
 * @brief						��readϵͳ���ü�ӵ���
 * @param[in]				filp												struct file�ṹ�����ָ�룬��read�����е�fd��Ӧ
 * @param[out]			buf													Ӧ�ó���ռ��buf
 * @param[in]				len													��ȡ���ݵĳ���
 * @param[out]			loff												�޸��ļ�λ��ָ��(�������豸��˵��û����)
 * @notes						__user											��ʶbufΪӦ�ó���ռ�buf������ֱ��ʹ���ڴ濽��
 */
ssize_t adc_read(struct file *filp, char __user *buf, size_t len, loff_t *loff)
{
	ssize_t ret = 0;
	
	printk("%s\n", __func__);
	
/*
 * @brief						�������ݵ�Ӧ�ó���ռ�
 * @param[out]			to													Ŀ�껺������ַ
 * @param[in]			 	from												Դ��������ַ
 * @param[in]				n														�������ֽ���
 * @return					δ�ɹ��������ֽ���
 * @notes						__user											��ʶtoΪӦ�ó���ռ�buf
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
 * @brief						��̬�����豸���
 * @param[out]			dev													�豸���(��һ��)
 * @param[in]				firstminor									�����һ���α��
 * @param[in]				count												����������
 * @param[in]				name												�豸����(��/proc/devices�ļ��пɼ�) 
 * @return					=0													����ɹ�
 * 									<0													������
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

// �豸�б�(������������豸ʱ)
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

// ��BSP�����е��豸ƥ��
	.id_table = adc_ids,
	.driver = {
		.name = "adc",
		.owner = THIS_MODULE,
// �豸�б�(���豸���е��豸�б����ƥ��)
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
