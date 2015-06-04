/*
 * ʵ��Ŀ��:
 * 
 *
 * ʵ�ֲ���:
 * 1. ȷ����̬���뷶Χ
 *    1.1 ȷ����������
 *        file_operations���жϴ�����
 *
 *    1.2 ȷ��ͬʱ���ʹ�����Դ�Ĵ���
 *        file_operations: read��ioctl��poll��adc_isr
 *        file_operations��adc_isr֮��û�ù�����Դ
 *
 *    1.3 ȷ�����ʶ�ռ������Դ�Ĵ���
 *
 * 2. �����̬(���������ź���)
 *    2.1 �����ź���
 *        2.1.1 �����ڴ�
 *              �豸�ṹ��
 *
 *        2.1.2 ��ʼ��
 *              probe
 *
 *    2.2 ��ȡ�ź���/�ͷ��ź���
 *
 * ���Բ���:
 * 1. ����
 *    $ make
 *    $ arm-cortex_a8-linux-gnueabi-gcc adc_app.c -o adc_app
 *
 * 2. ������������
 *    $ cp adc.ko /source/rootfs/lib/modules/2.6.35/
 *    $ cp adc_app /source/rootfs/root
 *
 * 3. ��������
 *    # insmod /lib/modules/2.6.35/adc.ko
 *    ע��۲��ӡ
 *
 * 4. ��ȡ����
 *    #/root/adc_app /dev/adc &
 *    #/root/adc_app /dev/adc &
 *    ...
 *    �۲��ӡ
 *
 * 5. ж��
 *    # rmmod adc
 *    �۲��ӡ
 * 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>

#include <linux/semaphore.h>



#include "adc.h"

#define	ADCCON	0x0
#define	ADCDAT	0xC
#define	ADCCLRINT	0x18
#define	ADCMUX	0x1C

struct adc_cdev{
	struct cdev cdev;
	dev_t devno;
	
	struct clk *clk;
	struct resource *reg_res;
	
	// 2.1.1 �����ڴ�
	struct semaphore sem;
	void __iomem *regs;
	
	int irqno;
	wait_queue_head_t readq;
	
	struct fasync_struct *fasyncq;
};

struct class * adc_class;

void adc_dev_init(struct adc_cdev *adc)
{
	unsigned long reg;
	
	reg = readl(adc->regs + ADCCON);
	
	// start by read
	reg |= 0x1 << 1;
	
	// not standby
	reg &= ~(0x1 << 2);
	
	// prescaler (1/256)
	reg |= 0xff << 6;
	
	// enable prescaler
	reg |= 0x1 << 14;
	
	// resolution 12bit
	reg |= 0x1 << 16;
	
	writel(reg, adc->regs + ADCCON);
	
	// aux select (AIN0)
	reg = readl(adc->regs + ADCMUX);
	reg &= ~0xf;
	writel(reg,adc->regs + ADCMUX);
	
	// enable
	readl(adc->regs + ADCDAT);
}

int adc_dev_read(struct adc_cdev *adc)
{
	return (readl(adc->regs + ADCDAT) & 0xfff);
}

void adc_dev_set_resolution(struct adc_cdev *adc, int resolution)
{
	unsigned long reg;
	
	reg = readl(adc->regs + ADCCON);
	if (12 == resolution){
		// resolution 12bit
		reg |= 0x1 << 16;
	} else {
		reg &= ~(0x1 << 16);
	}
	
	writel(reg, adc->regs + ADCCON);
}

int adc_dev_is_finished(struct adc_cdev *adc)
{
	return (readl(adc->regs + ADCCON) & (0x1 << 15));
}

irqreturn_t adc_isr(int irq, void *dev_id)
{
	struct adc_cdev *adc = (struct adc_cdev *)dev_id;
	
	printk("%s\n", __func__);
	
  wake_up_interruptible(&adc->readq);
  
  if (adc->fasyncq != NULL) {
  	kill_fasync(&adc->fasyncq, SIGIO, POLL_IN);
	}
  
  writel(0, adc->regs + ADCCLRINT);
  
	return IRQ_HANDLED;
}

int adc_open(struct inode *inode, struct file *filp)
{
	struct adc_cdev *adc = container_of(inode->i_cdev, struct adc_cdev, cdev);
	
	printk("%s\n", __func__);
	
	filp->private_data = adc;
	
	
	return 0;
}

int adc_dev_read_lock(struct adc_cdev *adc)
{
	int vol = -1;
	
	// 2.2 ��ȡ�ź���/�ͷ��ź���
	down(&adc->sem);
	if (adc_dev_is_finished(adc)) {
		vol = adc_dev_read(adc);
	}
	up(&adc->sem);
	
	return vol;
}

ssize_t adc_read(struct file *filp, char __user *buf, size_t len, loff_t *loff)
{
	ssize_t ret = 0;
	ssize_t vol;
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	if (!(filp->f_flags & O_NONBLOCK)) {
	// ����ģʽ
	// �жϵ�ǰ��ѹֵ�Ƿ�ɼ���ϣ��ɼ���ϣ���ȡ��ѹ�����ظ�Ӧ�ó���
	//                             û�вɼ����, �õ�ǰ��������
		ret = wait_event_interruptible(adc->readq, (vol = adc_dev_read_lock(adc)) >= 0);
		if (ret < 0) {
			goto exit;
		}

	} else {
	// ������ģʽ
	// �жϵ�ǰ��ѹֵ�Ƿ�ɼ���ϣ��ɼ���ϣ���ȡ��ѹ�����ظ�Ӧ�ó���
	//                             û�вɼ����, ���ش���(-EAGAIN)
		if ((vol = adc_dev_read_lock(adc)) < 0) {
			ret = -EAGAIN;
			goto exit;
		}
	}
	
	ret = copy_to_user(buf, &vol, sizeof(vol));
	if (ret != 0){
		ret = -EFAULT;
	} else {
		ret = vol;
	}
	
exit:
	return ret;
}

long adc_unlocked_ioctl(struct file *filp, unsigned int requests, unsigned long arg)
{
	long ret = 0;
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	switch (requests){
		case IOCTL_SET_RESOLUTION:
			printk("%s: IOCTL_SET_RESOLUTION(arg = %d)\n", __func__, (int)arg);
			adc_dev_set_resolution(adc, (int)arg);
			break;
			
		default:
			ret = -EINVAL;
			break;
	}
	
	return ret;
}

unsigned int adc_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	// �Ӷ��ȴ����е�wait
	poll_wait(filp, &adc->readq, wait);
	
	if (adc_dev_is_finished(adc)){
		// �ɶ�
		mask |= POLLIN | POLLRDNORM;
	}   
	
	// ��д�ȴ����е�wait
	// poll_wait(filp, &dev->w_wait, wait); 
	// if (...){
		// ��д
	  // mask |= POLLOUT | POLLWRNORM;
  // }
     return mask;
}

// 2. �����첽�źŶ���
// Oflags = fcntl(fd, F_GETFL); 
// Oflags |= FASYNC;
// fcntl(fd, F_SETFL, Oflags);
int adc_fasync(int fd, struct file *filp, int mode)
{
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	return fasync_helper(fd, filp, mode, &adc->fasyncq);
	
}

int adc_release(struct inode *inode, struct file *filp)
{
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	// �����첽�źŶ���
	adc_fasync(-1, filp, 0);
	adc->fasyncq = NULL;
	
	return 0;
}

struct file_operations fops = {
	.open = adc_open,
	.read = adc_read,
	.unlocked_ioctl = adc_unlocked_ioctl,
	.poll = adc_poll,
	.fasync = adc_fasync,
	.release = adc_release,
};

int adc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *device;
	struct adc_cdev *adc;
	struct resource *res;
	
	printk("%s\n", __func__);
	
	adc = kmalloc(sizeof(struct adc_cdev), GFP_KERNEL);
	if (NULL == adc){
		ret = -ENOMEM;
		goto exit;
	}
	platform_set_drvdata(pdev, adc);
	
	cdev_init(&adc->cdev, &fops);
	adc->cdev.owner = THIS_MODULE;
	
	adc->clk = clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)){
		ret = PTR_ERR(adc->clk);
		goto err_clk_get;
	}
	
	clk_enable(adc->clk);
	
	printk("%s: clock is OK!\n", __func__);
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res){
		ret = -ENOENT;
		goto err_platform_get_resource;
	}
	printk("%s: res = <%08X, %d>\n", __func__, res->start, resource_size(res));
	
	adc->reg_res = request_mem_region(res->start, resource_size(res), "adc");
	if (NULL == adc->reg_res){
		ret = -EBUSY;
		goto err_request_mem_region;
	}
	
	adc->regs = ioremap(adc->reg_res->start, resource_size(res));
	if (NULL == adc->regs){
		ret = -ENOMEM;
		goto err_ioremap;
	}
	printk("%s: regs = <%p>\n", __func__, adc->regs);
	
	ret = platform_get_irq(pdev, 0);
	if (ret < 0){
		goto err_platform_get_irq;
	}
	printk("%s: irqno = <%d>\n", __func__, ret);
	adc->irqno = ret;
	
	ret = request_irq(adc->irqno, adc_isr, IRQF_DISABLED, "adc", adc);
	if (ret < 0){
		goto err_request_irq;
	}
	
	// 2.1.2 ��ʼ��
	sema_init(&adc->sem, 1); 
	
	adc_dev_init(adc);
	init_waitqueue_head(&adc->readq);
	adc->fasyncq = NULL;
	
	ret = alloc_chrdev_region(&adc->devno, 0, 1, "adc");
	if (ret < 0){
		goto err_alloc_chrdev_region;
	}
	
	ret = cdev_add(&adc->cdev, adc->devno, 1);
	if (ret < 0){
		goto err_cdev_add;
	}
	
	device = device_create(adc_class, NULL, adc->devno, NULL, "adc");
	if (IS_ERR(device)){
		ret = PTR_ERR(device);
		goto err_device_create;
	}
	goto exit;

err_device_create:
	cdev_del(&adc->cdev);
	
err_cdev_add:
	unregister_chrdev_region(adc->devno, 1);
	
err_alloc_chrdev_region:
	free_irq(adc->irqno, adc);
err_request_irq:
err_platform_get_irq:
	iounmap(adc->regs);
err_ioremap:
	release_mem_region(adc->reg_res->start, resource_size(adc->reg_res));
err_request_mem_region:
err_platform_get_resource:
	clk_disable(adc->clk);
	clk_put(adc->clk);
	
err_clk_get:
	kfree(adc);
	
exit:
	return ret;
}

int adc_remove(struct platform_device *pdev)
{
	struct adc_cdev *adc = platform_get_drvdata(pdev);
	
	printk("%s\n", __func__);
	
	device_destroy(adc_class, adc->devno);
	cdev_del(&adc->cdev);
	unregister_chrdev_region(adc->devno, 1);
	free_irq(adc->irqno, adc);
	iounmap(adc->regs);
	release_mem_region(adc->reg_res->start, resource_size(adc->reg_res));
	clk_disable(adc->clk);
	clk_put(adc->clk);
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
	int ret = 0;
	
	printk("%s\n", __func__);
	
	adc_class = class_create(THIS_MODULE, "adc");
	if (IS_ERR(adc_class)){
		ret = PTR_ERR(adc_class);
		goto exit;
	}
	
	ret = platform_driver_register(&adc_driver);
	if (ret < 0){
		class_destroy(adc_class);
	}

exit:
	return ret;
}

void __exit adc_exit(void)
{
	printk("%s\n", __func__);
	
	platform_driver_unregister(&adc_driver);
	class_destroy(adc_class);
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_LICENSE("GPL");
