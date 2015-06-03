/*
 * ʵ��Ŀ��:
 * Ӳ������
 *
 * ʵ�ֲ���:
 * 1. clk����
 *    1.1 ��ȡʱ��
 *    1.2 ʹ��ʱ��
 *    1.3 ����ʱ��Ƶ��(����������)
 *
 * 2. IOMEM����
 *    2.1 ��ȡ(���豸�л�ȡ)
 *    2.2 ����/ע��
 *    2.3 ӳ��/ȡ��ӳ��
 *    2.4 ʹ��
 *
 * 3. IRQ����
 *    3.1 ��ȡ��
 *    3.2 ʵ��ISR
 *    3.3 ע���ж�
 *    
 * 4. Ӳ������
 *    adc_init
 *    adc_read
 *    adc_set_resolution
 *
 * 5. ��(����)
 *    5.1 �����ȴ�����
 *        5.1.1 �����ڴ�
 *              �豸�Ľṹ��
 *
 *        5.1.2 ��ʼ��
 *              probe
 *
 *    5.2 ����Ҫ�ȴ��ĵط��ȴ�
 *        file_operations : read
 *
 *    5.3 ����Ҫ���ѵĵط�����
 *  			adc_isr
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
 * 4. ȷ��iomem����
 *    # cat /proc/iomem
 *
 * 5. ȷ���ж�ע��ɹ�
 *    # cat /proc/interrupts
 *
 * 6. ��ȡ����
 *    #/root/adc_app
 *
 * 7. ж��
 *    # rmmod adc
 *    �۲��ӡ
 *
 * 8. ȷ���ж�ע���ɹ�
 *    # cat /proc/interrupts
 * 
 *
 */

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

// mknod
#include <linux/device.h>

// clk
#include <linux/clk.h>

// ioremap
#include <linux/io.h>

// irq
#include <linux/interrupt.h>

// wait queue
#include <linux/wait.h>
#include <linux/sched.h>

#include "adc.h"

#define	ADCCON	0x0
#define	ADCDAT	0xC
#define	ADCCLRINT	0x18
#define	ADCMUX	0x1C

struct adc_cdev{
	struct cdev cdev;
	dev_t devno;
	
	// �����豸����
	struct clk *clk;
	struct resource *reg_res;
	
	void __iomem *regs;
	int irqno;
	
	// 5.1.1 �����ڴ�
	wait_queue_head_t readq;
};

struct class * adc_class;

// 4. Ӳ������
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


// 3.2 ʵ��ISR
/*
 * @brief          �����豸�ж�(����ϵͳ�ܵ�IRQ����������)
 * @param[in]      irq                     �жϺ�
 * @param[in]      dev_id                  �����豸�Ľṹ��ָ��
 * @return         �ж��Ƿ���
 *                 @IRQ_HANDLED            �ж��Ѿ�����
 *                 @IRQ_NONE               �жϻ�δ����
 * @notes          ���ж��������е��ã���ARM����������IRQģʽ��ִ��
 */
irqreturn_t adc_isr(int irq, void *dev_id)
{
	struct adc_cdev *adc = (struct adc_cdev *)dev_id;
	
	printk("%s\n", __func__);
	
	// 1. �ж��豸�Ƿ����ж�(�����ж�)
	//    ����Ҫ��
	
  // 2. �жϴ���
  //    ���ѵȴ���ȡ��ѹ�Ľ���
  
  //    5.3 ����Ҫ���ѵĵط�����
  wake_up_interruptible(&adc->readq);
  
  // 3. ����ж��ڲ�����
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
	ssize_t vol;
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	if (!(filp->f_flags & O_NONBLOCK)) {
	// ����ģʽ
	// �жϵ�ǰ��ѹֵ�Ƿ�ɼ���ϣ��ɼ���ϣ���ȡ��ѹ�����ظ�Ӧ�ó���
	//                             û�вɼ����, �õ�ǰ��������
	
		// 5.2 ����Ҫ�ȴ��ĵط��ȴ�
		ret = wait_event_interruptible(adc->readq, adc_dev_is_finished(adc));
		if (ret < 0) {
			goto exit;
		}
		
		
	} else {
	
	// ������ģʽ
	// �жϵ�ǰ��ѹֵ�Ƿ�ɼ���ϣ��ɼ���ϣ���ȡ��ѹ�����ظ�Ӧ�ó���
	//                             û�вɼ����, ���ش���(-EAGAIN)
	}
	
	vol = adc_dev_read(adc);
	
/*
 * @brief						�������ݵ�Ӧ�ó���ռ�
 * @param[out]			to													Ŀ�껺������ַ
 * @param[in]			 	from												Դ��������ַ
 * @param[in]				n														�������ֽ���
 * @return					δ�ɹ��������ֽ���
 * @notes						__user											��ʶtoΪӦ�ó���ռ�buf
 * unsigned long copy_to_user(void __user *to, const void *from, unsigned long n);
 */
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
	

// 1.1 ��ȡʱ��
/*
 * @brief						��ȡʱ��
 * @param[in]				dev													ָ����ȡʱ�ӵ��豸
 * @param[in]				con_id											�����豸ID
 * @return 					����ʱ�ӽṹ
 *									IS_ERR(����ֵ)							�ж��Ƿ��Ǵ�����
 *									PTR_ERR(����ֵ)							ȡ�ô�����
 * struct clk *clk_get(struct device *dev, const char *con_id);
 */
	adc->clk = clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)){
		ret = PTR_ERR(adc->clk);
		goto err_clk_get;
	}
	
// 1.2 ʹ��ʱ��
/*
 * @brief						ʹ��ʱ��
 * @param[in]				clk													ʱ�ӽṹ
 * @return 					���
 *									@li 0												ʹ�ܳɹ�
 *									@li !0											������
 * int clk_enable(stuct clk *clk);
 */
	clk_enable(adc->clk);
	
	printk("%s: clock is OK!\n", __func__);
	
	// 2.1 ��ȡ
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res){
		ret = -ENOENT;
		goto err_platform_get_resource;
	}
	printk("%s: res = <%08X, %d>\n", __func__, res->start, resource_size(res));
	
	// 2.2 ����
	adc->reg_res = request_mem_region(res->start, resource_size(res), "adc");
	if (NULL == adc->reg_res){
		ret = -EBUSY;
		goto err_request_mem_region;
	}
	
	// 2.3 ӳ��
	adc->regs = ioremap(adc->reg_res->start, resource_size(res));
	if (NULL == adc->regs){
		ret = -ENOMEM;
		goto err_ioremap;
	}
	printk("%s: regs = <%p>\n", __func__, adc->regs);
	
	// 3.1 ��ȡ�жϺ�
	ret = platform_get_irq(pdev, 0);
	if (ret < 0){
		goto err_platform_get_irq;
	}
	printk("%s: irqno = <%d>\n", __func__, ret);
	adc->irqno = ret;
	
// 3.3 ע���ж�
/*
* @brief          �����ж�
* @param[in]      irq                     �жϺ�
* @param[in]      handler                 �жϴ������
* @param[in]      flags                   �жϱ�־(�жϴ������ԡ��ж��Ƿ����ж��Ƿ����Ƕ��......)
*                                         ÿ�ֱ�־ռ��һ��bit�����Զ����־���
* @param[in]      name                    �ж���������
* @param[in]      dev_id                  �����豸�Ľṹ��ָ��,�ڵ����жϷ������ʱ��������
* @return         @li 0                   ��ʾ����ɹ�
*                 @li < 0                 ������
* @notes          cat /proc/interrupts
* int request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id);
*/
	ret = request_irq(adc->irqno, adc_isr, IRQF_DISABLED, "adc", adc);
	if (ret < 0){
		goto err_request_irq;
	}
	
	adc_dev_init(adc);
	
// 5.1.2 ��ʼ��
	init_waitqueue_head(&adc->readq);
	
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
	
/*
 * @brief          ע���ж�
 * @param[in]      irq                     �жϺ�
 * @param[in]      dev_id                  �����豸�Ľṹ��ָ��
 * void free_irq(unsigned int irq, void *dev_id);
 */
	free_irq(adc->irqno, adc);
       
	
// 2.3 ȡ��ӳ��
	iounmap(adc->regs);
	
// 2.2 ע��
	release_mem_region(adc->reg_res->start, resource_size(adc->reg_res));
	
/*
 * @brief						��ֹʱ��
 * @param[in]				clk													ʱ�ӽṹ
 * void clk_disable(struct clk *clk);
 */
	clk_disable(adc->clk);
 
 
/*
 * @brief						�ͷ�ʱ��
 * @param[in]				clk													ʱ�ӽṹ
 * void clk_put(stuct clk *clk);
 */
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
