/*
 * 实现目标:
 * 硬件操作
 *
 * 实现步骤:
 * 1. clk设置
 *    1.1 获取时钟
 *    1.2 使能时钟
 *    1.3 设置时钟频率(本驱动不做)
 *
 * 2. IOMEM配置
 *    2.1 获取(从设备中获取)
 *    2.2 申请/注销
 *    2.3 映射/取消映射
 *    2.4 使用
 *
 * 3. IRQ配置
 *    3.1 获取号
 *    3.2 实现ISR
 *    3.3 注册中断
 *    
 * 4. 硬件驱动
 *    adc_init
 *    adc_read
 *    adc_set_resolution
 *
 * 5. 读(阻塞)
 *    5.1 创建等待队列
 *        5.1.1 分配内存
 *              设备的结构体
 *
 *        5.1.2 初始化
 *              probe
 *
 *    5.2 在需要等待的地方等待
 *        file_operations : read
 *
 *    5.3 在需要唤醒的地方唤醒
 *  			adc_isr
 *
 * 调试步骤:
 * 1. 编译
 *    $ make
 *    $ arm-cortex_a8-linux-gnueabi-gcc adc_app.c -o adc_app
 *
 * 2. 拷贝到开发板
 *    $ cp adc.ko /source/rootfs/lib/modules/2.6.35/
 *    $ cp adc_app /source/rootfs/root
 *
 * 3. 加载驱动
 *    # insmod /lib/modules/2.6.35/adc.ko
 *    注意观察打印
 *
 * 4. 确认iomem申请
 *    # cat /proc/iomem
 *
 * 5. 确认中断注册成功
 *    # cat /proc/interrupts
 *
 * 6. 读取数据
 *    #/root/adc_app
 *
 * 7. 卸载
 *    # rmmod adc
 *    观察打印
 *
 * 8. 确认中断注销成功
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
	
	// 增加设备属性
	struct clk *clk;
	struct resource *reg_res;
	
	void __iomem *regs;
	int irqno;
	
	// 5.1.1 分配内存
	wait_queue_head_t readq;
};

struct class * adc_class;

// 4. 硬件驱动
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


// 3.2 实现ISR
/*
 * @brief          处理设备中断(操作系统总的IRQ处理函数调用)
 * @param[in]      irq                     中断号
 * @param[in]      dev_id                  描述设备的结构体指针
 * @return         中断是否处理
 *                 @IRQ_HANDLED            中断已经处理
 *                 @IRQ_NONE               中断还未处理
 * @notes          在中断上下文中调用，在ARM处理器中在IRQ模式下执行
 */
irqreturn_t adc_isr(int irq, void *dev_id)
{
	struct adc_cdev *adc = (struct adc_cdev *)dev_id;
	
	printk("%s\n", __func__);
	
	// 1. 判断设备是否发生中断(共享中断)
	//    不需要做
	
  // 2. 中断处理
  //    唤醒等待读取电压的进程
  
  //    5.3 在需要唤醒的地方唤醒
  wake_up_interruptible(&adc->readq);
  
  // 3. 清除中断内部锁存
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
	ssize_t vol;
	struct adc_cdev *adc = (struct adc_cdev *)filp->private_data;
	
	printk("%s\n", __func__);
	
	if (!(filp->f_flags & O_NONBLOCK)) {
	// 阻塞模式
	// 判断当前电压值是否采集完毕，采集完毕，读取电压，返回给应用程序
	//                             没有采集完毕, 让当前进程休眠
	
		// 5.2 在需要等待的地方等待
		ret = wait_event_interruptible(adc->readq, adc_dev_is_finished(adc));
		if (ret < 0) {
			goto exit;
		}
		
		
	} else {
	
	// 非阻塞模式
	// 判断当前电压值是否采集完毕，采集完毕，读取电压，返回给应用程序
	//                             没有采集完毕, 返回错误(-EAGAIN)
	}
	
	vol = adc_dev_read(adc);
	
/*
 * @brief						拷贝数据到应用程序空间
 * @param[out]			to													目标缓存区地址
 * @param[in]			 	from												源缓存区地址
 * @param[in]				n														拷贝的字节数
 * @return					未成功拷贝的字节数
 * @notes						__user											标识to为应用程序空间buf
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
	

// 1.1 获取时钟
/*
 * @brief						获取时钟
 * @param[in]				dev													指定获取时钟的设备
 * @param[in]				con_id											连接设备ID
 * @return 					返回时钟结构
 *									IS_ERR(返回值)							判断是否是错误码
 *									PTR_ERR(返回值)							取得错误码
 * struct clk *clk_get(struct device *dev, const char *con_id);
 */
	adc->clk = clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)){
		ret = PTR_ERR(adc->clk);
		goto err_clk_get;
	}
	
// 1.2 使能时钟
/*
 * @brief						使能时钟
 * @param[in]				clk													时钟结构
 * @return 					结果
 *									@li 0												使能成功
 *									@li !0											错误码
 * int clk_enable(stuct clk *clk);
 */
	clk_enable(adc->clk);
	
	printk("%s: clock is OK!\n", __func__);
	
	// 2.1 获取
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res){
		ret = -ENOENT;
		goto err_platform_get_resource;
	}
	printk("%s: res = <%08X, %d>\n", __func__, res->start, resource_size(res));
	
	// 2.2 申请
	adc->reg_res = request_mem_region(res->start, resource_size(res), "adc");
	if (NULL == adc->reg_res){
		ret = -EBUSY;
		goto err_request_mem_region;
	}
	
	// 2.3 映射
	adc->regs = ioremap(adc->reg_res->start, resource_size(res));
	if (NULL == adc->regs){
		ret = -ENOMEM;
		goto err_ioremap;
	}
	printk("%s: regs = <%p>\n", __func__, adc->regs);
	
	// 3.1 获取中断号
	ret = platform_get_irq(pdev, 0);
	if (ret < 0){
		goto err_platform_get_irq;
	}
	printk("%s: irqno = <%d>\n", __func__, ret);
	adc->irqno = ret;
	
// 3.3 注册中断
/*
* @brief          申请中断
* @param[in]      irq                     中断号
* @param[in]      handler                 中断处理程序
* @param[in]      flags                   中断标志(中断触发极性、中断是否共享、中断是否可以嵌套......)
*                                         每种标志占用一个bit，可以多个标志组合
* @param[in]      name                    中断名，调试
* @param[in]      dev_id                  描述设备的结构体指针,在调用中断服务程序时，传给它
* @return         @li 0                   表示申请成功
*                 @li < 0                 错误码
* @notes          cat /proc/interrupts
* int request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id);
*/
	ret = request_irq(adc->irqno, adc_isr, IRQF_DISABLED, "adc", adc);
	if (ret < 0){
		goto err_request_irq;
	}
	
	adc_dev_init(adc);
	
// 5.1.2 初始化
	init_waitqueue_head(&adc->readq);
	
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
 * @brief          注销中断
 * @param[in]      irq                     中断号
 * @param[in]      dev_id                  描述设备的结构体指针
 * void free_irq(unsigned int irq, void *dev_id);
 */
	free_irq(adc->irqno, adc);
       
	
// 2.3 取消映射
	iounmap(adc->regs);
	
// 2.2 注销
	release_mem_region(adc->reg_res->start, resource_size(adc->reg_res));
	
/*
 * @brief						禁止时钟
 * @param[in]				clk													时钟结构
 * void clk_disable(struct clk *clk);
 */
	clk_disable(adc->clk);
 
 
/*
 * @brief						释放时钟
 * @param[in]				clk													时钟结构
 * void clk_put(stuct clk *clk);
 */
	clk_put(adc->clk);

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
