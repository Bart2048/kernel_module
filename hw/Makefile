# 仅仅打印
$(info KERNELRELEASE=$(KERNELRELEASE))

# 根据KERNELRELEASE(主Makefile调用导出的变量)是否有值判断是被make命令解析执行，还是被内核的主Makefile调用
ifeq ($(KERNELRELEASE),)
# 被make命令解析执行


# $(shell pwd) 调用Makefile的shell函数执行pwd命令
PWD := $(shell pwd)

# 内核的路径
KERNDIR := /home/linux/work/linux/kernel/fsc100/linux-2.6.35-farsight

all: 
# -C $(KERNDIR)   做任何事情前，切换make的当前路径到内核源代码
# M=$(PWD)        告诉内核主Makefile，仅仅只编译指定路径下的模块
# modules         只编译模块，不编译内核(zImage)
	$(MAKE) -C $(KERNDIR) M=$(PWD) modules
	
clean:
# 清楚模块生成的一切文件
	$(MAKE) -C $(KERNDIR) M=$(PWD) clean
	
else

# 被内核的主Makefile调用, 编译模块
obj-m += adc.o

endif
