# kernel_module
+ 设备驱动内核模块模板

+ 以设备ADC为例添加设备驱动模块

## 驱动设计流程:
module -> bus -> char -> mknod -> resource -> hardware -> noblock/signal
