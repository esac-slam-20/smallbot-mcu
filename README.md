# Project-SmallBot-MCU

小车计划主控代码

## 编译

首先安装 RISC-V GCC，[手动安装指南](https://xpack.github.io/riscv-none-embed-gcc/install/#manual-install)。

> 我们推荐使用手动安装方式安装，并将其添加到系统环境变量中。

然后：
``` bash
make
```

## 烧录

推荐使用USB DFU方式烧录。按住主控板上的BOOT0按钮，然后按下并松开Reset按钮，再松开Boot0按钮，即可进入DFU模式。

### Linux 下

对于Linux系统，请下载[gd32vflash](https://dl.sipeed.com/fileList/LONGAN/platformio/dl-packages/tool-gd32vflash-v0.1.0-linux.tar.gz)并解压。

添加下面的 udev 规则到`/etc/udev/rules.d/99-gd32vf103.rules`：
```
ATTRS{idVendor}=="28e9", ATTRS{idProduct}=="0189", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
```
添加完毕后，需要重新拔插你的USB设备。

对于预编译固件，使用以下命令行烧录：
```
dfu-util -d 28e9:0189 -a 0 --dfuse-address 0x08000000:leave -D main.bin
```

对于开发固件，使用以下命令行烧录：
```
make flash
```
### Windows 下

请到[官方下载页面](http://www.gd32mcu.com/cn/download?kw=DFU&lan=cn)，下载DFU Tool 和 DFU Driver。解压后先安装驱动，然后运行DFU程序烧录。
