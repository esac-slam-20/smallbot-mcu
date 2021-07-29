# Project-SmallBot-MCU

小车计划主控代码

## 编译

首先安装 [RISC-V GCC](https://xpack.github.io/riscv-none-embed-gcc/)，将其添加到你的环境变量中。

然后：
``` bash
make
```

## 烧录

推荐使用USB DFU方式烧录

对于Linux系统，请下载[gd32vflash](https://dl.sipeed.com/fileList/LONGAN/platformio/dl-packages/tool-gd32vflash-v0.1.0-linux.tar.gz)。

```
/etc/udev/rules.d/99-platformio-udev.rules

ATTRS{idVendor}=="28e9", ATTRS{idProduct}=="0189", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"

```

然后 `dfu-util -d 28e9:0189 -a 0 --dfuse-address 0x08000000:leave -D _build/main.hex`

