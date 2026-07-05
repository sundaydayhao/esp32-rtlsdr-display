# ESP32-S31-Korvo-1 RTL-SDR 瀑布图

这是一个 ESP-IDF 项目，用于通过 ESP32-S31-Korvo-1 的 USB OTG Type-A 主机接口连接 RTL2832U RTL-SDR 接收棒，并在 RGB LCD 接口上绘制 FFT 瀑布图。

默认 GPIO 映射遵循 ESP32-S31-Korvo-1 官方资料：RGB LCD 数据线使用 GPIO8-19 和 GPIO33-36，PCLK/DE/HSYNC/VSYNC 使用 GPIO40/43/44/45，USB 主机接口通过开发板上的 Type-A 连接器引出。

## 构建

```powershell
& C:\esp\v6.0.2\esp-idf\export.ps1
idf.py --preview set-target esp32s31
idf.py --preview menuconfig
idf.py --preview build
idf.py --preview flash monitor
```

ESP32-S31 支持目前通过 ESP-IDF preview target 暴露，因此本项目的所有 `idf.py` 命令都需要带上 `--preview`。本项目已使用 `C:\esp\v6.0.2\esp-idf` 下的 ESP-IDF v6.2.0 以及 `riscv32-esp-elf` 工具链 `esp-16.1.0_20260609` 验证。

重要配置项位于 `RTL-SDR Waterfall` 菜单下：

- 显示用中心频率和 RTL2832U 采样率。
- FFT 点数和瀑布图尺寸。
- FM 接收模式、PCM 采样率、音量，以及 Korvo-1 扬声器 codec GPIO。
- LCD RGB 时序和 GPIO 映射。
- LVGL 绘制缓冲区和任务大小。
- I2C 触摸输入。默认 ESP32-S31-Korvo-1 LCD 子板配置使用 SDA GPIO0、SCL GPIO1，以及 GT911 地址 0x14。固件也会探测 FT5x06 兼容控制器；如果使用其他 LCD 子板，请根据实际硬件设置 SDA/SCL/INT/RST 和坐标轴变换选项。
- 可选的 RTL2832U 基带初始化。

首次硬件调试时建议使用默认的低采样率。只有在设备枚举成功、bulk 读取连续稳定、LCD 刷新正常之后，再提高采样率。

## 当前范围

本项目目前验证了 ESP-IDF 路径的完整链路：USB Host 枚举、RTL2832U bulk IN 传输、IQ 缓冲、FFT 处理，以及 RGB LCD 瀑布图绘制。如果没有可用的 RTL-SDR 数据流，程序会绘制合成瀑布图，便于独立测试 LCD 路径。

显示部分现在基于 LVGL，并复刻了 Linux/Qt `rtlsdr-display` 项目的触摸导航。主页提供 ADS-B、FM、Waterfall、Sensors、IQ Record 和 Scanner 功能入口。Waterfall 页面已经接入现有实时 FFT 管线，并支持通过触摸选择标记频率。FM 页面复用同一路 RTL-SDR IQ 数据流进行 WFM/NFM 解调，显示解码后的音频波形、音频电平和输出状态，并可调整共享中心频率。其余功能页已经作为 LVGL 导航目标和状态/配置占位存在，后续还需要实现对应的 ESP32 侧解码、存储或扫描逻辑。

瀑布图布局参考 ESP32-DIV 的频谱页面：滚动瀑布图上方绘制实时 FFT 概览，水平刻度标出采样跨度的左边缘、中心和右边缘，信号强度使用从蓝/紫到黄/白的 RGB565 调色板。FFT bin 缩放到 LCD 宽度时，每个输出像素列取覆盖范围内的峰值，从而保留窄带峰值。

Waterfall 页面现在可以在运行时修改中心频率。ESP32 侧 RTL-SDR 任务会把这些请求应用到受支持的 Fitipower FC0012/FC0013 和 Rafael R820T/R820T2/R828D 调谐器，然后继续通过同一条 FFT 管线传输 IQ 数据。其他调谐器系列仍需要单独实现驱动路径，才能在该目标板上调谐。

FM 接收器遵循桌面版 `rtlsdr-display` 的解调结构：正交相位差 FM 检波、FIR 音频低通滤波、抽取到 PCM、直流阻断，以及 WFM 去加重。ESP32-S31-Korvo-1 板载扬声器路径默认启用 FM 音频输出：ES8389 通过 I2C GPIO0/GPIO1 控制，使用 I2S SCLK GPIO3、LRCLK GPIO4、DOUT GPIO5 接收 16-bit 立体声 PCM，并通过 PA 使能 GPIO7 驱动 NS4150B 扬声器功放。解调器仍然输出单声道；扬声器后端会把采样复制到左右声道。

部分 RGB LCD 子板在 RGB 数据流可见之前，还需要通过 ESP32-S31-Korvo-1 的 3-wire LCD SPI 引脚完成控制器初始化。本项目已经实现 RGB 数据和时序路径；如果背光亮起但没有图像，请添加对应子板的 panel-init 表。
