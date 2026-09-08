# ESP-IDF 6.1 迁移记录

日期：2026-09-08。目标板：ESP32-S31-Korvo-1；芯片目标：`esp32s31`。

## 是否还需要 `--preview`

**需要。ESP-IDF v6.1 是正式的 SDK 版本，但它对 ESP32-S31 的支持仍属于预览。**

核实依据：

- [v6.1 发布说明](https://github.com/espressif/esp-idf/releases/tag/v6.1) 将 ESP32-S31 列为新增的 preview support。
- [v6.1 的 `constants.py`](https://github.com/espressif/esp-idf/blob/v6.1/tools/idf_py_actions/constants.py) 中，`esp32s31` 位于 `PREVIEW_TARGETS`，不在 `SUPPORTED_TARGETS`。
- 本机 `C:\esp\v6.1\esp-idf` 的 Git 版本为 `v6.1`，目标列表与上述源码一致。
- 本机 `tools/idf_py_actions/core_ext.py` 会检查预览目标，没有传入 `--preview` 就报错。

因此继续使用 `idf.py --preview build` 和 `idf.py --preview flash monitor`。部分入门文档和 SDK 构建完成后的通用烧录提示省略了该参数，应以所用版本的实际 CLI 实现为准。以后升级到其他 6.1.x 版本时重新检查目标列表，不能只根据目录名或版本号推断。

## 本次迁移范围

| 项目 | 迁移前 | 迁移后 |
| --- | --- | --- |
| SDK 路径 | 文档引用 `C:\esp\v6.0.2\esp-idf` | `C:\esp\v6.1\esp-idf` |
| 实际 SDK 记录 | `dependencies.lock` 与 `sdkconfig` 都记录 `6.2.0` | 使用本机 `v6.1` 重新生成 |
| 组件清单的 SDK 约束 | `idf: ">=6.0"` | `idf: "~6.1.0"`，即 `>=6.1.0` 且 `<6.2.0` |
| VS Code | 用户已将 `idf.currentSetup` 改为 v6.1 路径 | 保留该设置 |
| 构建参数 | `--preview` | 保留 |

这次是把此前基于 6.2.0 生成的工程配置迁移到指定的 6.1 发布版，而不只是替换一个目录字符串。`sdkconfig` 和 `dependencies.lock` 由 IDF/组件管理器更新，不手工伪造版本字段。

## Windows 环境与命令

本机最初只有 v6.1 源码，没有 `idf6.1_py3.11_env` Python 环境。使用官方脚本补齐依赖：

```powershell
& C:\esp\v6.1\esp-idf\install.ps1 esp32s31
```

安装完成后，在项目目录执行：

```powershell
$env:PYTHONUTF8 = '1'
& C:\esp\v6.1\esp-idf\export.ps1
idf.py --preview --version
# 切换 SDK 后执行一次；会清理旧构建缓存和托管组件缓存。
idf.py --preview fullclean
idf.py --preview build
```

后续日常构建只需激活环境并执行 `idf.py --preview build`。已有 `sdkconfig` 的目标就是 `esp32s31`，无需重复 `set-target`；若确需初始化/重置配置，使用 `idf.py --preview set-target esp32s31`。

烧录命令如下，`COMx` 需替换为实际串口：

```powershell
idf.py --preview -p COMx flash monitor
```

## 验证记录

已执行 `idf.py --preview fullclean` 后运行 `idf.py --preview build`，完整构建成功，退出码为 **0**。

| 验证项 | 结果 |
| --- | --- |
| SDK / 目标 | `ESP-IDF v6.1` / `esp32s31` |
| C/C++ 工具链 | `riscv32-esp-elf`，`esp-15.2.0_20251204`，GCC 15.2.0 |
| Python | 独立的 `idf6.1_py3.11_env`，Python 3.11.15 |
| 托管组件 | LVGL 9.5.0、USB 1.4.1、esp_codec_dev 1.5.11，保持原锁定版本 |
| 应用固件 | `build/esp32_rtlsdr_display.bin`，885,568 字节（`0xD8340`） |
| 应用分区 | 1 MiB，剩余 163,008 字节，构建工具显示约 16% |
| 启动与分区产物 | `build/bootloader/bootloader.bin` 和 `build/partition_table/partition-table.bin` 均生成成功 |
| 工程配置 | 对比迁移前后所有 `main/Kconfig.projbuild` 选项，值均保持一致 |
| 文档检查 | 本地 Markdown 链接有效；`git diff --check` 通过 |

当前业务 C 源码可以直接通过 v6.1 编译，无需为迁移修改接收、显示或解调逻辑。生成的 `sdkconfig` 移除了该 SDK 中不存在的旧配置项，并生成 v6.1 的平台配置项。构建描述文件确认使用的是 `C:/esp/v6.1/esp-idf`，不是旧构建缓存。

配置阶段有 SDK 内部的 Wi-Fi 私有头文件依赖警告，以及 NimBLE/FatFS 布尔默认值提示；它们没有导致构建失败。本次没有修改 SDK 源码来隐藏这些提示。

本次没有执行硬件烧录或运行测试。

构建验证不代替硬件运行验证。烧录后需要检查 USB RTL-SDR 枚举、LCD/触摸、FM 音频与页面切换，以及 ADS-B、扫描、传感器和录制的实际工作状态。
