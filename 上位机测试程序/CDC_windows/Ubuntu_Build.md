# USB CDC 上位机程序 Ubuntu 编译说明

本文记录本次为了让 `CDC_windows` 上位机测试程序能在 Ubuntu/Linux 下编译运行所做的修改，以及在 Ubuntu 上生成可执行文件的方法。

## 修改内容

### 1. `main.cpp`

原程序主要面向 Windows，包含以下 Windows 专用内容：

```cpp
#include <windows.h>
Sleep(10);
system("pause");
```

这些内容在 Ubuntu 下不能直接编译。本次已改为跨平台写法：

```cpp
#include <chrono>
#include <thread>

std::this_thread::sleep_for(std::chrono::milliseconds(10));
```

`system("pause")` 只在 Windows 下保留，Linux 下不会执行。

### 2. 串口名称处理

Windows 下仍然可以输入 `3` 或 `COM3`，程序会打开 `COM3`。

Ubuntu/Linux 下默认打开：

```bash
/dev/ttyACM0
```

也可以运行时指定串口：

```bash
./MySerialApp /dev/ttyACM0
./MySerialApp /dev/ttyUSB0
./MySerialApp ttyACM1
./MySerialApp 0
```

其中 `0` 会被转换为 `/dev/ttyACM0`。

### 3. `CMakeLists.txt`

原 CMake 配置固定链接了 Windows 专用库：

```cmake
setupapi
```

Ubuntu 下没有这个库，所以已删除。

同时设置：

```cmake
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
```

这样 `CSerialPort` 会以静态库方式链接进 `MySerialApp`，生成的程序更方便拷贝到 Ubuntu 上运行。

注意：Linux 文件名区分大小写，CMake 顶层文件名应为：

```text
CMakeLists.txt
```

不能是 `CmakeLists.txt`。

## Ubuntu 编译步骤

先安装编译工具：

```bash
sudo apt update
sudo apt install -y build-essential cmake
```

进入上位机程序目录：

```bash
cd Three_Wheel_Omni/上位机测试程序/CDC_windows
```

生成构建目录并编译：

```bash
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux -j
```

编译成功后，可执行文件位置：

```bash
build-linux/Release/MySerialApp
```

## Ubuntu 运行方法

STM32 USB CDC 通常会识别为 `/dev/ttyACM0`：

```bash
./build-linux/Release/MySerialApp /dev/ttyACM0
```

如果是 USB 转串口模块，常见设备名是 `/dev/ttyUSB0`：

```bash
./build-linux/Release/MySerialApp /dev/ttyUSB0
```

可以用下面命令查看当前串口设备：

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

## 串口权限问题

如果运行时报权限错误，可以把当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

执行后需要注销并重新登录，或者重启 Ubuntu。

临时测试也可以使用：

```bash
sudo ./build-linux/Release/MySerialApp /dev/ttyACM0
```

但长期建议使用 `dialout` 组权限。

## 当前环境验证情况

当前这台 Windows 环境没有配置可用的 C++ 编译器，也没有可用的 WSL Ubuntu，所以这里无法直接生成 Linux ELF 可执行文件。

代码和 CMake 配置已经按 Ubuntu 编译方式修改，需在 Ubuntu 机器或 WSL Ubuntu 中执行上面的编译命令生成最终可执行程序。
