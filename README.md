# 编译工程
```bash
cd BSP/stm32f1
# cd BSP/stm32f4
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE="cmake/gcc-arm-none-eabi.cmake" -Bbuild/Debug -G Ninja
cmake --build build/Debug
```