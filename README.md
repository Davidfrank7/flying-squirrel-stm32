# 编译

```
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -Bbuild/Debug -G Ninja
cmake --build build/Debug
```

# 调试和烧录
寻找build/下的elf文件，修改launch.json中的"executable"为该文件的路径，然后按F5即可开始调试。