## 修改了Fines_Serial，当前收发逻辑为椅夫轮椅通信协议
还需配置：端口地址、485通信地址、速度映射关系

```shell
source install/setup.bash

# 如果没有提供串口号，则默认使用 /dev/ttyUSB0
ros2 run fines_serial fines_serial -- <serial_port>
```
