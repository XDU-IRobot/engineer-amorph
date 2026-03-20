# T265 自定义控制器

## dependencies

- Ubuntu 20.04

- librealsense `2.50.0` `需要编译python binding`

  ```cmake
  cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=$(which python3)
  ```

- `pip install -r requirements.txt`

- uhubctl `sudo apt install -y uhubctl`

## 环境配置

1. 启用串口2：修改 `/boot/firmware/config.txt`，在末尾添加：`dtoverlay=uart2`，重启

2. 启用i2c：修改 `/boot/firmware/config.txt`，在末尾添加：`dtparam=i2c_arm=on`，重启

3. 配置自动加载内核模块：修改 `/etc/modules`，在末尾添加：`i2c-dev`，重启

4. 安装i2c工具并添加当前用户到i2c组：

    ```bash
    sudo apt update
    sudo apt install -y i2c-tools
    sudo usermod -aG i2c $USER
    ```

5. 检查屏幕是否连接好：

    ```
    pi@raspberrypi:~/engineer-amorph$ i2cdetect -y 1
        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
    10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
    30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- -- 
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
    60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
    70: -- -- -- -- -- -- -- --         
    ```