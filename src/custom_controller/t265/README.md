# T265 自定义控制器

## dependencies

- Ubuntu 20.04

- librealsense `2.50.0` `需要编译python binding`

  ```cmake
  cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=$(which python3)
  ```

- `pip install -r requirements.txt`
