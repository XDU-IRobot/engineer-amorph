#include <stdbool.h>

// 生成完py文件之后要把几个bool类型的bitfield成员改成c_ubyte类型，否则在Python中访问时会有问题

struct __attribute__((__packed__)) CartesianCustomControllerCommand {
  bool is_cartesian : 1;
  bool ok : 1;
  bool btn_a : 1;
  bool btn_b : 1;
  bool reserved_1 : 1;
  bool reserved_2 : 1;
  bool reserved_3 : 1;
  bool reserved_4 : 1;
  float translation_xyz[3];
  float rotation_wxyz[4];
};