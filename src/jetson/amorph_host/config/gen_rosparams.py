#!/usr/bin/env python3

import sys
import yaml


def infer_cpp_type(value, key=None):
    if isinstance(value, bool):
        return "bool"
    elif isinstance(value, int):
        return "int"
    elif isinstance(value, float):
        return "double"
    elif isinstance(value, str):
        return "std::string"
    elif isinstance(value, list):
        if all(isinstance(x, float) for x in value):
            return "std::vector<double>"
        elif all(isinstance(x, int) for x in value):
            return "std::vector<int>"
        elif all(isinstance(x, str) for x in value):
            return "std::vector<std::string>"
        else:
            print("ERROR: Unsupported list element type")
            exit(1)
    elif isinstance(value, dict):
        if key is None:
            return "NestedParams"
        return f"{key.capitalize()}Params"
    else:
        print("ERROR: Unsupported type")
        exit(1)


# 递归生成嵌套匿名结构体，子结构体直接嵌在父结构体内部
def parse_yaml_to_struct(data, struct_name="RosParams", indent=0):
    lines = [f"{'  '*indent}struct {struct_name} {{"]
    for key, value in data.items():
        if isinstance(value, dict):
            lines.append(f"{'  '*(indent+1)}struct {{")
            sublines = parse_yaml_to_struct(
                value, struct_name="", indent=indent+2)
            # 去掉最外层struct {None} { 和最后的 }
            sublines = sublines.split('\n')[1:-1]
            lines.extend(sublines)
            lines.append(f"{'  '*(indent+1)}}} {key};")
        else:
            cpp_type = infer_cpp_type(value, key)
            lines.append(f"{'  '*(indent+1)}{cpp_type} {key};")
    lines.append(f"{'  '*indent}}};")
    return '\n'.join(lines)


if __name__ == "__main__":
    argc, argv = len(sys.argv), sys.argv
    yaml_path = None
    if argc == 1:
        yaml_path = "settings.yaml"
    elif argc == 2:
        yaml_path = argv[1]
    else:
        print("ERROR: Invalid arguments")
        exit(1)
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    # 兼容ros__parameters嵌套
    if isinstance(data, dict) and len(data) == 1:
        data = list(data.values())[0].get(
            "ros__parameters", data[list(data.keys())[0]])
    struct_code = parse_yaml_to_struct(data)
    print(struct_code)