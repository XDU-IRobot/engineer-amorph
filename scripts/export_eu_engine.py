#!/usr/bin/env python3
"""把 best.pt 转成 TensorRT-YOLO 可用、且带 TensorRT 插件的 ONNX。

产物：`resource/best-trtyolo.onnx`（走 LFS 提交）。engine 与 TensorRT 版本、
GPU 架构强绑定，不在这里构建也不进仓库，见 scripts/build_eu_engine.sh。

支持的导出器有两代，接口不同，脚本会自动选择：

- `trtyolo-export`（1.x）：输入直接是 .pt，一步产出。
- `trtyolo-export`（2.x）：输入是普通 ONNX，需先由 ultralytics 导出。
  注意 **2.0.1 与 TensorRT-YOLO 6.4 不兼容**——导出的 mask 系数 Gather 维度
  错误（5250×1×32 vs 期望的 20×32×…），TensorRT 解析时会在 MatMul 处
  报维度不匹配。脚本会拒绝 2.x 并提示降级。

用法:
    .venv/bin/python scripts/export_eu_engine.py
    .venv/bin/python scripts/export_eu_engine.py --model best.pt --imgsz 640
"""

import argparse
import importlib.metadata
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = REPO_ROOT / "src/jetson/amorph_vision_cpp/resource/best.pt"
DEFAULT_OUTPUT_NAME = "best-trtyolo.onnx"

# TensorRT-YOLO 6.4 的 C++ 后处理固定按 5 个输出张量取值
# （num_dets / det_boxes / det_scores / det_classes / det_masks）。
EXPECTED_OUTPUTS = ["num_dets", "det_boxes", "det_scores", "det_classes", "det_masks"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="训练好的 .pt 权重")
    parser.add_argument("--outdir", type=Path, default=None, help="输出目录，默认与权重同目录")
    parser.add_argument("--imgsz", type=int, default=640, help="导出分辨率（需与 engine 输入一致）")
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset（仅 2.x 导出器使用）")
    parser.add_argument("--conf-thres", type=float, default=0.25, help="写进 engine 的置信度阈值")
    parser.add_argument("--iou-thres", type=float, default=0.45, help="写进 engine 的 NMS IoU 阈值")
    parser.add_argument("--max-dets", type=int, default=20, help="engine 最多输出多少个目标")
    parser.add_argument("--overwrite", action="store_true", help="覆盖已存在的产物")
    return parser.parse_args()


def exporter_version():
    try:
        return importlib.metadata.version("trtyolo-export")
    except importlib.metadata.PackageNotFoundError:
        return None


def find_exporter(name: str) -> str:
    path = shutil.which(name)
    if path is not None:
        return path
    candidate = Path(sys.executable).parent / name
    if candidate.is_file():
        return str(candidate)
    raise SystemExit(f"找不到 {name} 可执行文件")


def export_with_v1(model_path: Path, outdir: Path, args: argparse.Namespace) -> Path:
    """trtyolo-export 1.x：从 .pt 一步导出。"""
    exporter = find_exporter("trtyolo")
    print("[1/1] trtyolo export 从 .pt 一步导出（含 TensorRT 插件）")
    with tempfile.TemporaryDirectory() as tmpdir:
        subprocess.run(
            [
                exporter, "export",
                "-v", "ultralytics",
                "-w", str(model_path),
                "-o", tmpdir,
                "--imgsz", str(args.imgsz),
                "--max_boxes", str(args.max_dets),
                "--conf_thres", str(args.conf_thres),
                "--iou_thres", str(args.iou_thres),
            ],
            check=True,
        )
        produced = sorted(Path(tmpdir).glob("*.onnx"))
        if len(produced) != 1:
            raise SystemExit(f"预期 1 个 ONNX，实际得到 {[p.name for p in produced]}")
        output = outdir / DEFAULT_OUTPUT_NAME
        shutil.move(str(produced[0]), output)
    return output


def export_with_v2(model_path: Path, outdir: Path, args: argparse.Namespace) -> Path:
    """trtyolo-export 2.x：先 ultralytics 导出，再改写图。"""
    from ultralytics import YOLO

    exporter = find_exporter("trtyolo-export")
    print("[1/2] ultralytics 导出普通 ONNX")
    model = YOLO(str(model_path))
    if getattr(model, "task", None) != "segment":
        raise SystemExit(f"{model_path} 不是实例分割模型（task={getattr(model, 'task', None)}）")

    with tempfile.TemporaryDirectory() as tmpdir:
        raw = Path(model.export(format="onnx", imgsz=args.imgsz, opset=args.opset, simplify=False))
        staged = Path(tmpdir) / raw.name
        shutil.move(str(raw), staged)

        output = outdir / DEFAULT_OUTPUT_NAME
        print(f"[2/2] trtyolo-export 注入 TensorRT 插件 -> {output}")
        subprocess.run(
            [
                exporter,
                "-i", str(staged),
                "-o", str(output),
                "--conf-thres", str(args.conf_thres),
                "--iou-thres", str(args.iou_thres),
                "--max-dets", str(args.max_dets),
            ],
            check=True,
        )
    return output


def validate(path: Path, max_dets: int) -> None:
    """校验输出张量布局与 TensorRT-YOLO 6.4 的 C++ 后处理约定一致。"""
    import onnx

    graph = onnx.load(str(path)).graph
    names = [output.name for output in graph.output]
    if names != EXPECTED_OUTPUTS:
        raise SystemExit(f"输出张量不符合预期。\n  期望: {EXPECTED_OUTPUTS}\n  实际: {names}")

    def dims(value_info) -> list:
        # trtyolo 导出器有时把常量维度写成 dim_param（字符串），两种都要认
        result = []
        for dim in value_info.type.tensor_type.shape.dim:
            if dim.HasField("dim_value"):
                result.append(dim.dim_value)
            elif dim.dim_param.isdigit():
                result.append(int(dim.dim_param))
            else:
                result.append(dim.dim_param or "?")
        return result

    masks = next(o for o in graph.output if o.name == "det_masks")
    mask_dims = dims(masks)
    if len(mask_dims) != 4 or mask_dims[1] != max_dets:
        raise SystemExit(f"det_masks 形状异常: {mask_dims}（期望 [*, {max_dets}, H, W]）")

    plugins = [node.op_type for node in graph.node if node.op_type.endswith("_TRT")]
    if "ROIAlign_TRT" not in plugins:
        raise SystemExit(f"缺少 ROIAlign_TRT 插件，插件列表: {plugins}")

    print(f"校验通过: 输出 {names}, det_masks {mask_dims}, 插件 {plugins}")


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        raise SystemExit(f"找不到权重文件: {args.model}")

    outdir = args.outdir or args.model.parent
    outdir.mkdir(parents=True, exist_ok=True)
    output = outdir / DEFAULT_OUTPUT_NAME

    version = exporter_version()
    if version is None:
        raise SystemExit("未安装 trtyolo-export：pip install 'trtyolo-export<2.0'")

    if output.exists() and not args.overwrite:
        print(f"复用已存在的产物: {output}（加 --overwrite 可强制重导）")
        validate(output, args.max_dets)
        return 0

    print(f"trtyolo-export 版本: {version}")
    if version.startswith("2."):
        raise SystemExit(
            f"trtyolo-export {version} 与 TensorRT-YOLO 6.4 不兼容：导出的 mask 系数\n"
            "Gather 维度错误，TensorRT 解析时会报 MatMul 维度不匹配。\n"
            "请降级：pip install 'trtyolo-export<2.0'")
    if version.startswith("1."):
        result = export_with_v1(args.model, outdir, args)
    else:
        result = export_with_v2(args.model, outdir, args)

    validate(result, args.max_dets)

    print()
    print(f"完成: {result}")
    print("注意：置信度阈值在导出时固化（默认 0.25），")
    print("      运行时 conf_threshold 只做二次过滤，不能低于这个值。")
    print("下一步：在目标机上运行 scripts/build_eu_engine.sh 生成 engine。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
