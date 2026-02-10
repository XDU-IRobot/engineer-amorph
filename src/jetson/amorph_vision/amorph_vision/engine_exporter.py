#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import shutil
from pathlib import Path

from ultralytics import YOLO # type: ignore
from ament_index_python.packages import get_package_share_directory


def check_trtexec_available() -> bool:
    """检查系统中是否可用 trtexec"""
    return shutil.which('trtexec') is not None


def export_to_onnx(model_path: str, output_path: str = "") -> str:
    """
    导出 YOLO 模型为 ONNX 格式
    
    Args:
        model_path: 输入模型路径（.pt 文件）
        output_path: 输出目录，如果为空则使用与输入相同的目录
    
    Returns:
        导出后的 ONNX 文件路径
    """
    print(f"Loading model from: {model_path}")
    
    if not os.path.exists(model_path):
        print(f"Error: Model file not found: {model_path}")
        sys.exit(1)
    
    try:
        model = YOLO(model_path)
        print("Model loaded successfully")
    except Exception as e:
        print(f"Error loading model: {e}")
        sys.exit(1)
    
    # 确定输出位置
    if not output_path:
        output_dir = os.path.dirname(model_path)
    else:
        output_dir = output_path
        os.makedirs(output_dir, exist_ok=True)
    
    print(f"Exporting to ONNX format...")
    print(f"  Output directory: {output_dir}")
    
    try:
        # 导出为 ONNX
        onnx_path = model.export(
            format='onnx',
            imgsz=640,
        )
        print(f"ONNX export successful!")
        print(f"ONNX model saved at: {onnx_path}")
        return str(onnx_path)
    except Exception as e:
        print(f"Error during ONNX export: {e}")
        sys.exit(1)


def export_via_trtexec(onnx_path: str, output_path: str = "", use_fp16: bool = True) -> str:
    """
    使用 trtexec 从 ONNX 文件导出 TensorRT engine
    
    Args:
        onnx_path: ONNX 模型路径
        output_path: 输出路径，如果为空则使用与 ONNX 相同的目录
        use_fp16: 是否使用 FP16 精度
    
    Returns:
        导出后的 engine 文件路径
    """
    if not os.path.exists(onnx_path):
        print(f"Error: ONNX file not found: {onnx_path}")
        sys.exit(1)
    
    # 确定输出位置
    if not output_path:
        output_dir = os.path.dirname(onnx_path)
        engine_name = Path(onnx_path).stem + '.engine'
        engine_path = os.path.join(output_dir, engine_name)
    else:
        engine_path = output_path
        os.makedirs(os.path.dirname(engine_path), exist_ok=True)
    
    print(f"Exporting to TensorRT engine via trtexec...")
    print(f"  Input ONNX: {onnx_path}")
    print(f"  Output engine: {engine_path}")
    print(f"  FP16 precision: {use_fp16}")
    
    try:
        # 构建 trtexec 命令
        cmd = [
            'trtexec',
            '--onnx=' + onnx_path,
            '--saveEngine=' + engine_path,
            '--useCudaGraph',
        ]
        
        if use_fp16:
            cmd.append('--fp16')
        
        print(f"Executing: {' '.join(cmd)}")
        print()
        
        # 执行 trtexec
        result = subprocess.run(cmd, check=True)
        
        if result.returncode == 0:
            print(f"\nTensorRT export successful!")
            print(f"Engine saved at: {engine_path}")
            return engine_path
        else:
            print(f"Error: trtexec failed with return code {result.returncode}")
            sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error during trtexec export: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


def export_to_tensorrt(model_path: str, output_path: str = "", use_fp16: bool = True, device: int = 0):
    """
    导出 YOLO 模型为 TensorRT engine格式
    
    优先使用 trtexec (如果可用)，否则使用 ultralytics 直接转换
    
    Args:
        model_path: 输入模型路径（.pt 文件）
        output_path: 输出路径，如果为空则使用与输入相同的目录
        use_fp16: 是否使用 FP16 精度加速
        device: GPU 设备 ID
    
    Returns:
        导出后engine文件路径
    """
    
    # 检查是否有 trtexec
    has_trtexec = check_trtexec_available()
    
    if has_trtexec:
        print("Found trtexec in system, will use ONNX -> TensorRT conversion path")
        print()
        # 方案 1: ONNX -> TensorRT (via trtexec)
        onnx_dir = os.path.dirname(model_path) if not output_path else os.path.dirname(output_path)
        onnx_path = export_to_onnx(model_path, onnx_dir)
        engine_path = export_via_trtexec(onnx_path, output_path, use_fp16)
        return engine_path
    else:
        print("trtexec not found, will use ultralytics direct conversion")
        print()
        # 方案 2: 直接用 ultralytics 转换
        return export_to_tensorrt_direct(model_path, output_path, use_fp16, device)


def export_to_tensorrt_direct(model_path: str, output_path: str = "", use_fp16: bool = True, device: int = 0) -> str:
    """
    使用 ultralytics 直接导出 TensorRT engine
    
    Args:
        model_path: 输入模型路径（.pt 文件）
        output_path: 输出路径，如果为空则使用与输入相同的目录
        use_fp16: 是否使用 FP16 精度加速
        device: GPU 设备 ID
    
    Returns:
        导出后engine文件路径
    """
    
    print(f"Loading model from: {model_path}")
    
    if not os.path.exists(model_path):
        print(f"Error: Model file not found: {model_path}")
        sys.exit(1)
    
    try:
        model = YOLO(model_path)
        print("Model loaded successfully")
    except Exception as e:
        print(f"Error loading model: {e}")
        sys.exit(1)
    
    # 确定输出位置
    if not output_path:
        output_dir = os.path.dirname(model_path)
    else:
        output_dir = os.path.dirname(output_path)
        os.makedirs(output_dir, exist_ok=True)
    
    print(f"Exporting to TensorRT format via ultralytics...")
    print(f"  Output directory: {output_dir}")
    print(f"  FP16 precision: {use_fp16}")
    print(f"  GPU device: {device}")
    
    try:
        # 导出为 TensorRT 引擎
        engine_path = model.export(
            format='engine',
            device=device,
            half=use_fp16,
            imgsz=640,
        )
        print(f"\nExport successful!")
        print(f"Engine saved at: {engine_path}")
        return str(engine_path)
    except Exception as e:
        print(f"Error during export: {e}")
        sys.exit(1)


def main():
    """命令行入口"""
    parser = argparse.ArgumentParser(
        description='Export YOLO model to TensorRT engine format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Export default model using best available method
  export_engine
  
  # Export with specific model path
  export_engine --model /path/to/model.pt
  
  # Export with custom output path
  export_engine --output /path/to/output_engine.engine
  
  # Force using ultralytics (skip trtexec)
  export_engine --force-direct
  
  # Use FP32 precision instead of FP16
  export_engine --fp32
        """
    )
    parser.add_argument(
        '--model',
        type=str,
        default='',
        help='Path to YOLO model (.pt file). If not provided, uses default model from package.'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='',
        help='Output path for the TensorRT engine. If not provided, uses same directory as input model.'
    )
    parser.add_argument(
        '--fp32',
        action='store_true',
        help='Use FP32 precision instead of FP16 (slower but more accurate)'
    )
    parser.add_argument(
        '--device',
        type=int,
        default=0,
        help='GPU device ID for ultralytics export (default: 0)'
    )
    parser.add_argument(
        '--force-direct',
        action='store_true',
        help='Force using ultralytics direct conversion (skip trtexec even if available)'
    )
    
    args = parser.parse_args()
    
    # 获取模型路径
    if args.model:
        model_path = args.model
    else:
        # 使用包中的默认模型
        try:
            pkg_share = get_package_share_directory('amorph_vision')
            model_path = os.path.join(pkg_share, 'models', '251120_epoch335.pt')
        except:
            print("Error: Could not find package 'amorph_vision'")
            print("Please specify model path with --model argument")
            sys.exit(1)
    
    print(f"=" * 50)
    print(f"YOLO Model TensorRT Engine Exporter")
    print(f"=" * 50)
    print()
    
    # 导出模型
    if args.force_direct:
        print("Forcing direct ultralytics conversion (--force-direct)")
        print()
        export_to_tensorrt_direct(
            model_path=model_path,
            output_path=args.output,
            use_fp16=not args.fp32,
            device=args.device
        )
    else:
        export_to_tensorrt(
            model_path=model_path,
            output_path=args.output,
            use_fp16=not args.fp32,
            device=args.device
        )


if __name__ == '__main__':
    main()
