#!/usr/bin/env bash
# 在目标机上把 trtyolo 版 ONNX 构建成 TensorRT engine。
#
# engine 与 TensorRT 版本、GPU 架构强绑定，必须在要运行的那台机器上构建：
#   - Jetson Orin NX（JetPack 6.x / TensorRT 10.3）
#   - 开发机（TensorRT 11.1 / RTX 40 系）
#
# Segment 模型的自定义 mask 插件必须序列化进 engine，因此需要
# libcustom_plugins.so（由 amorph_vision_cpp 构建产出），否则反序列化会失败。
#
# 精度：TensorRT 11 移除了 --fp16（改为 strongly typed，精度由图决定），
# 脚本会探测本机 trtexec 是否支持 --fp16 再决定是否传，无需手工区分平台。
#
# 用法:
#   scripts/build_eu_engine.sh                        # 包内默认 ONNX / 输出路径
#   scripts/build_eu_engine.sh -i foo.onnx -o foo.engine
#   scripts/build_eu_engine.sh --no-fp16              # 需要更高精度时
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_DIR="${REPO_ROOT}/src/jetson/amorph_vision_cpp"

ONNX_PATH="${PKG_DIR}/resource/best-trtyolo.onnx"
ENGINE_PATH="${PKG_DIR}/resource/best.engine"
# custom_plugins 由 libs/CMakeLists.txt 构建，产物落在 <build>/libs/libcustom_plugins.so
PLUGIN_LIB="${REPO_ROOT}/build/amorph_vision_cpp/libs/libcustom_plugins.so"
TRTEXEC="$(command -v trtexec || true)"
FP16=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i | --onnx)
            ONNX_PATH="$2"
            shift 2
            ;;
        -o | --engine)
            ENGINE_PATH="$2"
            shift 2
            ;;
        -p | --plugins)
            PLUGIN_LIB="$2"
            shift 2
            ;;
        --trtexec)
            TRTEXEC="$2"
            shift 2
            ;;
        --no-fp16)
            FP16=0
            shift
            ;;
        -h | --help)
            sed -n '2,22p' "$0"
            exit 0
            ;;
        *)
            echo "未知参数: $1" >&2
            exit 1
            ;;
    esac
done

[[ -f "${ONNX_PATH}" ]] || {
    echo "找不到 ONNX: ${ONNX_PATH}" >&2
    echo "先运行 scripts/export_eu_engine.py 生成" >&2
    exit 1
}
[[ -n "${TRTEXEC}" && -x "${TRTEXEC}" ]] || {
    echo "找不到 trtexec，请用 --trtexec 指定（JetPack 下在 /usr/src/tensorrt/bin/trtexec）" >&2
    exit 1
}
[[ -f "${PLUGIN_LIB}" ]] || {
    echo "找不到 ${PLUGIN_LIB}" >&2
    echo "先构建本包：scripts/build.py" >&2
    exit 1
}
echo "trtexec      : ${TRTEXEC}"
echo "ONNX         : ${ONNX_PATH}"
echo "engine       : ${ENGINE_PATH}"
echo "custom 插件  : ${PLUGIN_LIB}"

ARGS=(
    "--onnx=${ONNX_PATH}"
    "--saveEngine=${ENGINE_PATH}"
    "--staticPlugins=${PLUGIN_LIB}"
    "--setPluginsToSerialize=${PLUGIN_LIB}"
)

if [[ ${FP16} -eq 1 ]]; then
    if "${TRTEXEC}" --help 2>&1 | grep -q -- '--fp16'; then
        ARGS+=("--fp16")
        echo "精度         : FP16"
    else
        # TensorRT 11+：strongly typed 网络，精度由 ONNX 决定，无法从命令行强制 FP16
        echo "精度         : 由 ONNX 决定（本机 trtexec 不支持 --fp16）"
    fi
fi

mkdir -p "$(dirname "${ENGINE_PATH}")"
"${TRTEXEC}" "${ARGS[@]}"

echo
echo "完成: ${ENGINE_PATH}"
echo "运行时用 -p engine_path:=${ENGINE_PATH} 指定"
