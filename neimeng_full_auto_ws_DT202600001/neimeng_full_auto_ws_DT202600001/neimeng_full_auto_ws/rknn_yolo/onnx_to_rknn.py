#!/usr/bin/env python3
"""独立 ONNX -> RKNN 工具。默认匹配本项目 RK3588 / RGB / 640x640。

python3 onnx_to_rknn.py best.onnx -o models/my_truck.rknn
python3 onnx_to_rknn.py best.onnx -o models/my_truck_int8.rknn --quantize --dataset dataset.txt
需要 Linux 环境中的 rknn-toolkit2 和 onnx；不依赖 ROS、摄像头或项目模块。
"""

import argparse
import math
from pathlib import Path
import sys
import tempfile


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("onnx", type=Path, help="输入 ONNX 文件")
    parser.add_argument("-o", "--output", type=Path, help="输出 RKNN 文件，默认与 ONNX 同名")
    parser.add_argument("--target", default="rk3588", help="目标芯片，默认 rk3588")
    parser.add_argument("--size", type=int, default=640, help="静态正方形输入边长，默认 640")
    parser.add_argument("--mean", type=float, nargs=3, default=[0, 0, 0], metavar=("R", "G", "B"))
    parser.add_argument("--std", type=float, nargs=3, default=[255, 255, 255], metavar=("R", "G", "B"))
    parser.add_argument("--quantize", action="store_true", help="启用 INT8 量化，必须提供校准集")
    parser.add_argument("--dataset", type=Path, help="校准图片清单，每行一张；相对路径以清单目录为基准")
    parser.add_argument("--force", action="store_true", help="允许覆盖已有 RKNN 文件")
    parser.add_argument("--verbose", action="store_true", help="显示 Toolkit 详细日志")
    args = parser.parse_args(argv)
    if args.size <= 0:
        parser.error("--size 必须为正数")
    if not all(math.isfinite(x) for x in args.mean + args.std) or any(x <= 0 for x in args.std):
        parser.error("mean/std 必须为有限数值，std 必须大于 0")
    if args.quantize != (args.dataset is not None):
        parser.error("--quantize 和 --dataset 必须一起使用")
    return args


def inspect_onnx(path, size):
    import onnx

    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    initializers = {item.name for item in model.graph.initializer}
    inputs = [item for item in model.graph.input if item.name not in initializers]
    if len(inputs) != 1:
        raise ValueError("本工具面向项目的单输入检测模型，不支持多输入模型")

    def shape(item):
        return [dim.dim_value if dim.HasField("dim_value") else dim.dim_param or "?"
                for dim in item.type.tensor_type.shape.dim]

    actual = shape(inputs[0])
    print("ONNX 输入:", inputs[0].name, actual)
    expected = [1, 3, size, size]
    if actual != expected:
        raise ValueError("本项目需要静态 NCHW 输入 {}，实际 {}。请重新导出固定尺寸、batch=1 的 ONNX；"
                         "--size 只校验尺寸，不会修改模型。".format(expected, actual))
    for item in model.graph.output:
        print("ONNX 输出:", item.name, shape(item))
    print("请核对类别顺序和输出语义：当前检测器仅实现 YOLOv8 单输出或特定 9 输出解码。")
    if len(model.graph.output) not in (1, 9):
        print("警告：此输出数量无法直接接入当前 post_process，需要适配后处理。", file=sys.stderr)


def prepare_dataset(source, destination):
    source = source.expanduser().resolve()
    paths = []
    for number, line in enumerate(source.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        path = Path(line).expanduser()
        if not path.is_absolute():
            path = source.parent / path
        path = path.resolve()
        if not path.is_file():
            raise ValueError("校准清单第 {} 行文件不存在: {}".format(number, path))
        if any(char.isspace() for char in str(path)):
            raise ValueError("Toolkit 校准路径请勿包含空白字符: {}".format(path))
        if path.suffix.lower() not in (".jpg", ".jpeg", ".png", ".bmp"):
            raise ValueError("本工具的校准清单仅支持 jpg/jpeg/png/bmp 图片: {}".format(path))
        paths.append(str(path))
    if not paths:
        raise ValueError("校准清单没有有效图片")
    destination.write_text("\n".join(paths) + "\n", encoding="utf-8")
    print("校准图片数量:", len(paths))


def check_ret(stage, ret):
    if ret != 0:
        raise RuntimeError("{} 失败，返回码 {}；请查看上方 Toolkit 日志".format(stage, ret))


def convert(args):
    source = args.onnx.expanduser().resolve()
    output = (args.output or source.with_suffix(".rknn")).expanduser().resolve()
    if not source.is_file() or source.suffix.lower() != ".onnx":
        raise ValueError("请输入存在的 .onnx 文件: {}".format(source))
    if output.suffix.lower() != ".rknn":
        raise ValueError("输出文件必须使用 .rknn 后缀")
    if output.exists() and not args.force:
        raise ValueError("输出已存在；请更换名称或指定 --force: {}".format(output))
    inspect_onnx(source, args.size)
    from rknn.api import RKNN

    output.parent.mkdir(parents=True, exist_ok=True)
    # 在同一文件系统暂存，只有全部导出成功后才替换目标模型。
    with tempfile.TemporaryDirectory(prefix="onnx_to_rknn_", dir=str(output.parent)) as directory:
        temp_dir = Path(directory)
        dataset = None
        if args.quantize:
            dataset = temp_dir / "dataset.txt"
            prepare_dataset(args.dataset, dataset)
        rknn = RKNN(verbose=args.verbose)
        try:
            print("[1/4] 配置目标芯片:", args.target)
            check_ret("config", rknn.config(target_platform=args.target,
                                            mean_values=[args.mean], std_values=[args.std]))
            print("[2/4] 加载 ONNX")
            check_ret("load_onnx", rknn.load_onnx(model=str(source)))
            print("[3/4] 构建模型，量化:", args.quantize)
            build_args = {"do_quantization": args.quantize}
            if dataset is not None:
                build_args["dataset"] = str(dataset)
            check_ret("build", rknn.build(**build_args))
            print("[4/4] 导出 RKNN")
            temporary_output = temp_dir / "model.rknn"
            check_ret("export_rknn", rknn.export_rknn(str(temporary_output)))
            if not temporary_output.is_file() or temporary_output.stat().st_size == 0:
                raise RuntimeError("Toolkit 未生成有效的 RKNN 文件")
        finally:
            rknn.release()
        temporary_output.replace(output)
    print("转换成功:", output)
    print("仍需在目标板上验证模型加载、输出结构和检测精度。")


def main(argv=None):
    args = parse_args(argv)
    try:
        convert(args)
    except ImportError as exc:
        print("依赖加载失败: {}。请在受支持的 Linux/Python 环境安装 onnx 和 rknn-toolkit2；"
              "rknn-toolkit-lite2 只能推理，不能替代转换工具。".format(exc), file=sys.stderr)
        return 1
    except (ValueError, OSError, RuntimeError) as exc:
        print("转换失败:", exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
