#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwd2png.py —— 把 X11 的 .xwd 截图转成 PNG。

为什么需要它：这台机器上没有 scrot / import / ffmpeg，只有 xwd，
而 PIL 不认 XWD 格式。所以这里手写一个最小 XWD 解析器（零第三方依赖，
只用 numpy + PIL）。

XWD 布局（X Window Dump，file_version 7）：
    [0:100]                25 个 uint32 头字段
    [100:header_size]      窗口名字符串（补零）
    [header_size:+n*12]    ncolors 个 12 字节色表项
    [此后]                 像素数据，每行 bytes_per_line 字节

注意：头字段的字节序 = pixmap_byteorder 字段本身，所以要先试探。
      x86 上一般是 LSBFirst（小端）。

用法：
    python3 xwd2png.py input.xwd output.png
"""

import struct
import sys

import numpy as np
from PIL import Image

FIELDS = [
    "header_size", "file_version", "pixmap_format", "pixmap_depth",
    "pixmap_width", "pixmap_height", "xoffset", "byte_order",
    "bitmap_unit", "bitmap_bit_order", "bitmap_pad", "bits_per_pixel",
    "bytes_per_line", "visual_class",
    "red_mask", "green_mask", "blue_mask",
    "bits_per_rgb", "colormap_entries", "ncolors",
    "window_width", "window_height", "window_x", "window_y", "window_bdrwidth",
]


def parse_header(buf: bytes) -> dict:
    """头字段按小端读；若宽高不合常理再按大端重读。"""
    for endian in ("<", ">"):
        vals = struct.unpack(endian + "25I", buf[:100])
        h = dict(zip(FIELDS, vals))
        if 0 < h["pixmap_width"] <= 16384 and 0 < h["pixmap_height"] <= 16384 \
                and h["file_version"] == 7:
            h["_endian"] = endian
            return h
    raise ValueError("不是合法的 XWD 文件（头字段解析失败）")


def channel_from_mask(values: np.ndarray, mask: int) -> np.ndarray:
    """按位掩码从打包像素值里抽出某个通道，并归一到 0-255。"""
    if mask == 0:
        return np.zeros(values.shape, dtype=np.uint8)
    shift = (mask & -mask).bit_length() - 1
    width = bin(mask).count("1")
    out = (values & np.uint32(mask)) >> np.uint32(shift)
    if width == 0:
        return np.zeros(values.shape, dtype=np.uint8)
    full = (1 << width) - 1
    return (out.astype(np.float64) * 255.0 / full).round().astype(np.uint8)


def convert(src: str, dst: str) -> None:
    with open(src, "rb") as f:
        data = f.read()
    if len(data) < 100:
        raise ValueError("文件太小，不是 XWD")

    h = parse_header(data)
    w = int(h["pixmap_width"])
    hgt = int(h["pixmap_height"])
    bpl = int(h["bytes_per_line"])
    bpp = int(h["bits_per_pixel"])
    ncolors = int(h["ncolors"])
    pix_off = int(h["header_size"]) + ncolors * 12

    need = pix_off + bpl * hgt
    if len(data) < need:
        raise ValueError(f"数据不足：需要 {need} 字节，实际 {len(data)}")

    endian = h["_endian"]
    print(f"  XWD: {w}x{hgt}  depth={h['pixmap_depth']} bpp={bpp} "
          f"bytes/line={bpl} endian={'小端' if endian == '<' else '大端'} "
          f"ncolors={ncolors} class={h['visual_class']}")

    raw = np.frombuffer(data, dtype=np.uint8,
                        count=bpl * hgt, offset=pix_off).reshape(hgt, bpl)

    if bpp == 32:
        px = raw[:, : w * 4].reshape(hgt, w, 4)
        # 按字节序把 4 个 uint8 拼成 uint32
        if endian == "<":
            v = (px[:, :, 0].astype(np.uint32)
                 | (px[:, :, 1].astype(np.uint32) << 8)
                 | (px[:, :, 2].astype(np.uint32) << 16)
                 | (px[:, :, 3].astype(np.uint32) << 24))
        else:
            v = (px[:, :, 3].astype(np.uint32)
                 | (px[:, :, 2].astype(np.uint32) << 8)
                 | (px[:, :, 1].astype(np.uint32) << 16)
                 | (px[:, :, 0].astype(np.uint32) << 24))
        r = channel_from_mask(v, int(h["red_mask"]))
        g = channel_from_mask(v, int(h["green_mask"]))
        b = channel_from_mask(v, int(h["blue_mask"]))
    elif bpp == 24:
        px = raw[:, : w * 3].reshape(hgt, w, 3)
        if endian == "<":
            r, g, b = px[:, :, 2], px[:, :, 1], px[:, :, 0]
        else:
            r, g, b = px[:, :, 0], px[:, :, 1], px[:, :, 2]
    elif bpp == 16:
        px = raw[:, : w * 2].reshape(hgt, w, 2)
        if endian == "<":
            v = px[:, :, 0].astype(np.uint32) | (px[:, :, 1].astype(np.uint32) << 8)
        else:
            v = px[:, :, 1].astype(np.uint32) | (px[:, :, 0].astype(np.uint32) << 8)
        r = channel_from_mask(v, 0xF800)
        g = channel_from_mask(v, 0x07E0)
        b = channel_from_mask(v, 0x001F)
    else:
        raise ValueError(f"暂不支持 {bpp} bpp")

    rgb = np.dstack([r, g, b])
    Image.fromarray(rgb, "RGB").save(dst)
    print(f"  已写入 {dst}")


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    convert(sys.argv[1], sys.argv[2])
    return 0


if __name__ == "__main__":
    sys.exit(main())
