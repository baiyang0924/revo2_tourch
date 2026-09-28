# -*- coding: utf-8 -*-
"""极简 YAML 读取器（标准库实现，零依赖）

为什么需要它：
    项目里的手势库和动作序列用 YAML 写（可读性好、适合人工调参）。
    但 `pyyaml` 在 Windows 上不一定是预装的。为了「拉下来就能跑」，
    这里提供一个只解析**本仓库用到的 YAML 子集**的兜底实现。

优先级：装了 pyyaml 就用 pyyaml（正确性优先），没装才用本模块兜底。

支持的语法子集（够用且经过对照测试）：
    - 缩进嵌套的映射
    - 列表（`- ` 开头的行）
    - 列表项是映射（`- key: value` + 后续同缩进的键）
    - 行内列表（`[a, b, c]`）
    - 标量：整数 / 浮点 / true / false / null / 字符串
    - 单行注释（`#` 在行首或空格之后）
    - 单双引号字符串
    - 块标量：`|` 保留换行 / `>` 折叠换行（含 `-` `+` 收尾修饰）

明确不支持（本仓库也不需要）：
    锚点与别名（`&` `*`）、复杂键、YAML 标签（`!!`）、
    多文档（`---` 分隔）、块标量显式缩进指示数字（如 `|2`）。

如果以后配置变复杂了，直接 `pip install pyyaml` 即可，本模块会自动让位。
"""

from __future__ import annotations

import os
import re
from typing import Any

__all__ = ["load", "loads", "have_pyyaml", "MiniYamlError"]


class MiniYamlError(ValueError):
    """YAML 子集解析失败。"""


# ══════════════════════════════════════════════════════════════
def have_pyyaml() -> bool:
    try:
        import yaml  # noqa: F401

        return True
    except Exception:
        return False


# ══════════════════════════════════════════════════════════════
# 标量
# ══════════════════════════════════════════════════════════════
_NUM_RE = re.compile(r"^[-+]?(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?$")
_INT_RE = re.compile(r"^[-+]?\d+$")


_ESCAPES = {"n": "\n", "t": "\t", "r": "\r", '"': '"', "'": "'", "\\": "\\", "0": "\0"}


def _unescape_double(inner: str) -> str:
    """按顺序扫描解析双引号字符串里的转义。

    注意必须**从左到右逐字符**处理，不能像早前那样用连续的
    str.replace() —— 先替换 `\\n` 再替换 `\\\\` 会把文本里真实存在的
    「反斜杠 + n」错误地变成一个换行符。块标量会把换行编码成 `\\n`，
    这个顺序问题就变成了实际错误。
    """
    out: list[str] = []
    i = 0
    n = len(inner)
    while i < n:
        ch = inner[i]
        if ch == "\\" and i + 1 < n:
            nxt = inner[i + 1]
            if nxt in _ESCAPES:
                out.append(_ESCAPES[nxt])
                i += 2
                continue
            # 未知转义：原样保留反斜杠，不要静默丢弃
            out.append(ch)
            i += 1
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def _parse_scalar(text: str) -> Any:
    s = text.strip()
    if s == "":
        return None

    # 引号字符串
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "\"'":
        inner = s[1:-1]
        if s[0] == '"':
            inner = _unescape_double(inner)
        return inner

    low = s.lower()
    if low in ("null", "~", ""):
        return None
    if low in ("true", "yes", "on"):
        return True
    if low in ("false", "no", "off"):
        return False

    if _INT_RE.match(s):
        try:
            return int(s)
        except ValueError:
            pass
    if _NUM_RE.match(s):
        try:
            return float(s)
        except ValueError:
            pass

    return s


def _split_inline_list(text: str) -> list[str] | None:
    """把 `[a, b, c]` 拆成元素列表；不是行内列表则返回 None。"""
    s = text.strip()
    if not (s.startswith("[") and s.endswith("]")):
        return None
    body = s[1:-1].strip()
    if body == "":
        return []
    items: list[str] = []
    buf = []
    q: str | None = None
    depth = 0
    for ch in body:
        if q:
            buf.append(ch)
            if ch == q:
                q = None
            continue
        if ch in "\"'":
            q = ch
            buf.append(ch)
            continue
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
        if ch == "," and depth == 0:
            items.append("".join(buf).strip())
            buf = []
        else:
            buf.append(ch)
    if buf:
        items.append("".join(buf).strip())
    return items


def _parse_value(text: str) -> Any:
    """解析一个「值」位置的内容（可能是行内列表或标量）。"""
    lst = _split_inline_list(text)
    if lst is not None:
        return [_parse_scalar(x) for x in lst]
    return _parse_scalar(text)


# ══════════════════════════════════════════════════════════════
# 块标量（`|` / `>`）展开
# ══════════════════════════════════════════════════════════════
# 块标量必须在「行预处理」之前处理掉：它依赖原始缩进和 `#`，
# 而 _prep 会把缩进压平、把 `#` 当注释切掉。
#
# 做法是把多行内容**编码成一个双引号标量**，再交回普通流程。
# 这样只影响这一行的文本形态，不动后面的解析逻辑。
_BLOCK_RE = re.compile(
    r"^(?P<indent>[ ]*)(?P<key>(?:-[ ]+)?[^:#\s][^:#]*?)[ ]*:[ ]*"
    r"(?P<style>[|>])(?P<chomp>[-+]?)(?P<digits>\d*)[ ]*(?P<tail>#.*)?$"
)


def _escape_double(text: str) -> str:
    """把任意文本编码进双引号标量（_unescape_double 的逆运算）。"""
    out: list[str] = []
    for ch in text:
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\n":
            out.append("\\n")
        elif ch == "\t":
            out.append("\\t")
        elif ch == "\r":
            out.append("\\r")
        else:
            out.append(ch)
    return "".join(out)


def _expand_block_scalars(text: str) -> str:
    """把 `key: |` / `key: >` 的后续缩进块折叠回该行的值位置。"""
    lines = text.splitlines()
    out: list[str] = []
    i = 0
    n = len(lines)
    while i < n:
        raw = lines[i]
        m = _BLOCK_RE.match(raw)
        if not m:
            out.append(raw)
            i += 1
            continue

        indent = len(m.group("indent"))
        style = m.group("style")
        chomp = m.group("chomp")
        i += 1

        # 收集属于这个块的后续行：空行，或缩进比键更深的行
        block: list[str] = []
        while i < n:
            cur = lines[i]
            if cur.strip() == "":
                block.append("")
                i += 1
                continue
            cur_indent = len(cur) - len(cur.lstrip(" "))
            if cur_indent <= indent:
                break
            block.append(cur)
            i += 1

        # 去掉块内容的最小公共缩进
        non_blank = [b for b in block if b.strip() != ""]
        if non_blank:
            common = min(len(b) - len(b.lstrip(" ")) for b in non_blank)
            block = [b[common:] if b.strip() else "" for b in block]
        # 去掉首尾的空行
        while block and block[0].strip() == "":
            block.pop(0)
        while block and block[-1].strip() == "":
            block.pop()

        if style == "|":
            content = "\n".join(block)
        else:
            # `>` 折叠：连续非空行之间用空格连接，空行变成换行
            pieces: list[str] = []
            buf: list[str] = []
            for b in block:
                if b.strip() == "":
                    if buf:
                        pieces.append(" ".join(buf))
                        buf = []
                    pieces.append("")
                else:
                    buf.append(b.strip())
            if buf:
                pieces.append(" ".join(buf))
            content = "\n".join(pieces)

        # 收尾修饰：`-` 去掉尾部换行，默认 clip 保留一个，`+` 全部保留
        if chomp == "-":
            content = content.rstrip("\n")
        elif chomp == "+":
            content = content + "\n"
        else:
            content = content.rstrip("\n") + "\n"

        head = f"{m.group('indent')}{m.group('key')}: "
        out.append(head + '"' + _escape_double(content) + '"')
    return "\n".join(out)


# ══════════════════════════════════════════════════════════════
# 行预处理
# ══════════════════════════════════════════════════════════════
def _strip_comment(line: str) -> str:
    out: list[str] = []
    q: str | None = None
    for i, ch in enumerate(line):
        if q:
            out.append(ch)
            if ch == q:
                q = None
            continue
        if ch in "\"'":
            q = ch
            out.append(ch)
            continue
        if ch == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        out.append(ch)
    return "".join(out).rstrip()


def _prep(text: str) -> list[tuple[int, str, int]]:
    """返回 [(缩进, 内容, 原始行号)]，去掉空行与注释行。"""
    rows: list[tuple[int, str, int]] = []
    for ln, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise MiniYamlError(f"第 {ln} 行：缩进请用空格，不要用 Tab")
        stripped = _strip_comment(raw)
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip(" "))
        rows.append((indent, stripped.strip(), ln))
    return rows


_KEY_RE = re.compile(r"^([^:#]+?)\s*:\s*(.*)$")


def _is_key_line(content: str) -> bool:
    return bool(_KEY_RE.match(content)) and not content.startswith("- ")


# ══════════════════════════════════════════════════════════════
# 递归解析
# ══════════════════════════════════════════════════════════════
def _parse_block(rows: list[tuple[int, str, int]], idx: int,
                 indent: int) -> tuple[Any, int]:
    """解析缩进为 indent 的一个块，返回 (值, 下一个未消费的行号)。"""
    if idx >= len(rows):
        return None, idx

    is_list = rows[idx][1].startswith("- ")
    if is_list:
        return _parse_list(rows, idx, indent)
    return _parse_map(rows, idx, indent)


def _parse_map(rows: list[tuple[int, str, int]], idx: int,
               indent: int) -> tuple[dict, int]:
    result: dict[str, Any] = {}
    while idx < len(rows):
        cur_indent, content, ln = rows[idx]
        if cur_indent < indent:
            break
        if cur_indent > indent:
            raise MiniYamlError(
                f"第 {ln} 行：缩进比上一层深，但上一行没有可嵌套的键")
        if content.startswith("- "):
            break

        m = _KEY_RE.match(content)
        if not m:
            raise MiniYamlError(f"第 {ln} 行：不是合法的 `键: 值` 形式 → {content!r}")

        key = m.group(1).strip()
        rest = m.group(2).strip()
        if len(key) >= 2 and key[0] == key[-1] and key[0] in "\"'":
            key = key[1:-1]

        if rest != "":
            result[key] = _parse_value(rest)
            idx += 1
            continue

        # 值为空 → 看下一行是不是更深的块
        if idx + 1 < len(rows) and rows[idx + 1][0] > cur_indent:
            value, idx = _parse_block(rows, idx + 1, rows[idx + 1][0])
            result[key] = value
        else:
            result[key] = None
            idx += 1
    return result, idx


def _parse_list(rows: list[tuple[int, str, int]], idx: int,
                indent: int) -> tuple[list, int]:
    items: list[Any] = []
    while idx < len(rows):
        cur_indent, content, ln = rows[idx]
        if cur_indent < indent or not content.startswith("- "):
            break
        if cur_indent > indent:
            raise MiniYamlError(f"第 {ln} 行：列表项缩进不一致")

        rest = content[2:].strip()

        # `- ` 后面为空 → 嵌套块
        if rest == "":
            if idx + 1 < len(rows) and rows[idx + 1][0] > cur_indent:
                value, idx = _parse_block(rows, idx + 1, rows[idx + 1][0])
                items.append(value)
            else:
                items.append(None)
                idx += 1
            continue

        # `- key: value` → 列表项是映射，后续同级键也算进这个映射
        if _is_key_line(rest):
            sub_rows = [(cur_indent + 2, rest, ln)]
            idx += 1
            while idx < len(rows) and rows[idx][0] > cur_indent \
                    and not (rows[idx][0] == cur_indent
                             and rows[idx][1].startswith("- ")):
                sub_rows.append(rows[idx])
                idx += 1
            value, _ = _parse_map(sub_rows, 0, cur_indent + 2)
            items.append(value)
            continue

        # 普通标量项
        items.append(_parse_value(rest))
        idx += 1

    return items, idx


# ══════════════════════════════════════════════════════════════
# 对外接口
# ══════════════════════════════════════════════════════════════
def loads(text: str, prefer_pyyaml: bool = True) -> Any:
    """解析 YAML 文本。装了 pyyaml 就优先用它。"""
    if prefer_pyyaml:
        try:
            import yaml  # type: ignore

            return yaml.safe_load(text)
        except ImportError:
            pass
        except Exception as exc:
            raise MiniYamlError(f"pyyaml 解析失败：{exc}") from exc

    rows = _prep(_expand_block_scalars(text))
    if not rows:
        return None
    value, idx = _parse_block(rows, 0, rows[0][0])
    if idx < len(rows):
        raise MiniYamlError(f"第 {rows[idx][2]} 行起有无法归属的内容")
    return value


def load(path: str, prefer_pyyaml: bool = True) -> Any:
    """读取 YAML 文件。"""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, encoding="utf-8") as f:
        return loads(f.read(), prefer_pyyaml=prefer_pyyaml)


# ══════════════════════════════════════════════════════════════
def _selftest() -> int:
    sample = r"""
# 顶层注释
meta:
  unit: degree
  motor_order: [拇指Flex, 拇指Aux, 食指]
  angle_limits: [59, 90, 81]
  verified: false
  ratio: 0.5
  nothing:

gestures:
  open:
    description: 完全张开   # 行尾注释
    angles: [0, 0, 0]
    duration_ms: 1000
    risk: low
  fist:
    description: "握拳"
    angles: [50, 70, 78]
    duration_ms: 1200

seq:
  - name: 张开
    angles: [0, 0, 0]
    duration_ms: 800
  - name: 握拳
    angles: [50, 70, 78]
    duration_ms: 1200
  - 纯标量项
  - another

notes:
  folded: >
    第一行 说明
    第二行 还在同一段
      缩进更深的行

    空行之后是新段落
  literal: |
    第一行
    第二行
  stripped: >-
    只有一段，不要尾部换行
  with_hash: >
    这行有 # 井号，不是注释
  with_quote: >
    引号 " 和反斜杠 \ 都要活下来
"""
    data = loads(sample, prefer_pyyaml=False)

    checks = [
        ("meta 是字典", isinstance(data.get("meta"), dict)),
        ("unit == degree", data["meta"].get("unit") == "degree"),
        ("motor_order 长度 3", len(data["meta"]["motor_order"]) == 3),
        ("motor_order[0] 中文", data["meta"]["motor_order"][0] == "拇指Flex"),
        ("angle_limits 全为 int", all(isinstance(x, int)
                                     for x in data["meta"]["angle_limits"])),
        ("verified 是 False", data["meta"]["verified"] is False),
        ("ratio 是 float", isinstance(data["meta"]["ratio"], float)),
        ("nothing 是 None", data["meta"].get("nothing") is None),
        ("gestures.open.angles[2] == 0", data["gestures"]["open"]["angles"][2] == 0),
        ("行尾注释被去掉",
         data["gestures"]["open"]["description"] == "完全张开"),
        ("引号字符串被还原", data["gestures"]["fist"]["description"] == "握拳"),
        ("列表项是字典", isinstance(data["seq"][0], dict)),
        ("列表项字典取值", data["seq"][0]["name"] == "张开"),
        ("多个字典项", data["seq"][1]["duration_ms"] == 1200),
        ("纯标量列表项", data["seq"][2] == "纯标量项"),
        ("`>` 折叠：段内换行变空格",
         data["notes"]["folded"].startswith("第一行 说明 第二行 还在同一段")),
        ("`>` 折叠：空行变换行",
         "\n" in data["notes"]["folded"].strip()),
        ("`|` 保留换行",
         data["notes"]["literal"].strip() == "第一行\n第二行"),
        ("`>-` 去掉尾部换行",
         not data["notes"]["stripped"].endswith("\n")),
        ("块标量里的 # 不当注释",
         "# 井号" in data["notes"]["with_hash"]),
        ("块标量里的引号/反斜杠不被破坏",
         '"' in data["notes"]["with_quote"]
         and "\\" in data["notes"]["with_quote"]),
        ("双引号转义顺序正确（真实反斜杠+n 不变换行）",
         _unescape_double(r"a\nb") == "a\nb"
         and _unescape_double(r"c\nd") == "c\nd"
         and _unescape_double("x\\\\ny") == "x\\ny"),
    ]

    print("── miniyaml 自检 ──")
    ok_all = True
    for label, passed in checks:
        print(f"  {'OK  ' if passed else 'FAIL'}  {label}")
        if not passed:
            ok_all = False

    print()
    print(f"  pyyaml 可用：{have_pyyaml()}")
    print("  自检通过" if ok_all else "  自检失败")
    return 0 if ok_all else 1


if __name__ == "__main__":
    import sys

    sys.exit(_selftest())
