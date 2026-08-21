#!/usr/bin/env python3
"""Surgical repair of log-removal corruption in copyright docx."""
from __future__ import annotations

import ast
import re
import shutil
import sys
from pathlib import Path

from docx import Document

DOCX = Path(r"D:\Desktop\川大\科研\GridVision_软件著作权代码最终版.docx")
SUBMIT = Path(__file__).resolve().parents[1]
REPAIRS: list[dict] = []


def extract_blocks(doc) -> list[dict]:
    blocks: list[dict] = []
    current: dict | None = None
    state: str | None = None
    for par in doc.paragraphs:
        ts = par.text.strip()
        if ts in ("前端代码", "后端代码"):
            continue
        if re.match(r"^文件名称[：:]", ts):
            if current:
                blocks.append(current)
            current = {"name": re.sub(r"^文件名称[：:]", "", ts).strip(), "pars": []}
            state = "after_name"
            continue
        if current is None:
            continue
        if re.match(r"^功能说明[：:]", ts):
            state = "after_func"
            continue
        if state == "after_func" and ts:
            state = "code"
            continue
        if state in ("after_name", "after_func") and not ts:
            continue
        if state == "code" or (
            state == "after_func"
            and ts.startswith(("const ", "function ", "def ", "class ", "import ", "from ", "@"))
        ):
            state = "code"
            current["pars"].append(par)
    if current:
        blocks.append(current)
    return blocks


def reference_lines(name: str) -> list[str]:
    sub = "frontend" if name.endswith(".js") else "backend"
    raw = (SUBMIT / sub / name).read_text(encoding="utf-8")
    if name.endswith(".js"):
        return strip_js(raw).splitlines()
    return strip_py(raw).splitlines()


def strip_py(text: str) -> str:
    lines = text.splitlines()
    out: list[str] = []
    audit = re.compile(r"^\s*(_log_[a-zA-Z0-9_]+|_point_flow_log_dashboard)\s*\(")
    i = 0
    while i < len(lines):
        line = lines[i]
        if audit.match(line) or re.match(r"^\s*print\s*\(", line):
            if line.rstrip().endswith(")") and line.count("(") <= line.count(")"):
                i += 1
                continue
            j = i + 1
            depth = line.count("(") - line.count(")")
            while j < len(lines) and depth > 0:
                depth += lines[j].count("(") - lines[j].count(")")
                j += 1
            i = j
            continue
        out.append(line)
        i += 1
    text = "\n".join(out)
    while re.search(r"except [^\n]+:\n\s*print\(", text):
        text = re.sub(r"(except [^\n]+:\n)(\s*)print\([^\n]*\n", r"\1\2    pass\n", text, count=1)
    text = re.sub(
        r"(except [^\n]+:\n)(\s*)(?=try:)",
        lambda m: f"{m.group(1)}{m.group(2)}    pass\n{m.group(2)}",
        text,
    )
    return fill_empty_py_blocks(text)


def fill_empty_py_blocks(text: str) -> str:
    lines = text.splitlines()
    out: list[str] = []
    for i, line in enumerate(lines):
        out.append(line)
        stripped = line.strip()
        if not re.match(r"^(if |elif |while |for ).+:$", stripped) and stripped not in ("else:",) and not stripped.startswith("except "):
            continue
        indent = len(line) - len(line.lstrip())
        j = i + 1
        while j < len(lines) and not lines[j].strip():
            j += 1
        if j >= len(lines) or (len(lines[j]) - len(lines[j].lstrip()) <= indent):
            out.append(" " * (indent + 4) + "pass")
    return "\n".join(out)


def strip_js(text: str) -> str:
    for fn in (
        "logInspectionPointCoordAudit",
        "logInspectionRouteBindingSummary",
        "logRoutePointAudit",
        "printPlaybackCheck",
    ):
        text = re.sub(rf"\s*{re.escape(fn)}\([\s\S]*?\);\s*\n", "\n", text)
    out, i, n = [], 0, len(text)
    while i < n:
        m = re.match(r"console\.(log|warn|error)\s*\(", text[i:])
        if not m:
            out.append(text[i])
            i += 1
            continue
        j = i + m.end()
        depth = 1
        instr = None
        while j < n and depth:
            c = text[j]
            if instr:
                if c == instr:
                    instr = None
            elif c in "'\"`":
                instr = c
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
            j += 1
        while j < n and text[j] in " \t":
            j += 1
        if j < n and text[j] == ";":
            j += 1
        if j < n and text[j] == "\n":
            j += 1
        i = j
    text = "".join(out)
    text = re.sub(
        r"(\} else if \(rowsWithXY\.length\) \{\n)\s*logRoutePointAudit[\s\S]*?\n\s*\}\s*\n",
        r"\1  }\n",
        text,
        count=1,
    )
    text = re.sub(
        r"\s*const segSummary = \[\];[\s\S]*?else \{[\s\S]*?\}\s*\n",
        "\n",
        text,
        count=1,
    )
    text = re.sub(r"\s*_badSnappedLogged\.clear\(\);\s*\n", "\n", text)
    return text


def find_subseq(hay: list[str], needle: list[str]) -> int:
    if not needle:
        return -1
    for i in range(len(hay) - len(needle) + 1):
        if hay[i : i + len(needle)] == needle:
            return i
    return -1


def replace_region(lines: list[str], start: int, end: int, new_lines: list[str], name: str, label: str) -> list[str]:
    before = "\n".join(lines[start:end])
    after = "\n".join(new_lines)
    REPAIRS.append(
        {
            "file": name,
            "line": start + 1,
            "type": label,
            "before": before[:400],
            "after": after[:400],
        }
    )
    return lines[:start] + new_lines + lines[end:]


def repair_lines(name: str, lines: list[str], ref: list[str]) -> list[str]:
    fixed = list(lines)

    # JS: broken else-if audit fragment
    for i, line in enumerate(fixed):
        if "else if (rowsWithXY.length)" in line and i + 1 < len(fixed) and fixed[i + 1].strip().startswith("point_id:"):
            ref_i = find_subseq(ref, ["  } else if (rowsWithXY.length) {", "  }"])
            new = ["  } else if (rowsWithXY.length) {", "  }"]
            j = i + 1
            while j < len(fixed) and fixed[j].strip() not in ("}", "  }"):
                j += 1
            if j < len(fixed):
                j += 1
            fixed = replace_region(fixed, i, j, new, name, "broken_else_if")
            break

    # orphan closing fragments
    fixed = [ln for ln in fixed if ln.strip() not in ("})));",) and not re.fullmatch(r"point_id: String\(r\.point.*", ln.strip())]

    # Python: sync corrupted step5_detect region by anchor
    anchor = "detect_black_inspection_points_with_stats"
    if anchor in "\n".join(fixed):
        doc_a = next(i for i, l in enumerate(fixed) if anchor in l)
        doc_b = next(i for i, l in enumerate(fixed) if l.strip() == "self.line_inspection_points = []")
        ref_a = next(i for i, l in enumerate(ref) if anchor in l)
        ref_b = next(i for i, l in enumerate(ref) if l.strip() == "self.line_inspection_points = []")
        ref_slice = [ln for ln in ref[ref_a:ref_b] if ln.strip() != "" or ref[ref_a:ref_b].index(ln) == 0]
        # keep blank lines roughly
        ref_slice = ref[ref_a:ref_b]
        if fixed[doc_a:doc_b] != ref_slice:
            fixed = replace_region(fixed, doc_a, doc_b, ref_slice, name, "step5_detect_region")

    # Python: empty if before segments_out in replan
    marker = "if topo_reload_ok and missing_topo_edges:"
    if marker in "\n".join(fixed):
        i = next(j for j, l in enumerate(fixed) if marker in l)
        j = i + 1
        while j < len(fixed) and not fixed[j].strip():
            j += 1
        if j < len(fixed) and fixed[j].startswith("    segments_out"):
            fixed.insert(j, "        pass")
            REPAIRS.append(
                {
                    "file": name,
                    "line": j + 1,
                    "type": "empty_if_pass",
                    "before": "",
                    "after": "        pass",
                }
            )

    # Generic: replace any remaining syntax-broken spans using ref alignment from first def/function
    if name.endswith(".py"):
        doc_start = next((i for i, l in enumerate(fixed) if l.lstrip().startswith("def ")), 0)
        ref_start = next((i for i, l in enumerate(ref) if l.lstrip().startswith("def ")), 0)
        trial = list(fixed)
        n = len(trial) - doc_start
        ref_seg = ref[ref_start : ref_start + n]
        if len(ref_seg) == n:
            trial[doc_start : doc_start + n] = ref_seg
            chunk = "\n".join(trial[doc_start : doc_start + n])
            try:
                compile(chunk, "<chk>", "exec")
                if trial != fixed:
                    REPAIRS.append(
                        {
                            "file": name,
                            "line": doc_start + 1,
                            "type": "def_block_resync",
                            "before": "\n".join(fixed[doc_start : doc_start + 5]),
                            "after": "\n".join(trial[doc_start : doc_start + 5]),
                        }
                    )
                    fixed = trial
            except SyntaxError:
                pass

    # Python: truncated tuple after print removal (bg_src)
    if re.search(r"bg_src = \(\s*\n\s+or metadata", "\n".join(fixed)):
        ref_slice = [
            "        bg_src = (",
            '            metadata.get("clean_map_image")',
            '            or metadata.get("display_map_image")',
            '            or metadata.get("map_image")',
            "            or input_file",
            '            or "data/test.png"',
            "        )",
        ]
        i = next(j for j, l in enumerate(fixed) if "bg_src = (" in l)
        j = i + 1
        while j < len(fixed) and fixed[j].strip() != ")":
            j += 1
        if j < len(fixed):
            j += 1
        fixed = replace_region(fixed, i, j, ref_slice, name, "bg_src_tuple")

    return fixed


def has_issues(code: str, name: str) -> bool:
    if re.search(r"^if .+:\n\s*elif ", code, re.M):
        return True
    if re.search(r"else if \([^\)]*\) \{\s*\n\s*point_id:", code):
        return True
    if re.search(r"except [^\n]+:\n\s*try:", code):
        return True
    if re.search(r"point_id: String\(r\.point", code):
        return True
    if re.fullmatch(r"\s*\}\)\);\s*", code.strip()):
        return True
    if re.search(r"bg_src = \(\s*\n\s+or metadata", code):
        return True
    if re.search(r"^\s*print\s*\(\s*$", code, re.M):
        return True
    return False


def pad_or_trim(lines: list[str], n: int) -> list[str]:
    if len(lines) == n:
        return lines
    if len(lines) > n:
        return lines[:n]
    return lines + [""] * (n - len(lines))


def main() -> int:
    work = DOCX
    repair_tmp = DOCX.with_suffix(".repair_save.docx")
    if repair_tmp.exists():
        work = repair_tmp
    doc = Document(str(work))
    files_fixed = 0
    refs = {b["name"]: reference_lines(b["name"]) for b in extract_blocks(doc)}
    for block in extract_blocks(doc):
        name = block["name"]
        pars = block["pars"]
        before = [p.text for p in pars]
        code = "\n".join(before)
        if not has_issues(code, name):
            continue
        fixed = repair_lines(name, before, refs[name])
        fixed = pad_or_trim(fixed, len(before))
        for par, new in zip(pars, fixed):
            par.text = new
        files_fixed += 1

    problems = []
    for block in extract_blocks(doc):
        code = "\n".join(p.text for p in block["pars"])
        if has_issues(code, block["name"]):
            problems.append(block["name"])

    tmp = DOCX.with_suffix(".repair_save.docx")
    doc.save(str(tmp))
    try:
        shutil.copy2(tmp, DOCX)
    except PermissionError:
        print(f"WARN: 目标文件占用，已保存至 {tmp}")
    else:
        tmp.unlink(missing_ok=True)

    print("修复文件数:", files_fixed)
    for r in REPAIRS:
        print("---")
        print("文件:", r["file"])
        print("位置: 约第", r["line"], "行")
        print("类型:", r["type"])
        print("修复前:", r["before"])
        print("修复后:", r["after"])
    print("最终检查:", "PASS" if not problems else problems)
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
