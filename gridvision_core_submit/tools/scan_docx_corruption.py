#!/usr/bin/env python3
"""Scan docx code blocks for corruption patterns."""
from __future__ import annotations

import re
from pathlib import Path
from docx import Document

DOCX = Path(r"D:\Desktop\川大\科研\GridVision_软件著作权代码最终版.docx")
ORIG = Path(r"D:\Desktop\川大\科研\GridVision_软件著作权代码最终提交版_Consolas版.docx")

CLEAN_ROOT = Path(__file__).resolve().parents[1] / "clean"


def extract_modules(doc) -> list[dict]:
    frontend, backend = [], []
    section = None
    pending = None
    for para in doc.paragraphs:
        t = para.text.strip()
        if not t:
            continue
        if t == "前端代码":
            section = "frontend"
            pending = None
            continue
        if t == "后端代码":
            section = "backend"
            pending = None
            continue
        if re.match(r"^文件(名称)?[：:]", t):
            name = re.sub(r"^文件(名称)?[：:]", "", t).strip()
            name = re.sub(r"^(frontend|backend)/", "", name)
            pending = {"name": name, "desc": "", "code": ""}
            continue
        if re.match(r"^来源[：:]", t):
            continue
        if re.match(r"^功能(说明)?[：:]", t):
            if pending is not None:
                pending["desc"] = re.sub(r"^功能(说明)?[：:]", "", t).strip()
            continue
        if pending is not None and (
            len(t) > 80
            or t.startswith(
                ("const ", "function ", "def ", "class ", "import ", "from ", "async ", "@")
            )
        ):
            pending["code"] = t
            (frontend if section == "frontend" else backend).append(pending)
            pending = None
    return frontend + backend


def scan_issues(code: str, name: str) -> list[dict]:
    issues = []
    lines = code.split("\n")
    for i, line in enumerate(lines):
        n = i + 1
        stripped = line.strip()
        if re.match(r"^point_id:\s*$", stripped):
            issues.append({"file": name, "line": n, "type": "orphan_point_id", "text": line})
        if re.match(r"^segment=\s*$", stripped) or re.match(r"segment=\$\{", stripped):
            issues.append({"file": name, "line": n, "type": "orphan_segment", "text": line})
        if re.match(r"^\s*print\s*\(\s*$", stripped):
            issues.append({"file": name, "line": n, "type": "truncated_print", "text": line})
        if re.match(r"^\s*except\s*:\s*$", stripped) and i + 1 < len(lines):
            nxt = lines[i + 1].strip()
            if not nxt or re.match(r"^(def |class |@|except|else|elif|finally)", nxt):
                issues.append({"file": name, "line": n, "type": "empty_except", "text": line})

    text = code
    for m in re.finditer(r"^if .+:\nelif ", text, flags=re.MULTILINE):
        issues.append(
            {
                "file": name,
                "line": text[: m.start()].count("\n") + 1,
                "type": "if_direct_elif",
                "text": text[m.start() : m.end() + 40],
            }
        )

    for m in re.finditer(r"else if\s*\([^\)]*\)\s*\{\s*point_id:", text):
        issues.append(
            {
                "file": name,
                "line": text[: m.start()].count("\n") + 1,
                "type": "broken_else_if",
                "text": text[m.start() : m.start() + 80],
            }
        )

    for m in re.finditer(r"if\s*\([^\)]*\)\s*\{\s*\}", text):
        issues.append(
            {
                "file": name,
                "line": text[: m.start()].count("\n") + 1,
                "type": "empty_if_block_js",
                "text": m.group(0),
            }
        )

    for m in re.finditer(r"else\s*\{\s*\}", text):
        issues.append(
            {
                "file": name,
                "line": text[: m.start()].count("\n") + 1,
                "type": "empty_else_js",
                "text": m.group(0),
            }
        )

    for m in re.finditer(r"^if .+:\s*\n\s*elif ", text, flags=re.MULTILINE):
        between = text[m.start() : m.end()]
        if "pass" not in between.split("\n")[1:-1]:
            issues.append(
                {
                    "file": name,
                    "line": text[: m.start()].count("\n") + 1,
                    "type": "if_empty_before_elif_py",
                    "text": between[:120],
                }
            )

    return issues


def main():
    doc = Document(str(DOCX))
    mods = extract_modules(doc)
    all_issues = []
    for mod in mods:
        all_issues.extend(scan_issues(mod["code"], mod["name"]))
    print("modules", [m["name"] for m in mods])
    print("total issues", len(all_issues))
    for iss in all_issues:
        print("---")
        for k, v in iss.items():
            print(f"{k}: {v!r}")


if __name__ == "__main__":
    main()
