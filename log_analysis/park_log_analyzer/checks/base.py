"""规则基类与注册表（开发文档 §5.2 / §6）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence

PASS, WARN, FAIL, INFO, SKIP = "PASS", "WARN", "FAIL", "INFO", "SKIP"
_ORDER = {FAIL: 3, WARN: 2, INFO: 1, PASS: 0, SKIP: -1}


@dataclass
class CheckResult:
    id: str
    title: str
    severity: str
    summary: str
    detail: dict = field(default_factory=dict)
    evidence: List[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "id": self.id,
            "title": self.title,
            "severity": self.severity,
            "summary": self.summary,
            "detail": self.detail,
            "evidence": self.evidence,
        }


@dataclass
class Check:
    id: str
    title: str
    requires: Sequence[str]
    fn: Callable


CHECKS: List[Check] = []


def check(cid: str, title: str, requires: Sequence[str] = ()):
    def deco(fn):
        CHECKS.append(Check(cid, title, tuple(requires), fn))
        return fn

    return deco


def run_checks(ctx, only: Sequence[str] | None = None) -> List[CheckResult]:
    enabled = ctx.cfg.get("checks", {})
    out: List[CheckResult] = []
    for ck in CHECKS:
        if only and not _match(ck.id, only):
            continue
        if enabled.get(ck.id) is False:
            continue
        missing = [k for k in ck.requires if not ctx.has_topic(k)]
        if missing:
            names = [ctx.topic(k) or ("(未配置 %s)" % k) for k in missing]
            out.append(CheckResult(
                ck.id, ck.title, SKIP,
                "缺少必需话题：%s" % "、".join(names),
                {"missing": names},
            ))
            continue
        try:
            r = ck.fn(ctx)
        except Exception as e:  # 单条规则异常不影响整份报告
            r = CheckResult(ck.id, ck.title, WARN, "规则执行异常：%s: %s" % (type(e).__name__, e),
                            {"error": repr(e)})
        out.append(r)
    return out


def _match(cid: str, only: Sequence[str]) -> bool:
    for pat in only:
        if pat.endswith("*") and cid.startswith(pat[:-1]):
            return True
        if pat == cid:
            return True
    return False


def _find_result(results: Sequence[CheckResult], cid: str):
    for r in results:
        if r.id == cid:
            return r
    return None


def worst(results: Sequence[CheckResult]) -> str:
    level = PASS
    for r in results:
        if _ORDER.get(r.severity, 0) > _ORDER.get(level, 0):
            level = r.severity
    return level


def exit_code(results: Sequence[CheckResult]) -> int:
    sev = {r.severity for r in results}
    if FAIL in sev:
        return 1
    if WARN in sev:
        return 2
    return 0
