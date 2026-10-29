# SPDX-License-Identifier: MulanPSL-2.0
"""检测：拿到"目标在哪里"。

**统一入口**（接口形状刻意与 `detect_object` 服务一致，将来若改用该服务，
只需替换本模块的实现，调用方一行不用动）：

    Detector.detect(object_name, backend="") ->
        {"success": bool, "bbox_2d": [..], "object_center_3d": [],
         "confidence": float, "message": str}

    backend: "yolo" | "vlm" | ""（空 = 用配置的 detect_scheme）

**两条路径**（行为与整理前一致）：

  yolo  静置(等臂稳) → 取图(去畸变) → 宿主 GPU YOLO **全量**检测（不带名字过滤）
        → 两帧一致性（间隔 interval、同类且 IoU > iou 才保留，滤闪烁误检）
        → 盒内排除 → 文本 LLM 把用户的措辞对到检测出的类名
  vlm   静置 → 取图(去畸变) → 一次多模态调用，模型直接给 bbox → 盒内排除

**检测环节的失败一律 `raise RuntimeError("detection_failed: ...")`，文案与整理前逐字一致**
（pilot 依赖这些文案向用户转述"没有 X"，不要随意改写）。

**匹配环节（把用户措辞对到类名的那个大模型）的失败用另一个前缀 `matcher_llm_failed:`**
——那一步出错是大模型的问题（超时、连不上、返回内容解析不了），跟"桌上没有这个物体"
是两回事，文案必须写明白，别让 robonix 和操作者跑去检查桌面。

依赖注入：本模块不读任何全局配置，也不自己订阅相机 —— 帧来自 `camera.CameraFrames`，
像素↔臂基座换算与落盒点判定来自 `geometry.*`，参数由 `DetectConfig` 传入。
"""
from __future__ import annotations

import base64
import json
import logging
import os
import time
import urllib.request
from dataclasses import dataclass
from typing import Optional, Sequence
from urllib.parse import urlencode

from .camera import CameraFrames
from .config import choice, num, text
from .geometry import DropZone, Homography

log = logging.getLogger("pick_skill.detect")

_BACKEND_ALIASES = {
    "yolo": "yolo", "yolo_http": "yolo",
    "vlm": "vlm", "llm": "vlm",
}


def normalize_backend(name: object) -> str:
    """把后端选择归一成 "yolo" / "vlm"；认不出（或空）返回 ""。"""
    return _BACKEND_ALIASES.get(str(name or "").strip().lower(), "")


@dataclass
class DetectConfig:
    """检测所需的一切参数（由技能在 init 时从清单配置填好）。"""

    # 取图（臂刚动过：先等它停稳再拍，避免拍到运动中的臂/拖影）
    settle_s: float = 2.0
    first_frame_wait_s: float = 5.0
    max_frame_age_s: float = 3.0
    debug_frame_dir: str = ""          # 非空 = 把检测用的帧落盘（调试用，默认关）

    # YOLO（宿主 GPU 服务）
    yolo_url: str = "http://127.0.0.1:8770/detect"
    yolo_timeout_s: float = 10.0
    yolo_conf: float = 0.6
    yolo_iou: float = 0.45
    two_frame_interval_s: float = 0.7
    two_frame_iou: float = 0.3

    # VLM（开放词表；与 pilot 同一个多模态端点）
    vlm_base_url: str = ""
    vlm_api_key: str = ""
    vlm_model: str = ""
    vlm_timeout_s: float = 60.0

    # 措辞匹配（纯文本 LLM：把"红色的那个"对到 YOLO 类名）
    match_base_url: str = ""
    match_api_key: str = ""
    match_model: str = ""
    match_timeout_s: float = 30.0

    # 默认后端（backend 为空时用它；运行时开关文件可覆盖，见 atlas_bridge）
    default_backend: str = "yolo"

    @classmethod
    def from_cfg(cls, cfg: dict) -> "DetectConfig":
        """按 manifest 的 config 块建配置；缺项一律用默认值，坏值告警后回落。

        settle / 首帧等待暂用固定值：臂刚动完要等它停稳（否则拍到运动中的臂），
        刚激活时首帧要等相机流起来。
        """
        conf = cls(
            settle_s=2.0,
            first_frame_wait_s=5.0,
            max_frame_age_s=3.0,
            debug_frame_dir=text(cfg, "debug_frame_dir"),   # 空 = 不落盘
            yolo_url=text(cfg, "yolo_http_url", "http://127.0.0.1:8770/detect"),
            yolo_timeout_s=num(cfg, "yolo_http_timeout_s", 10.0),
            yolo_conf=num(cfg, "yolo_conf", 0.6),
            yolo_iou=num(cfg, "yolo_iou", 0.45),
            two_frame_interval_s=num(cfg, "yolo_two_frame_interval_s", 0.7, lo=0.0),
            two_frame_iou=num(cfg, "yolo_two_frame_iou", 0.3),
            vlm_base_url=text(cfg, "grab_vlm_base_url"),
            vlm_api_key=text(cfg, "grab_vlm_api_key"),
            vlm_model=text(cfg, "grab_vlm_model"),
            match_base_url=text(cfg, "grab_match_llm_base_url"),
            match_api_key=text(cfg, "grab_match_llm_api_key"),
            match_model=text(cfg, "grab_match_llm_model"),
            default_backend=choice(cfg, "detect_scheme", ("yolo", "vlm"), "yolo"),
        )
        if conf.default_backend == "vlm" and not (
                conf.vlm_base_url and conf.vlm_api_key and conf.vlm_model):
            log.warning("detect_scheme=vlm but grab_vlm_base_url/api_key/model "
                        "not fully configured — will fail at call time")
        return conf


class Detector:
    """检测入口。由技能在 activate 时构造，之后每次抓取调用一次。"""

    def __init__(self, frames: CameraFrames, homography: Homography,
                 drop_zone: DropZone, cfg: DetectConfig) -> None:
        self.frames = frames
        self.homography = homography
        self.drop_zone = drop_zone
        self.cfg = cfg

    # ── 统一入口 ────────────────────────────────────────────────────────
    def detect(self, object_name: str, backend: str = "",
               exclude_box: bool = True) -> dict:
        """见模块 docstring：`backend` 空值走配置的默认后端。

        exclude_box=False 关掉"盒内排除"——演示（demo）要用：那个模式下物体本来
        就可能停在落盒点附近，排除掉会让它"看不见"。
        """
        chosen = normalize_backend(backend)
        if backend and not chosen:
            log.warning("unknown backend %r (expected 'yolo'/'vlm' or empty) — "
                        "falling back to default %r", backend,
                        self.cfg.default_backend)
        chosen = chosen or normalize_backend(self.cfg.default_backend) or "yolo"
        log.info("detect: object=%r backend_req=%r -> %s (exclude_box=%s)",
                 object_name, backend, chosen, exclude_box)

        # 等待臂静止：reset 之后臂可能还在动，拍之前让它停稳。
        if self.cfg.settle_s > 0:
            time.sleep(self.cfg.settle_s)

        if chosen == "vlm":
            return self._detect_via_vlm(object_name, exclude_box=exclude_box)
        return self._detect_via_yolo(object_name, exclude_box=exclude_box)

    # ── 取图 ────────────────────────────────────────────────────────────
    def _frame(self) -> "object":
        """取一张可用的（已去畸变）帧；取不到时抛检测失败。"""
        frame = self.frames.frame(self.cfg.max_frame_age_s,
                                  self.cfg.first_frame_wait_s)
        if frame is None:
            raw, age = self.frames.latest()
            if raw is None:
                raise RuntimeError(
                    "detection_failed: no camera frame received yet")
            raise RuntimeError(
                f"detection_failed: camera frame too stale "
                f"({age:.1f}s > {self.cfg.max_frame_age_s:.0f}s)")
        if self.cfg.debug_frame_dir:
            self._dump_debug_frame(frame)
        return frame

    def _dump_debug_frame(self, frame: "object") -> None:
        """把这一帧落盘，便于排查"找不到目标"（配置了 debug_frame_dir 才做）。"""
        try:
            import cv2
            os.makedirs(self.cfg.debug_frame_dir, exist_ok=True)
            cv2.imwrite(os.path.join(self.cfg.debug_frame_dir,
                                     "frame_%d.jpg" % int(time.time())), frame)
        except Exception as e:  # noqa: BLE001
            log.warning("save detect frame failed: %s", e)

    # ── yolo 路径 ───────────────────────────────────────────────────────
    def _detect_via_yolo(self, object_name: str,
                         exclude_box: bool = True) -> dict:
        frame = self._frame()
        dets = self._yolo_detect_all(frame)
        if dets is None:
            raise RuntimeError("detection_failed: yolo_http failed (frame 1)")
        dets = self._two_frame_consistency(frame, dets)
        if exclude_box:
            dets = self._drop_sorted_in_box(dets)
        if not dets:
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(nothing found on the table)")
        matched = self._match_object(object_name, dets)
        if matched is None:
            found = ", ".join(sorted({str(d.get("object_name")) for d in dets}))
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(table has: {found})")
        log.info("match: user '%s' -> %s (conf=%.2f)",
                 object_name, matched.get("object_name"),
                 float(matched.get("confidence", 0.0)))
        return {
            "success": True,
            "bbox_2d": list(matched.get("bbox_2d") or []),
            "object_center_3d": [],
            "confidence": float(matched.get("confidence", 0.0)),
            "message": f"matched {matched.get('object_name')}",
        }

    def _yolo_detect_all(self, frame: "object") -> Optional[list]:
        """把一帧发给宿主 GPU YOLO 服务（**不带** object_name 过滤）。

        返回检测列表；`[]` = 桌面上什么都没检测到；`None` = 传输/服务错误
        （调用方据此区分"空桌"与"检测器挂了"）。
        """
        import cv2
        ok, enc = cv2.imencode(".jpg", frame)
        if not ok:
            log.warning("yolo: could not JPEG-encode frame")
            return None
        params = {"conf": str(self.cfg.yolo_conf), "iou": str(self.cfg.yolo_iou)}
        url = self.cfg.yolo_url + "?" + urlencode(params)
        req = urllib.request.Request(url, data=enc.tobytes(), method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.cfg.yolo_timeout_s) as r:
                body = json.loads(r.read().decode("utf-8"))
        except Exception as e:  # noqa: BLE001
            log.warning("yolo_http failed: %s", e)
            return None
        if not body.get("success"):
            log.warning("yolo error: %s", body.get("message"))
            return None
        return list(body.get("detections") or [])

    def _two_frame_consistency(self, frame_a: "object", dets_a: list) -> list:
        """两帧一致性：间隔 `two_frame_interval_s` 再拍一帧，只有两帧都出现、
        同类且 IoU > `two_frame_iou` 的检测才保留（真物体稳定，闪烁误检出局）。

        第二帧缺失或第二次 YOLO 调用失败时，回退为单帧结果（只告警）。
        """
        time.sleep(max(0.0, self.cfg.two_frame_interval_s))
        frame_b, _age = self.frames.latest()
        if frame_b is None:
            log.warning("2-frame: no second frame — falling back to single frame")
            return dets_a
        frame_b = self.frames.undistort(frame_b)
        dets_b = self._yolo_detect_all(frame_b)
        if dets_b is None:
            log.warning("2-frame: second yolo call failed — falling back to "
                        "single frame")
            return dets_a
        kept = []
        for d in dets_a:
            name = str(d.get("object_name", "")).strip().lower()
            for e in dets_b:
                if (str(e.get("object_name", "")).strip().lower() == name
                        and self._bbox_iou(list(d.get("bbox_2d") or []),
                                           list(e.get("bbox_2d") or []))
                        > self.cfg.two_frame_iou):
                    kept.append(d)
                    break
        log.info("2-frame consistency: %d/%d kept", len(kept), len(dets_a))
        return kept

    @staticmethod
    def _bbox_iou(a: Sequence[float], b: Sequence[float]) -> float:
        """两个 [x_min, y_min, x_max, y_max, ...] 框的 IoU（第 5 个元素忽略）。"""
        try:
            ax1, ay1, ax2, ay2 = (float(v) for v in a[:4])
            bx1, by1, bx2, by2 = (float(v) for v in b[:4])
        except (TypeError, ValueError, IndexError):
            return 0.0
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        return inter / (area_a + area_b - inter)

    def _match_object(self, user_request: str, detections: list) -> Optional[dict]:
        """用纯文本 LLM 把用户的措辞对到检测出的类名（返回匹配项或 None）。

        类属性表固定：tomato=红、potato=黄、carrot 无颜色别名。
        """
        cfg = self.cfg
        if not cfg.match_base_url or not cfg.match_api_key or not cfg.match_model:
            raise RuntimeError(
                "matcher_llm_failed: 匹配用户请求物品要用大模型，但配置里没给全"
                "（grab_match_llm_base_url/api_key/model）")
        if not detections:
            return None
        det_lines = "; ".join(
            f"{d.get('object_name')}(conf={float(d.get('confidence', 0.0)):.2f})"
            for d in detections)
        prompt = (
            "你是一个抓取目标匹配器。桌面上检测到的物体: " + det_lines + "\n"
            "物体属性: tomato=红色(red); potato=黄色(yellow); "
            "carrot=无颜色别名(只能按名字指代)。\n"
            f"用户请求: {user_request}\n"
            "判断用户请求匹配列表中的哪个物体。只输出JSON: "
            "{\"match\": <object_name 或 null>, \"reason\": \"简短原因\"}\n"
            "规则: 颜色描述按属性表对应(红/red→tomato, 黄/yellow→potato); "
            "请求中出现的名字直接匹配; 属性表外的颜色(如蓝/blue)或与检测列表"
            "无关的物体 → match=null。"
        )
        try:
            parsed = self._chat_completions_json(
                base_url=cfg.match_base_url, api_key=cfg.match_api_key,
                model=cfg.match_model, prompt=prompt,
                timeout_s=cfg.match_timeout_s, what="grab matcher LLM")
        except RuntimeError as e:
            # 这一步失败，是**匹配用的大模型**出了问题（超时 / 连不上 / 返回内容解析不了），
            # 跟"桌上没有这个物体"完全是两回事。文案要点名是模型的问题，
            # robonix 侧读到 `matcher_llm_failed:` 就知道该重试、该查模型，而不是去查桌面。
            raise RuntimeError(
                "matcher_llm_failed: 用大模型匹配用户请求物品时大模型出错"
                f"（模型 {cfg.match_model!r}）：{e}。"
                "这不是桌上没找到物体——重试一次通常能好；反复失败要查这个模型的端点/额度。"
            ) from e
        matched_name = str((parsed or {}).get("match") or "").strip().lower()
        if not matched_name or matched_name == "null":
            return None
        for d in detections:      # 先精确匹配
            if str(d.get("object_name", "")).strip().lower() == matched_name:
                return d
        for d in detections:      # 再子串匹配
            if matched_name in str(d.get("object_name", "")).strip().lower():
                return d
        return None

    # ── vlm 路径 ────────────────────────────────────────────────────────
    def _detect_via_vlm(self, object_name: str,
                        exclude_box: bool = True) -> dict:
        """一次多模态调用：把当前帧和用户请求交给 VLM，直接要像素 bbox。"""
        import cv2
        cfg = self.cfg
        if not cfg.vlm_base_url or not cfg.vlm_api_key or not cfg.vlm_model:
            raise RuntimeError(
                "vlm scheme requires grab_vlm_base_url/api_key/model")

        frame = self._frame()
        ok, enc = cv2.imencode(".jpg", frame)
        if not ok:
            raise RuntimeError("detection_failed: could not JPEG-encode frame")
        b64 = base64.b64encode(enc.tobytes()).decode("ascii")
        h, w = frame.shape[:2]
        prompt = (
            "你是抓取目标定位器。图片是一张桌面俯视图,宽%d像素,高%d像素。\n"
            f"用户请求: {object_name}\n"
            "在图中定位用户请求的物体,只输出JSON: "
            "{\"present\": true, \"bbox\": [x_min, y_min, x_max, y_max]} 或 "
            "{\"present\": false}。bbox 为像素坐标,必须在图像范围内,"
            "恰好框住目标;图中没有该物体时输出 {\"present\": false}。"
        ) % (w, h)
        payload = json.dumps({
            "model": cfg.vlm_model,
            "messages": [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
            ]}],
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
        }).encode("utf-8")
        url = cfg.vlm_base_url.rstrip("/") + "/chat/completions"
        req = urllib.request.Request(
            url, data=payload, method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {cfg.vlm_api_key}"})
        try:
            with urllib.request.urlopen(req, timeout=cfg.vlm_timeout_s) as r:
                body = json.loads(r.read().decode("utf-8"))
            parsed = json.loads(str(body["choices"][0]["message"]["content"]))
        except Exception as e:  # noqa: BLE001
            log.warning("grab VLM failed: %s", e)
            raise RuntimeError(f"grab VLM failed: {e}")

        if not parsed.get("present", False):
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(VLM found nothing)")
        try:
            bbox = [float(x) for x in parsed["bbox"]]
            assert len(bbox) == 4
        except (KeyError, TypeError, ValueError, AssertionError) as e:
            log.warning("grab VLM bad bbox: %r (%s)", parsed.get("bbox"), e)
            raise RuntimeError(f"grab VLM returned bad bbox: {e}")

        if exclude_box:
            self._reject_if_in_box(object_name, bbox)
        log.info("vlm locate: user '%s' -> bbox %s", object_name, bbox)
        return {
            "success": True,
            "bbox_2d": bbox,
            "object_center_3d": [],
            "confidence": 1.0,
            "message": f"vlm located {object_name}",
        }

    # ── 盒内排除 ────────────────────────────────────────────────────────
    def _drop_sorted_in_box(self, dets: list) -> list:
        """丢掉"已经在盒子里"的检测（落盒点半径内），不进入措辞匹配。"""
        if not dets or not self.drop_zone.enabled:
            return dets
        kept = []
        for d in dets:
            inside, dist = self.drop_zone.bbox_inside(
                self.homography, list(d.get("bbox_2d") or []))
            if inside:
                log.info("box exclusion: %s is %.3fm from box drop point "
                         "— already-sorted, dropped before LLM match",
                         d.get("object_name"), dist)
                continue
            kept.append(d)
        return kept

    def _reject_if_in_box(self, object_name: str, bbox: Sequence[float]) -> None:
        """单个 bbox 的盒内排除（vlm 路径）：在盒内 -> 报"未检测到"。"""
        inside, dist = self.drop_zone.bbox_inside(self.homography, bbox)
        if inside:
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(only an already-sorted item in the box, "
                f"{dist:.3f}m from drop point)")

    # ── HTTP 小工具 ─────────────────────────────────────────────────────
    @staticmethod
    def _chat_completions_json(*, base_url: str, api_key: str, model: str,
                               prompt: str, timeout_s: float,
                               what: str) -> Optional[dict]:
        """发一次纯文本 chat/completions，返回解析后的 JSON（失败抛 RuntimeError）。"""
        payload = json.dumps({
            "model": model,
            "messages": [{"role": "user", "content": prompt}],
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
        }).encode("utf-8")
        url = base_url.rstrip("/") + "/chat/completions"
        req = urllib.request.Request(
            url, data=payload, method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {api_key}"})
        try:
            with urllib.request.urlopen(req, timeout=timeout_s) as r:
                body = json.loads(r.read().decode("utf-8"))
            return json.loads(str(body["choices"][0]["message"]["content"]))
        except Exception as e:  # noqa: BLE001
            log.warning("%s failed: %s", what, e)
            raise RuntimeError(f"{what} failed: {e}")
