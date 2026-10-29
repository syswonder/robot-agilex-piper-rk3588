# Package upstreams

Upstream repositories are under `https://github.com/syswonder/`. Revisions identify
the source baseline. Runtime changes are described in the final column. Published
package names additionally use `.piper_rk3588`; transport and maintainer metadata
were completed without changing provider or contract IDs.

| Package | Upstream | Revision | Local changes |
| --- | --- | --- | --- |
| `primitives/primitive-agilex-piper-description-rbnx` | `primitive-agilex-piper-description-rbnx` | `b009854` | 无 |
| `primitives/primitive-agilex-piper-arm-rbnx` | `primitive-agilex-piper-arm-rbnx` | `0052f2b` | 新增 CAN 接口的动态发现与拉起，找不到 USB-CAN 时回退基线原逻辑 |
| `primitives/primitive-audio-mic-rbnx` | 官方版是 `primitive-audio-driver-rbnx`（无同名上游） | `6f7b46a` | 从官方版裁出纯麦克风变体（去掉 speaker、改命名空间与契约集）；TODO 换回官方版试跑，注意它的 speaker 会和 `audio_client_bridge` 撞契约 |
| `primitives/primitive-audio-client-bridge-rbnx` | `primitive-audio-client-bridge-rbnx` | `759e3b1` | 无 |
| `primitives/primitive-orbbec-dabai_dcw-camera-rbnx` | `primitive-orbbec-dabai_dcw-camera-rbnx` | `089bf54` | 把 launch 文件名从写死改成配置项，以支持 Gemini 336L |
| `services/service-grasp-pose-rbnx` | `service-grasp-pose-rbnx` | `78a54ac` | 只改注释和文档（说明 `approach_dist` 不是本服务读的参数） |
| `services/service-object-detect-rbnx` | `service-object-detect-rbnx` | `04e5806` | 增加双检测后端（`backend` 字段选 yolo/vlm）；另修红蓝通道互换——收到的图是 RGB 顺序、代码按 BGR 读，改成先转成 BGR |
| `services/service-roboarm-ik-rbnx` | `service-roboarm-ik-rbnx` | `919fdfc` | 无 |
| `services/service-speech` | 无独立仓库；上游是 robonix 主仓的 `services/speech/` | `fb68bc52` | 集成 FunASR（本地模型比腾讯 ASR 服务的可用性更好），另加 ASR 转写日志；依赖里把 mcp 封在 <2（robonix-api 用的是 mcp 1.x 的 API 却没封顶，装到 2.x 服务起不来） |
| `skills/skill-pick-vertical-grasp-piper-rbnx` | 同上 | — | 同上 |
