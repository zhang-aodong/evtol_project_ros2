"""Write the Chinese handover from measured acceptance results."""
from pathlib import Path
import json
import sys

root=Path(sys.argv[1]);base=Path(__file__).resolve().parent
audit=json.loads((root/'docs/perception_recall_acceptance_20261119.json').read_text(encoding='utf8'))
labels=['surround_final_radial_v2','surround_prefetch_holdout','surround_monotonic_original',
        'surround_escape_holdout','surround_targeted_holdout','surround_rotated_holdout',
        'surround_optimized_holdout','surround_optimized_verification']
rows={r['label']:r for r in audit['runs']}
def percent(value):return '无有效样本' if value is None else f'{value*100:.2f}%'
lines=['\n## 本轮真实验收结果\n',
    '**结论：尚未稳定达到 95% 验收要求。** 本次交付包含已部署的真实修复、原始日志与可重复验收脚本；不能将单次较高结果宣传为整体通过。\n',
    '| 测试 | 及时成功/完成事件 | 及时率 | 时刻召回 | 精确率 | 真值覆盖 | 说明 |',
    '|---|---:|---:|---:|---:|---:|---|']
notes={
 'surround_final_radial_v2':'历史测试，本机大部分在地面，不作为有效验收',
 'surround_prefetch_holdout':'历史较高及时率，但真值覆盖不足、本机未到目标高度',
 'surround_monotonic_original':'用户原航线可升空；仅1次事件，样本不足',
 'surround_escape_holdout':'起飞脱离修复后，仍有迟检',
 'surround_targeted_holdout':'串行测距对照，不采用',
 'surround_rotated_holdout':'姿态补偿版，仍有迟检',
 'surround_optimized_holdout':'最终代码，32机，28→9m，方位偏移7°',
 'surround_optimized_verification':'最终代码，32机，30→10m，方位偏移19°'}
for label in labels:
 r=rows[label];enc=r.get('encounters',{})
 lines.append(f"| {label} | {enc.get('success',0)}/{r.get('completed_encounters',0)} | {percent(r.get('timely_success_rate'))} | {percent(r.get('recall'))} | {percent(r.get('precision'))} | {percent(r.get('scoring_valid_fraction'))} | {notes[label]} |")
lines.extend(['\n最终代码已通过 **84 项相关回归测试**，包括六方向覆盖、真实视觉与深度双重证据、射线预算、姿态补偿、时间递增、起飞紧急避让、地面约束、历史 IMU 恢复和控制台配置。构建与测试日志位于 `reports/perception_optimized_*`。',
 '\n## 仍未解决的限制及下一步\n',
 '1. 密集小目标仍存在首次分割缺失、深度未命中及融合过期；当前图像到输出可能超过 300 ms，使 0.5 s 首次发现窗口只剩较少的有效机会。单纯修改置信度、延长历史或扩大关联门限不能作为解决方案。',
 '2. 原训练补采图为 512×512，训练使用 imgsz=768；插值训练不能补出原图不存在的目标细节。环视实际采集为 768×768。下一步应补采原生 768/更高像素的侧向、俯仰、倾斜、遮挡和远距小无人机，按独立场景划分训练/验证/最终测试，比较事件指标后再替换模型。离线 class-6 像素召回约90.3%，不能当作事件95%的证据。',
 '3. 真值全部32架同时出现采样缺口，表明负载下仍有整批消息/本机状态中断；最终报告保留缺口，不将未知帧标为检测成功。下一步可将原生kinematics写入带序号的共享内存环形历史，以恢复真实采样，继续保持0.15 s最大插值间隔与0.4 s评估缓冲。',
 '4. 相机原20 Hz链路本轮平均约18–19 Hz，雷达约19.8 Hz、IMU约99.5 Hz；相机偶见重排窗口超限。未宣称全部传感器频率严格达标。',
 '\n## 文件与恢复\n',
 '- 中文说明：`docs/perception_recall_20261119.md`。全部成功与失败测试汇总：`docs/perception_recall_acceptance_20261119.json`。',
 '- 最终原始结果：`reports/perception_surround_optimized_holdout/result.json`、`reports/perception_surround_optimized_verification/result.json`；各文件的 run 字段指向独立评估原始 metrics/matches/encounters。',
 '- 源码及模型 SHA256：`docs/perception_recall_deployment_20261119.json`。旧文件备份：`backups/perception_recall_20261119_140958`。',
 '- 用户的 console.local.json、原航点和 models/best.pt 保留；测试 finally 恢复 session_origin，并验证用户配置 hash。每次结果的 owned_remaining 应为空。',
 '- 已完成本机 UE 插件补丁与重新编译，复制到另一台机器仍需按前文步骤应用并编译插件；本轮新增原生接口是项目依赖。',
 '\n日常启动：运行 `./scripts/start_console.ps1 -NoBrowser -BridgePort 8766`，Foxglove 连接 `ws://127.0.0.1:8766`，由现有控制台按钮分别启动 UE 和 ROS。启动新的管理器前必须先关闭已有管理器，避免端口和控制权冲突。'])
text=(base/'docs_draft.md').read_text(encoding='utf8')+'\n'+'\n'.join(lines)+'\n'
(root/'docs/perception_recall_20261119.md').write_text(text,encoding='utf8')
print(root/'docs/perception_recall_20261119.md')
