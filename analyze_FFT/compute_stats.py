# -*- coding: utf-8 -*-
"""
FFT 振动数据分析 - 数值计算核心模块
=========================================
将固件 CSV 日志解析为结构化数据，计算每个传感器的统计指标。

输入：COM3.txt / COM28.txt / COM33.txt（固件 AUTO 输出）
输出：打印统计摘要 + 返回 pandas DataFrame

用法：
    py compute_stats.py <COM3.txt> <COM28.txt> <COM33.txt>
    py compute_stats.py <目录>          # 自动找同名 .txt
"""

import sys
import os
import pandas as pd
import numpy as np
from typing import List, Dict, Tuple

# ============================================================
# 通道 → 传感器编号 映射（v1 固件：3设备 × 3通道）
# v2 固件（单通道 A0）需改为：
#   CHANNEL_MAP = {'COM3': {'A0': 1}, 'COM28': {'A0': 2}, 'COM33': {'A0': 3}}
# ============================================================
CHANNEL_MAP = {
    'COM3':  {'A0': 1, 'IO15': 4, 'IO16': 5},
    'COM28': {'A0': 2, 'IO15': 6, 'IO16': 7},
    'COM33': {'A0': 3, 'IO15': 8, 'IO16': 9},
}

FM_COLS   = ['b0', 'b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7']
BAND_NAMES = ['B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B6', 'B7']
# 频段 Hz 范围（与固件 g_BAND_FREQS 一致）
BAND_HZ = ['0-78', '78-117', '117-156', '156-234', '234-312', '312-500', '500-1000', '1000-2000']


# ============================================================
# 第一步：解析固件 CSV 日志
# ============================================================

def parse_log(path: str) -> List[Dict]:
    """
    读取单个设备的日志文件，返回 dict 列表。

    过滤规则：
      - 跳过空行和所有 # 注释行（#CHANNEL 除外）
      - #CHANNEL,xxx → 切换当前通道名
      - 数据行：split(',') ≥ 15 列 且 cols[-1] == '#END'
    """
    port = os.path.splitext(os.path.basename(path))[0]
    rows = []
    cur_ch = None

    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # 通道切换标记
            if line.startswith('#CHANNEL,'):
                # 支持 "#CHANNEL,A0" 和 "#CHANNEL, A0" 两种格式
                parts = line.split(',')
                cur_ch = parts[1].strip() if len(parts) > 1 else None
                continue

            # 跳过其他所有注释行
            if line.startswith('#'):
                continue

            cols = line.split(',')
            if len(cols) < 15 or cols[-1] != '#END':
                continue
            if not cols[0].isdigit():          # 跳过 positionId 非数字的行
                continue

            # 查传感器编号
            sensor_id = CHANNEL_MAP.get(port, {}).get(cur_ch)
            if sensor_id is None:
                continue

            try:
                rows.append({
                    'sensor':  sensor_id,
                    'port':    port,
                    'channel': cur_ch,
                    'state':   cols[1],                        # 'Empty' 或 'Full'
                    'phase':   int(cols[2]),                   # 0=空管 1=满水
                    'peak':    float(cols[3]),                 # 主峰频率 Hz
                    'rms':     float(cols[4]),                 # RMS
                    'std':     float(cols[5]),                 # 标准差
                    **{BAND_NAMES[i]: int(cols[6 + i]) for i in range(8)},
                })
            except (ValueError, IndexError):
                pass

    return rows


def load_all_logs(paths: List[str]) -> pd.DataFrame:
    """加载并合并所有设备日志，返回合并后的 DataFrame。"""
    all_rows = []
    for p in paths:
        rows = parse_log(p)
        print(f'  {os.path.basename(p)}: {len(rows)} 行有效数据', flush=True)
        all_rows.extend(rows)

    df = pd.DataFrame(all_rows)
    print(f'  合计 {len(df)} 行 | 传感器: {sorted(df["sensor"].dropna().unique())}', flush=True)
    return df


# ============================================================
# 第二步：传感器级统计
# ============================================================

def sensor_stats(df: pd.DataFrame) -> pd.DataFrame:
    """
    按 sensor 分组，计算每个传感器 Empty/Full 的：
      - RMS 均值、标准差、样本数
      - 主峰频率均值
      - 差值 E-F、相对区分度 %
    """
    records = []
    for sid in sorted(df['sensor'].dropna().unique()):
        sub = df[df['sensor'] == sid]
        e   = sub[sub['state'] == 'Empty']
        f   = sub[sub['state'] == 'Full']

        er  = e['rms'].mean()  if len(e) else float('nan')
        es  = e['rms'].std()   if len(e) > 1 else 0.0
        fr  = f['rms'].mean()  if len(f) else float('nan')
        fs  = f['rms'].std()   if len(f) > 1 else 0.0

        diff = er - fr
        rel  = (diff / er * 100) if er and er != 0 else float('nan')

        # 主峰频率均值
        ep   = e['peak'].mean() if len(e) else float('nan')
        fp   = f['peak'].mean() if len(f) else float('nan')

        records.append({
            'sensor':          sid,
            'device':          sub['port'].iloc[0]    if len(sub) else '',
            'channel':         sub['channel'].iloc[0]  if len(sub) else '',
            'n_Empty':         len(e),
            'n_Full':          len(f),
            'Empty_RMS':       round(er, 2),
            'Empty_Std':       round(es, 2),
            'Full_RMS':        round(fr, 2),
            'Full_Std':        round(fs, 2),
            'E_minus_F':       round(diff, 2),
            'Diff_pct':        round(rel, 1) if not np.isnan(rel) else float('nan'),
            'Empty_PeakHz':   round(ep, 1) if not np.isnan(ep) else float('nan'),
            'Full_PeakHz':    round(fp, 1) if not np.isnan(fp) else float('nan'),
        })

    return pd.DataFrame(records)


def band_stats(df: pd.DataFrame) -> pd.DataFrame:
    """
    按 sensor × band 分组，计算 Empty/Full 的 8 个频段能量均值及差值。
    """
    records = []
    for sid in sorted(df['sensor'].dropna().unique()):
        sub = df[df['sensor'] == sid]
        e   = sub[sub['state'] == 'Empty']
        f   = sub[sub['state'] == 'Full']
        for i, (col, band) in enumerate(zip(FM_COLS, BAND_NAMES)):
            ev = e[col].mean() if len(e) else 0
            fv = f[col].mean() if len(f) else 0
            records.append({
                'sensor':  sid,
                'band':    band,
                'hz_range': BAND_HZ[i],
                'Empty_E': round(ev, 0),
                'Full_E':  round(fv, 0),
                'E_minus_F': round(ev - fv, 0),
            })
    return pd.DataFrame(records)


def time_stats(df: pd.DataFrame) -> pd.DataFrame:
    """
    按 sensor × state 分组，统计时间域指标（RMS 主峰位置分布等）。
    目前直接复用 sensor_stats 中的 peak 统计，此处预留扩展。
    """
    records = []
    for sid in sorted(df['sensor'].dropna().unique()):
        sub = df[df['sensor'] == sid]
        for state in ['Empty', 'Full']:
            s = sub[sub['state'] == state]
            records.append({
                'sensor':   sid,
                'state':    state,
                'count':    len(s),
                'rms_mean': round(s['rms'].mean(), 2)  if len(s) else float('nan'),
                'rms_min':  round(s['rms'].min(), 2)   if len(s) else float('nan'),
                'rms_max':  round(s['rms'].max(), 2)   if len(s) else float('nan'),
                'std_mean': round(s['std'].mean(), 2)  if len(s) else float('nan'),
            })
    return pd.DataFrame(records)


# ============================================================
# 第三步：打印摘要
# ============================================================

def print_sensor_summary(stats_df: pd.DataFrame):
    """在终端打印传感器统计摘要表格。"""
    print('\n' + '=' * 90)
    print('传感器区分度摘要（按 |E-F| 降序）')
    print('=' * 90)
    hdr = f"{'#':>3} {'设备':<6} {'通道':<6} {'n_E':>4} {'n_F':>4} {'E_RMS':>9} {'F_RMS':>9} {'E-F':>9} {'区分度%':>8} {'评价':>6}"
    print(hdr)
    print('-' * 90)

    df_s = stats_df.copy()
    df_s['abs_diff'] = df_s['E_minus_F'].abs()
    df_s = df_s.sort_values('abs_diff', ascending=False)

    for _, r in df_s.iterrows():
        diff_pct = r['Diff_pct']
        if pd.isna(diff_pct) or abs(diff_pct) < 2:
            tag = '✗ 无'
        elif abs(diff_pct) < 30:
            tag = '⚠ 弱'
        elif abs(diff_pct) < 80:
            tag = '⭐ 中'
        else:
            tag = '⭐⭐ 强'

        print(
            f"{int(r['sensor']):>3} {r['device']:<6} {r['channel']:<6} "
            f"{int(r['n_Empty']):>4} {int(r['n_Full']):>4} "
            f"{r['Empty_RMS']:>9.2f} {r['Full_RMS']:>9.2f} "
            f"{r['E_minus_F']:>9.2f} {diff_pct:>7.1f}% {tag:>6}"
        )
    print('=' * 90)


def print_band_summary(band_df: pd.DataFrame):
    """按传感器打印各频段区分度。"""
    print('\n' + '=' * 70)
    print('频段能量对比（E-F 差值，>0 表示空管能量更高）')
    print('=' * 70)
    for sid in sorted(band_df['sensor'].unique()):
        sub = band_df[band_df['sensor'] == sid]
        print(f'\n  传感器 {int(sid)} ({sub.iloc[0]["device"]}-{sub.iloc[0]["channel"]}):')
        for _, r in sub.iterrows():
            bar_len = int(abs(r['E_minus_F']) / 50)  # 缩放可视化
            sign = '+' if r['E_minus_F'] >= 0 else '-'
            print(f"    {r['band']} {r['hz_range']:>10}Hz  E={r['Empty_E']:>6.0f}  F={r['Full_E']:>6.0f}  {sign}{abs(r['E_minus_F']):>6.0f}  {'█' * bar_len}")
    print('=' * 70)


def print_recommendations(stats_df: pd.DataFrame):
    """打印传感器选型建议。"""
    df_s = stats_df.copy()
    df_s['abs_pct'] = df_s['Diff_pct'].abs()
    df_s = df_s.sort_values('abs_pct', ascending=False)

    print('\n' + '─' * 60)
    print('选型建议')
    print('─' * 60)

    best = df_s[df_s['abs_pct'] >= 80]
    mid  = df_s[(df_s['abs_pct'] >= 20) & (df_s['abs_pct'] < 80)]
    weak = df_s[df_s['abs_pct'] < 20]

    if len(best):
        print('  推荐主传感器（区分度 ≥ 80%）：')
        for _, r in best.iterrows():
            print(f'    传感器 {int(r["sensor"])} ({r["device"]}-{r["channel"]})  区分度 {r["Diff_pct"]:.1f}%  E_RMS={r["Empty_RMS"]}  F_RMS={r["Full_RMS"]}')
    if len(mid):
        print('  可用辅助传感器（20% ≤ 区分度 < 80%）：')
        for _, r in mid.iterrows():
            print(f'    传感器 {int(r["sensor"])} ({r["device"]}-{r["channel"]})  区分度 {r["Diff_pct"]:.1f}%')
    if len(weak):
        print('  不建议使用（区分度 < 20%）：')
        for _, r in weak.iterrows():
            print(f'    传感器 {int(r["sensor"])} ({r["device"]}-{r["channel"]})  区分度 {r["Diff_pct"]:.1f}%')


# ============================================================
# 第四步：导出 CSV（可选）
# ============================================================

def export_csv(df: pd.DataFrame, stats_df: pd.DataFrame, band_df: pd.DataFrame, out_dir: str):
    """将统计结果导出为 CSV 文件。"""
    df.to_csv(os.path.join(out_dir, 'raw_data.csv'), index=False, encoding='utf-8-sig')
    stats_df.to_csv(os.path.join(out_dir, 'sensor_stats.csv'), index=False, encoding='utf-8-sig')
    band_df.to_csv(os.path.join(out_dir, 'band_stats.csv'), index=False, encoding='utf-8-sig')
    print(f'\n  CSV 已导出至: {out_dir}')


# ============================================================
# 主流程
# ============================================================

def compute(paths: List[str], export: bool = False) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    完整计算流程：解析 → 统计 → 打印摘要。

    参数：
        paths  : 3 个日志文件路径
        export : True 则同时导出 CSV

    返回：
        (原始 DataFrame, 传感器统计 DataFrame, 频段统计 DataFrame)
    """
    print('\n[1/3] 解析日志文件...')
    df = load_all_logs(paths)

    print('\n[2/3] 计算传感器统计...')
    stats_df  = sensor_stats(df)
    band_df   = band_stats(df)

    print('\n[3/3] 打印摘要...')
    print_sensor_summary(stats_df)
    print_band_summary(band_df)
    print_recommendations(stats_df)

    if export:
        out_dir = os.path.dirname(paths[0])
        export_csv(df, stats_df, band_df, out_dir)

    return df, stats_df, band_df


def main():
    if len(sys.argv) < 2:
        print('用法:')
        print('  py compute_stats.py <COM3.txt> <COM28.txt> <COM33.txt>')
        print('  py compute_stats.py <目录>          # 自动在该目录找 *_COM3.txt 等')
        sys.exit(1)

    if len(sys.argv) == 2 and os.path.isdir(sys.argv[1]):
        # 目录模式：在指定目录自动查找 COM3.txt / COM28.txt / COM33.txt
        d = sys.argv[1]
        names = ['COM3.txt', 'COM28.txt', 'COM33.txt']
        paths = [os.path.join(d, n) for n in names]
        missing = [n for n, p in zip(names, paths) if not os.path.exists(p)]
        if missing:
            print(f'目录 {d} 中未找到: {missing}')
            sys.exit(1)
    elif len(sys.argv) >= 4 and all(os.path.exists(p) for p in sys.argv[1:4]):
        paths = sys.argv[1:4]
    else:
        print('参数错误，需要 3 个 .txt 文件路径，或 1 个包含这些文件的目录。')
        sys.exit(1)

    compute(paths, export=True)


if __name__ == '__main__':
    main()
