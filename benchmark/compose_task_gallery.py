"""Create a labeled overview from original simulation renders and reachability results."""
from pathlib import Path
import base64,json
from xml.sax.saxutils import escape
import cairosvg
ROOT=Path(__file__).resolve().parent
W,H=3600,2900
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}"><rect width="100%" height="100%" fill="#eef2f6"/><g font-family="Noto Sans CJK SC, sans-serif">']
def text(x,y,s,size=32,color='#193044',weight=500):svg.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" font-weight="{weight}">{escape(s)}</text>')
text(70,92,'双臂灵巧操作 · 十项训练场景',62,weight=700)
text(70,150,'独立物理场景 / 五路 RGBD / 初始状态与目标姿态定义',32,color='#53687b')
actions={'lift':'抓取并抬起','portrait_to_landscape':'竖屏旋转为横屏','flip':'翻转至另一面','positive_up':'调整至正极向上','upright':'调整为竖直使用姿态','roll':'绕长轴旋转 180°'}
rows=json.loads((ROOT/'tasks/index.json').read_text())
cards=[('整体布局','双臂位于桌面豁口处，五路相机覆盖全局与局部',ROOT/'images/overview.png'),('桌面可达区域','整体前移 0.38 m；采样可达比例 22.2% → 50.8%',ROOT/'tasks/reachability_comparison.png')]
for i,t in enumerate(rows,1):cards.append((f'{i:02d}  {t["title"]}',actions[t['action']]+' · '+('左手' if t['preferred_hand']=='left' else '右手')+'优先',ROOT/'tasks'/t['id']/'images/detail.png'))
for i,(title,subtitle,path) in enumerate(cards):
 x=60+(i%3)*1180;y=205+(i//3)*650
 svg.append(f'<rect x="{x}" y="{y}" width="1120" height="616" rx="16" fill="white"/>')
 text(x+24,y+55,title,38,weight=700);text(x+24,y+100,subtitle,27,color='#587082')
 raw=base64.b64encode(path.read_bytes()).decode();svg.append(f'<image x="{x+14}" y="{y+124}" width="1092" height="478" preserveAspectRatio="xMidYMid meet" href="data:image/png;base64,{raw}"/>')
text(70,2860,'青色线框为目标姿态；已检查场景与静态接触稳定性，尚未实现自动抓取、翻转策略。',29,color='#53687b')
svg.append('</g></svg>');out=ROOT/'images/task_scenes_overview.svg';out.write_text('\n'.join(svg));cairosvg.svg2png(url=str(out),write_to=str(out.with_suffix('.png')))
print(out.with_suffix('.png'))
