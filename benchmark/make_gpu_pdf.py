from pathlib import Path
import json,hashlib,io,re
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.colors import HexColor
from pypdf import PdfReader
from PIL import Image
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output';PD=OUT/'pdf';PD.mkdir(exist_ok=True)
m=json.loads((OUT/'manifest.json').read_text());rows=m['scenes'];assert len(rows)==83
pdfmetrics.registerFont(TTFont('CN','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'))
PDF=PD/'benchmark_gpu_five_camera_views.pdf';W,H=1190,842
c=canvas.Canvas(str(PDF),pagesize=(W,H),pageCompression=1)
c.setTitle('Benchmark GPU 重建 - 83 场景五路相机图册');c.setAuthor('Benchmark simulation workspace')
navy='#132A3B';muted='#587080';accent='#087E8B'
def txt(x,y,t,size=14,color=navy):
 c.setFillColor(HexColor(color))
 for run in re.findall(r'[\x00-\xff]+|[^\x00-\xff]+',t):
  font='Helvetica' if ord(run[0])<256 else 'CN'
  c.setFont(font,size);c.drawString(x,y,run);x+=pdfmetrics.stringWidth(run,font,size)
def base(page,label):
 c.setFillColor(HexColor('#F4F7F9'));c.rect(0,0,W,H,stroke=0,fill=1)
 txt(34,24,'BENCHMARK  /  RTX 5090  /  ISAAC SIM 6.0.1',10,muted)
 txt(920,24,f'{label}  |  {page:02d} / 85',10,muted)
def image(p,x,y,w,h):
 b=io.BytesIO()
 with Image.open(p) as im:im.convert('RGB').save(b,format='JPEG',quality=94,subsampling=0)
 b.seek(0);c.drawImage(ImageReader(b),x,y,w,h,preserveAspectRatio=True,anchor='c')
base(1,'GPU 新渲染');txt(40,770,'BENCHMARK / GPU REBUILD',22,accent)
txt(40,700,'83 个场景 · 五路相机图册',40)
txt(40,654,'RTX 5090  |  Isaac Sim 6.0.1  |  1280 × 720 原始画面',19,muted)
image(OUT/'views/01_pick_spoon/Overview.png',40,200,730,411)
for i,t in enumerate(['83 个注册场景','415 张五路相机 RGB','83 张整体视图','415 份理想几何深度','全部重新 GPU 渲染','保留新版 YCB 扫描纹理']):txt(810,574-i*48,t,19,accent if i==4 else navy)
notes=['采集状态：场景初始姿态，时间线暂停；本册用于检查布局、视野及视觉资产。','图像不是任务成功轨迹；不代表所有场景已完成物理动力学验收。','左右方位以机器人为准；青色线框为目标姿态标记。深度为理想射线距离。','YCB 扫描资产：YCB Object and Model Set / UC Berkeley / Google，CC BY 4.0。','来源及逐项署名保留于 assets/external/ycb/ATTRIBUTION.md；采集与哈希见 output/manifest.json。']
for i,t in enumerate(notes):txt(40,160-i*25,t,13,muted)
c.showPage()
base(2,'场景目录');txt(34,786,'场景目录',30);txt(34,750,'每页包含整体视图及五路相机；页码可点击跳转。',14,muted)
for i,r in enumerate(rows):
 col=i//28;j=i%28;x=34+col*385;y=714-j*23
 title=f'{i+1:02d}  '+r['title'];size=11.5
 while sum(pdfmetrics.stringWidth(run,'Helvetica' if ord(run[0])<256 else 'CN',size) for run in re.findall(r'[\x00-\xff]+|[^\x00-\xff]+',title))>325:size-=.2
 txt(x,y,title,size);txt(x+339,y,str(i+3),11,accent);c.linkRect('',r['id'],(x,y-3,x+365,y+16),relative=0,thickness=0)
c.showPage()
views=[('Overview','整体视图'),('Head','头部相机'),('LeftWrist','左腕相机'),('RightWrist','右腕相机'),('LeftSide','桌面左侧相机'),('RightSide','桌面右侧相机')]
for i,r in enumerate(rows):
 base(i+3,'五路相机');c.bookmarkPage(r['id']);c.addOutlineEntry(r['title'],r['id'],level=0)
 txt(34,783,f'{i+1:02d} / '+r['title'],27);txt(34,747,r['id'],13,accent)
 txt(34,720,'GPU RTX 实时光线追踪  |  初始场景静态采集  |  各视图 1280 × 720',13,muted)
 for j,(key,label) in enumerate(views):
  x=34+(j%3)*385;y=427-(j//3)*282
  txt(x,y+221,label,17)
  p=OUT/'views'/r['id']/(key+'.png');assert p.exists()
  with Image.open(p) as im:assert im.size==(1280,720)
  if key!='Overview':assert hashlib.sha256(p.read_bytes()).hexdigest()==r['views'][key]['sha256']
  image(p,x,y,352,198)
 txt(34,94,'场景入口：'+r['scene'],12,muted)
 txt(34,70,'同一场景状态的五个固定机位；腕部画面遮挡按原始相机安装保留。',12,muted)
 c.showPage()
c.save();reader=PdfReader(PDF);assert len(reader.pages)==85
assert all(p.extract_text().strip() for p in reader.pages)
(OUT/'pdf_validation.json').write_text(json.dumps({'pages':85,'scenes':83,'camera_images':415,'overview_images':83,'all_image_hashes_verified':True,'pdf_bytes':PDF.stat().st_size},indent=2))
print(PDF,PDF.stat().st_size,flush=True)
