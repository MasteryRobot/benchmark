"""Compose all ten scenes and five original camera views into a full-resolution atlas.

Original RGB pixels are pasted without cropping or resampling. Existing Chinese
label strips are reused when this portable environment has no CJK fonts.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
CAMERAS = [
    ('head', '头部相机', 'HEAD', '#187bb9', (1460, 163, 1830, 211)),
    ('leftwrist', '左腕相机', 'LEFT WRIST', '#d17b15', (2740, 1958, 3110, 2006)),
    ('rightwrist', '右腕相机', 'RIGHT WRIST', '#8258b3', (180, 1958, 550, 2006)),
    ('leftside', '桌左侧相机', 'LEFT SIDE', '#159374', (2790, 1068, 3210, 1116)),
    ('rightside', '桌右侧相机', 'RIGHT SIDE', '#d25260', (170, 1068, 590, 1116)),
]
FRAME_W, FRAME_H = 1280, 720
MARGIN, GAP = 64, 24
WIDTH = 2 * MARGIN + 5 * FRAME_W + 4 * GAP
ROW_HEADER, ROW_FOOTER = 80, 30
ROW_HEIGHT = ROW_HEADER + FRAME_H + ROW_FOOTER
HEADER, FOOTER = 368, 116
BACKGROUND = '#eef2f6'
INK, MUTED = '#193044', '#53687b'


def font(size, bold=False):
    import matplotlib
    folder = Path(matplotlib.get_data_path()) / 'fonts/ttf'
    return ImageFont.truetype(str(folder / ('DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf')), size)


def paste_label(canvas, source, box, xy, scale=1.0):
    label = source.crop(box).convert('RGB')
    if scale != 1:
        label = label.resize((round(label.width * scale), round(label.height * scale)), Image.Resampling.LANCZOS)
    canvas.paste(label, xy)


def camera_header(canvas, label_source, y):
    draw = ImageDraw.Draw(canvas)
    for index, (_, _, english, color, crop) in enumerate(CAMERAS):
        x = MARGIN + index * (FRAME_W + GAP)
        draw.rounded_rectangle((x, y, x + FRAME_W, y + 112), radius=12, fill='white')
        draw.rectangle((x, y, x + FRAME_W, y + 8), fill=color)
        draw.text((x + 24, y + 27), f'{index + 1:02d}', font=font(34, True), fill=color)
        paste_label(canvas, label_source, crop, (x + 102, y + 20))
        draw.text((x + 560, y + 34), english, font=font(30, True), fill=INK)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview-width', type=int, default=2200)
    args = parser.parse_args()
    if not 800 <= args.preview_width <= WIDTH:
        parser.error('--preview-width must be between 800 and full image width')
    tasks = json.loads((ROOT / 'tasks/index.json').read_text())
    if len(tasks) != 10 or len({task['id'] for task in tasks}) != 10:
        raise ValueError('Expected ten distinct task scenes')
    output = ROOT / 'images'
    output.mkdir(exist_ok=True)
    with Image.open(output / 'task_scenes_overview.png') as image:
        task_labels = image.convert('RGB')
    with Image.open(output / 'camera_views_overview.png') as image:
        camera_labels = image.convert('RGB')
    height = HEADER + len(tasks) * ROW_HEIGHT + FOOTER
    atlas = Image.new('RGB', (WIDTH, height), BACKGROUND)
    draw = ImageDraw.Draw(atlas)
    # Reuse the original Chinese title without changing its meaning.
    paste_label(atlas, task_labels, (64, 25, 1800, 112), (MARGIN, 32), 1.35)
    draw.text((MARGIN, 159), '10 SCENES  /  5 CAMERAS  /  50 ORIGINAL RGB VIEWS', font=font(36, True), fill=INK)
    draw.text((MARGIN, 213), 'Full frames at 1280 x 720 each  |  Left / right follow the robot  |  Existing simulation renders', font=font(29), fill=MUTED)
    camera_header(atlas, camera_labels, 254)
    manifest = {
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'description': '十场景 × 五路相机；原始视野完整保留。',
        'source_type': 'existing_saved_simulation_renders',
        'new_physics_or_render_execution': False,
        'layout': {'rows': 10, 'columns': 5, 'width': WIDTH, 'height': height},
        'camera_order': [{'name': c[0], 'label': c[1]} for c in CAMERAS],
        'source_frame_size': [FRAME_W, FRAME_H],
        'source_frame_resampled': False, 'source_frame_cropped': False,
        'label_source': ['images/task_scenes_overview.png', 'images/camera_views_overview.png'],
        'frames': [], 'scene_panels': [],
    }
    for row_index, task in enumerate(tasks):
        row = Image.new('RGB', (WIDTH, ROW_HEIGHT), BACKGROUND)
        rd = ImageDraw.Draw(row)
        rd.rounded_rectangle((MARGIN - 12, 7, WIDTH - MARGIN + 12, ROW_HEIGHT - 10), radius=14, fill='white')
        # The original gallery has two overview cards before the ten task cards.
        card_index = row_index + 2
        cx, cy = 60 + (card_index % 3) * 1180, 205 + (card_index // 3) * 650
        paste_label(row, task_labels, (cx + 20, cy + 17, cx + 1100, cy + 67), (MARGIN + 12, 18))
        rd.text((MARGIN + 1190, 27), task['id'], font=font(30), fill=MUTED)
        hand = 'LEFT HAND' if task['preferred_hand'] == 'left' else 'RIGHT HAND'
        rd.text((WIDTH - MARGIN - 300, 27), hand, font=font(28, True), fill=MUTED)
        y = HEADER + row_index * ROW_HEIGHT
        for col, (name, label, _, _, _) in enumerate(CAMERAS):
            source = ROOT / 'tasks' / task['id'] / 'images' / (name + '.png')
            with Image.open(source) as image:
                if image.size != (FRAME_W, FRAME_H):
                    raise ValueError(f'Unexpected camera frame size: {source.name}')
                frame = image.convert('RGB')
            x = MARGIN + col * (FRAME_W + GAP)
            row.paste(frame, (x, ROW_HEADER))
            manifest['frames'].append({
                'task': task['id'], 'camera': name, 'camera_label': label,
                'source': str(source.relative_to(ROOT)),
                'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'image_rect': [x, y + ROW_HEADER, FRAME_W, FRAME_H],
            })
        atlas.paste(row, (0, y))
        panel = Image.new('RGB', (WIDTH, ROW_HEIGHT + 148), BACKGROUND)
        camera_header(panel, camera_labels, 16)
        panel.paste(row, (0, 148))
        panel_path = ROOT / 'tasks' / task['id'] / 'images/five_camera_views.png'
        panel.save(panel_path, compress_level=6)
        manifest['scene_panels'].append(str(panel_path.relative_to(ROOT)))
        print(task['id'], '5 views composed', flush=True)
    draw = ImageDraw.Draw(atlas)
    draw.text((MARGIN, height - 91), 'Cyan wireframes indicate target poses. RGB panels show saved scene previews; no new simulation was run for this composition.', font=font(28), fill=MUTED)
    draw.text((MARGIN, height - 48), 'Open the full-resolution PNG to inspect individual views. The camera panels are pixel-identical to the source PNGs.', font=font(27), fill=MUTED)
    atlas_path = output / 'all_tasks_five_camera_views.png'
    atlas.save(atlas_path, compress_level=6)
    preview = atlas.resize((args.preview_width, round(height * args.preview_width / WIDTH)), Image.Resampling.LANCZOS)
    preview_path = output / 'all_tasks_five_camera_views_preview.jpg'
    preview.save(preview_path, quality=93, subsampling=0)
    manifest['atlas'] = str(atlas_path.relative_to(ROOT))
    manifest['preview'] = str(preview_path.relative_to(ROOT))
    (output / 'all_tasks_five_camera_views_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'size': [WIDTH, height], 'frames': len(manifest['frames']), 'atlas': manifest['atlas'], 'preview': manifest['preview']}))


if __name__ == '__main__':
    main()
