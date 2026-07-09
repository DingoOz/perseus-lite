import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10

OK = '#3f8a6f'
BLOCKED = '#b8503a'
SENSOR = '#c9a227'
PENDING = '#8a8371'
NA = '#948c78'

# ---------------------------------------------------------------------------
# Chart 1: stage-by-stage outcome
# ---------------------------------------------------------------------------
stages = [
    ('0-1', 'Env stand-up /\nGXF binary check', 'ok'),
    ('2-3', 'NITROS core build', 'ok'),
    ('4', 'AprilTag detection', 'ok'),
    ('5', 'DNN inference\n(TensorRT)', 'ok'),
    ('5f', 'YOLOv8 object\ndetection', 'ok'),
    ('6', 'Visual SLAM\n(cuVSLAM)', 'blocked'),
    ('7', 'U-Net\nsegmentation', 'ok'),
    ('8', 'CenterPose 3D\npose estimation', 'ok'),
    ('9', 'H.264 hardware\ncodec', 'blocked'),
    ('10', 'Combined multi-\npipeline test', 'ok'),
    ('11', 'ESS stereo\ndepth', 'blocked'),
    ('12', 'Occupancy grid\nlocalizer (lidar)', 'ok'),
]

colors = {'ok': OK, 'blocked': BLOCKED}
fig, ax = plt.subplots(figsize=(7.0, 4.6))
y_pos = range(len(stages))
bar_colors = [colors[s[2]] for s in stages]
bar_vals = [1 for _ in stages]
labels = [f"Stage {s[0]}" for s in stages]
descs = [s[1] for s in stages]

bars = ax.barh(list(y_pos)[::-1], bar_vals, color=bar_colors, height=0.68, edgecolor='white', linewidth=0.6)
ax.set_yticks(list(y_pos)[::-1])
ax.set_yticklabels(labels, fontsize=9)
ax.set_xlim(0, 1.42)
ax.set_ylim(-1.1, len(stages) - 0.3)
ax.set_xticks([])
for spine in ['top', 'right', 'bottom']:
    ax.spines[spine].set_visible(False)
ax.spines['left'].set_color('#999999')

for yp, desc in zip(list(y_pos)[::-1], descs):
    ax.text(1.05, yp, desc.replace('\n', ' '), va='center', ha='left', fontsize=8, color='#333333')

legend_handles = [
    plt.Rectangle((0, 0), 1, 1, color=OK, label='Verified working'),
    plt.Rectangle((0, 0), 1, 1, color=BLOCKED, label='Built, blocked at runtime (root-caused, not a code bug)'),
]
ax.legend(handles=legend_handles, loc='upper center', bbox_to_anchor=(0.5, -0.02),
          frameon=False, fontsize=8, ncol=1)
ax.set_title('Stage-by-stage outcome (12 stages, Orin Nano + JetPack 7)', fontsize=11, pad=12)
plt.tight_layout()
plt.savefig('stage_status.pdf')
plt.close()

# ---------------------------------------------------------------------------
# Chart 2: full Isaac ROS map breakdown (donut)
# ---------------------------------------------------------------------------
map_labels = [
    'Verified working\nnatively (7)',
    'Built, blocked\nat runtime (3)',
    'Needs stereo/\ndepth camera (3)',
    'Relevant,\nuntried (9)',
    'Not applicable\nto this robot (7)',
]
map_values = [7, 3, 3, 9, 7]
map_colors = [OK, BLOCKED, SENSOR, PENDING, NA]

fig, ax = plt.subplots(figsize=(6.4, 5.2))
wedges, texts = ax.pie(
    map_values, colors=map_colors, startangle=90, counterclock=False,
    wedgeprops=dict(width=0.42, edgecolor='white', linewidth=1.5),
)
ax.legend(wedges, map_labels, loc='center left', bbox_to_anchor=(1.0, 0.5),
          frameon=False, fontsize=9, handlelength=1.2)
ax.text(0, 0, '29\nrepos', ha='center', va='center', fontsize=15, fontweight='bold', color='#333333')
ax.set_title('Full Isaac ROS GEM repository map', fontsize=11, pad=14)
plt.tight_layout()
plt.savefig('map_breakdown.pdf')
plt.close()

# ---------------------------------------------------------------------------
# Chart 3: TensorRT engine build time by model
# ---------------------------------------------------------------------------
build_times = [41.8, 5.9, 182.0, 287.0]
model_sizes = [14, 10, 124, 79]  # MB onnx file size
models = [
    f'MobileNetV2\n(Stage 5)\n{model_sizes[0]} MB onnx',
    f'YOLOv8s\n(Stage 5f)\n{model_sizes[1]} MB onnx',
    f'U-Net\n(Stage 7)\n{model_sizes[2]} MB onnx',
    f'CenterPose\n(Stage 8)\n{model_sizes[3]} MB onnx',
]

fig, ax1 = plt.subplots(figsize=(6.6, 4.3))
x = range(len(models))
bars = ax1.bar(x, build_times, color='#5c7a99', width=0.5, edgecolor='white')
ax1.set_ylabel('TensorRT engine build time (s)', fontsize=9)
ax1.set_xticks(x)
ax1.set_xticklabels(models, fontsize=8)
for spine in ['top', 'right']:
    ax1.spines[spine].set_visible(False)
for xi, bt in zip(x, build_times):
    ax1.text(xi, bt + 8, f'{bt:.0f}s', ha='center', fontsize=8.5, color='#333333')
ax1.set_ylim(0, 330)
ax1.set_title('First-run TensorRT engine build time by model', fontsize=11, pad=12)
plt.tight_layout()
plt.savefig('engine_build_times.pdf')
plt.close()

print("charts written")
