"""Ejecutar desde el entorno de Hailo DFC en Ubuntu/WSL."""
from pathlib import Path
import json
import numpy as np
from PIL import Image
from hailo_sdk_client import ClientRunner

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'hef_hailo8_real'
OUT.mkdir(exist_ok=True)
runner = ClientRunner(hw_arch='hailo8')
ends = [f'/model.22/cv{branch}.{scale}/cv{branch}.{scale}.2/Conv'
        for scale in range(3) for branch in (2, 3)]
runner.translate_onnx_model(str(ROOT / 'pilares_v0.onnx'), 'pilares_v0',
                          end_node_names=ends, net_input_shapes={'images': [1, 3, 640, 640]})
runner.save_har(str(OUT / 'pilares_v0_parsed.har'))
hn = runner.get_hn_dict()
(OUT / 'network.json').write_text(json.dumps(hn, indent=2))
config = dict(nms_scores_th=0.2, nms_iou_th=0.7, image_dims=[640, 640],
              max_proposals_per_class=100, classes=2, regression_length=16,
              background_removal=False, bbox_decoders=[])
for stride, reg, cls in [(8, 41, 42), (16, 52, 53), (32, 62, 63)]:
    for index in (reg, cls):
        assert f'pilares_v0/conv{index}' in hn['layers'], f'Falta conv{index}'
    config['bbox_decoders'].append(dict(name=f'pilares_v0/bbox_decoder{reg}',
        stride=stride, reg_layer=f'pilares_v0/conv{reg}', cls_layer=f'pilares_v0/conv{cls}'))
nms = OUT / 'nms_config.json'
nms.write_text(json.dumps(config, indent=2))
script = '\n'.join([
    'normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])',
    *[f'change_output_activation(conv{i}, sigmoid)' for i in (42, 53, 63)],
    f'nms_postprocess("{nms.as_posix()}", meta_arch=yolov8, engine=cpu)',
    'allocator_param(width_splitter_defuse=disabled)',
])
(OUT / 'modelo.alls').write_text(script)
runner.load_model_script(script)
files = sorted(p for p in (ROOT / 'pilares_yolo/images/train').rglob('*')
               if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
assert files, 'No hay imagenes reales de calibracion'
rng = np.random.default_rng(42)
selected = [files[i] for i in rng.choice(len(files), min(256, len(files)), replace=False)]
data = np.empty((len(selected), 640, 640, 3), dtype=np.uint8)
for i, path in enumerate(selected):
    with Image.open(path) as source:
        rgb = source.convert('RGB')
        scale = min(640 / rgb.width, 640 / rgb.height)
        resized = rgb.resize((round(rgb.width * scale), round(rgb.height * scale)), Image.Resampling.BILINEAR)
        canvas = Image.new('RGB', (640, 640), (114, 114, 114))
        canvas.paste(resized, ((640 - resized.width) // 2, (640 - resized.height) // 2))
        data[i] = np.asarray(canvas)
(OUT / 'calibration_files.txt').write_text('\n'.join(str(p.relative_to(ROOT)) for p in selected))
print(f'Calibrando con {len(data)} imagenes reales', flush=True)
runner.optimize(data)
runner.save_har(str(OUT / 'pilares_v0_quant_real.har'))
del data
print('Compilando para Hailo-8 (26 TOPS)', flush=True)
hef = runner.compile()
(OUT / 'pilares_v0.hef').write_bytes(hef)
runner.save_har(str(OUT / 'pilares_v0_compiled.har'))
print(f'HEF generado: {OUT / "pilares_v0.hef"}', flush=True)
