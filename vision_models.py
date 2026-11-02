"""Bundled trained models and explicit local sample checks; never opens a camera."""
import copy
import json
from pathlib import Path
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parent
MODELS=ROOT/'models'
FILES={'person':'yolo11n.pt','ppe':'ppe.pt','counting':'yolo12n.pt'}
TITLES={'person':'人员跟随 · YOLO11n','ppe':'安全帽与防护用品 · 原参考项目 PPE','counting':'人数统计 · 用户提供 YOLO12n'}


def default_path(kind):
    return MODELS/FILES[kind]


def catalog():
    metadata=json.loads((MODELS/'sources.json').read_text()) if (MODELS/'sources.json').is_file() else dict(models=[])
    sources={m['id']:m for m in metadata['models']}
    return [dict(id=key,title=TITLES[key],file=filename,installed=default_path(key).is_file(),
                 source=sources.get(key,{}).get('source'),sha256=sources.get(key,{}).get('sha256'),
                 labels=sources.get(key,{}).get('classes',[]),license=sources.get(key,{}).get('license'))
            for key,filename in FILES.items()]


def detections(model,frame,confidence=.5,size=416,classes=None):
    rows=[]
    for result in model.predict(frame,conf=confidence,imgsz=size,classes=classes,verbose=False,device='cpu'):
        if result.boxes is None:continue
        for box in result.boxes:
            coordinates=[float(v) for v in box.xyxy[0]]
            rows.append(dict(class_id=int(box.cls[0]),class_name=model.names[int(box.cls[0])],
                             confidence=float(box.conf[0]),**dict(zip(['x1','y1','x2','y2'],coordinates))))
    return rows


class ModelChecks:
    def __init__(self,data):
        self.directory=Path(data)/'vision-checks';self.lock=threading.Lock();self.models={}

    def run(self,kind,progress):
        if kind not in FILES:raise ValueError('模型类型必须为 person、ppe 或 counting')
        from ultralytics import YOLO
        import cv2
        with self.lock:
            progress(.1,'正在加载已下载的预训练模型')
            if kind not in self.models:self.models[kind]=YOLO(str(default_path(kind)))
            model=self.models[kind]
            frame=cv2.imread(str(MODELS/'samples/persons.jpg'))
            if frame is None:raise ValueError('本机公开验证样例缺失')
            # Warm up separately so the reported timing is one complete inference.
            options=dict(confidence=.3,size=480,classes=[i for i,n in model.names.items() if str(n).lower()=='person']) if kind=='counting' else {}
            detections(model,frame,**options)
            begin=time.monotonic();rows=detections(model,frame,**options);elapsed=(time.monotonic()-begin)*1000
            progress(.8,'正在生成本机检测结果')
            for row in rows:
                p1=tuple(round(row[k]) for k in ['x1','y1']);p2=tuple(round(row[k]) for k in ['x2','y2'])
                color=(60,80,225) if row['class_name']=='NO-Hardhat' else (100,180,60)
                cv2.rectangle(frame,p1,p2,color,2)
                cv2.putText(frame,row['class_name']+' %.2f'%row['confidence'],(p1[0],max(16,p1[1]-5)),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2)
            self.directory.mkdir(parents=True,exist_ok=True);key=uuid.uuid4().hex
            if not cv2.imwrite(str(self.directory/(key+'.jpg')),frame):raise ValueError('未能生成检测预览')
            counts={name:sum(r['class_name']==name for r in rows) for name in sorted({r['class_name'] for r in rows})}
            return dict(ok=True,kind=kind,source='公开图片本机推理；非机器人相机',counts=counts,
                        detections=rows,infer_ms=round(elapsed,1),image='/api/vision/check-image/'+key,
                        model=str(default_path(kind)),labels=copy.deepcopy(model.names))
