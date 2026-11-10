"""CPU people count for a fixed-shape, raw-output Ultralytics ONNX export."""
import ast


class OnnxPeople:
    def __init__(self, path):
        import onnxruntime

        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(str(path), sess_options=options,
                                                    providers=['CPUExecutionProvider'])
        image, = self.session.get_inputs()
        batch, channels, self.height, self.width = image.shape
        if batch != 1 or channels != 3 or not all(type(v) is int and v > 0 for v in (self.height,self.width)):
            raise ValueError('人数模型需要固定尺寸的 ONNX 输入')
        self.input = image.name
        metadata = self.session.get_modelmeta().custom_metadata_map
        labels = ast.literal_eval(metadata.get('names', '{}'))
        self.classes = len(labels)
        self.people = [int(key) for key,name in labels.items() if str(name).casefold()=='person']
        if not self.people:
            raise ValueError('当前 ONNX 模型缺少 person 类别')

    def count(self, frame):
        import cv2
        import numpy as np

        height,width = frame.shape[:2]
        scale = min(self.width/width, self.height/height)
        width,height = round(width*scale),round(height*scale)
        image = cv2.resize(frame,(width,height),interpolation=cv2.INTER_LINEAR)
        left,top = (self.width-width)//2,(self.height-height)//2
        image = cv2.copyMakeBorder(image,top,self.height-height-top,left,self.width-width-left,
                                    cv2.BORDER_CONSTANT,value=(114,114,114))
        tensor = np.ascontiguousarray(image[:,:,::-1].transpose(2,0,1)[None], dtype=np.float32)/255.
        prediction = self.session.run(None,{self.input:tensor})[0]
        if prediction.ndim != 3 or prediction.shape[:2] != (1,4+self.classes):
            raise ValueError('人数模型需要未内置 NMS 的 YOLO 输出')
        rows = prediction[0].T
        scores = rows[:,4:].max(axis=1)
        selected = (scores>.3) & np.isin(rows[:,4:].argmax(axis=1),self.people)
        boxes = rows[selected,:4].copy()
        if not len(boxes):
            return 0
        boxes[:,:2] -= boxes[:,2:]/2
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores[selected].tolist(), .3, .7)
        return min(300, np.asarray(indices).size)
