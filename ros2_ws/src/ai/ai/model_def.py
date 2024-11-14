from torchvision.models.detection.ssdlite import SSDLite320_MobileNet_V3_Large_Weights, ssdlite320_mobilenet_v3_large
import torch
import cv2
import numpy as np


class DetectionWrapper:
    def __init__(self):
        # weights setup
        self.weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
        self.preprocess = self.weights.transforms()

        # model setup
        self.model = ssdlite320_mobilenet_v3_large(weights=self.weights)
        self.model.eval()

    def filter_objects(self, cls, thresh):
        output = []

        for obj in self.output:
            if obj["class"] == cls and obj["score"] >= thresh:
                output.append({
                    "class": obj["class"],
                    "bbox": obj["bbox"]
                })

        return output

    def detect_img(self, img):
        torch_img = torch.from_numpy(img).permute(2, 0, 1)

        batch = self.preprocess(torch_img)
        batch = torch.unsqueeze(batch, 0)

        prediction = self.model(batch)[0]
        boxes = prediction["boxes"]
        labels = prediction["labels"]
        scores = prediction["scores"]

        self.output = []

        for i, label in enumerate(labels):
            bbox = np.round(boxes[i].detach().numpy()).astype(np.uint32)
            score = scores[i]

            self.output.append({
                "bbox": bbox,
                "score": float(score),
                "class": int(label)
            })

if __name__ == "__main__":
    cam = cv2.VideoCapture(0)
    model = DetectionWrapper()

    while True:
        ret, frame = cam.read()

        model.detect_img(frame)