import sys
import cntk
import numpy as np
from PIL import Image, ImageDraw
from object_detection import ObjectDetection

MODEL_FILENAME = 'model.onnx'
LABELS_FILENAME = 'labels.txt'

class CNTKObjectDetection(ObjectDetection):
    """Object Detection class for CNTK
    """
    def __init__(self, model, labels):
        super(CNTKObjectDetection, self).__init__(labels)
        self.model = model
        
    def predict(self, preprocessed_image):
        inputs = np.array(preprocessed_image, dtype=np.float32)[:,:,(2,1,0)] # RGB -> BGR
        inputs = np.ascontiguousarray(np.rollaxis(inputs, 2))

        outputs = self.model.eval({self.model.arguments[0]: [inputs]})
        return np.squeeze(outputs).transpose((1,2,0))

def main(image_filename):
    model = cntk.Function.load(MODEL_FILENAME, format=cntk.ModelFormat.ONNX)
    
    # Load labels
    with open(LABELS_FILENAME, 'r') as f:
        labels = [l.strip() for l in f.readlines()]

    od_model = CNTKObjectDetection(model, labels)

    image = Image.open(image_filename)
    predictions = od_model.predict_image(image)
    print(predictions)
    
if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print('USAGE: {} image_filename'.format(sys.argv[0]))
    else:
        main(sys.argv[1])
