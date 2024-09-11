import cv2
import numpy as np
import argparse

# 0 is background
# 1 is ground
# 2 is rocks
# 3 is shadows

def read_yolo_label(label_path, img_shape):
    masks = []
    classes = []
    with open(label_path, 'r') as f:
        for line in f:
            data = list(map(float, line.strip().split()))
            cls = int(data[0])
            polygon = np.array(data[1:]).reshape(-1, 2)
            polygon[:, 0] *= img_shape[1]  # scale x
            polygon[:, 1] *= img_shape[0]  # scale y
            polygon = polygon.astype(np.int32)
            masks.append(polygon)
            classes.append(cls)
    return masks, classes

def create_mask(img_shape, masks, classes):
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    for polygon, cls in zip(masks, classes):
        value = 0
        if cls == 1:
            value = 86
        elif cls == 2:
            value = 128
        elif cls == 3:
            value = 43
        cv2.fillPoly(mask, [polygon], value)
    return mask

def main(image_path, label_path, output_path):
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Unable to read image at {image_path}")

    masks, classes = read_yolo_label(label_path, image.shape)
    mask = create_mask(image.shape, masks, classes)
    cv2.imwrite(output_path, mask)

    print(f"Processed image saved to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Path to the input image")
    parser.add_argument("--label", help="Path to the YOLO segmentation label file")
    parser.add_argument("--output", help="Path to save the output image")
    args = parser.parse_args()

    main(args.input, args.label, args.output)