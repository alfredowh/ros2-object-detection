import os
import yaml
import numpy as np
import cv2
from typing import Tuple, List


def generate_random_lines(
    imshape: Tuple[int, int, int], slant: np.ndarray, drop_length: int
) -> List[Tuple[int, int]]:
    """Generate lines visualizing rain drops."""
    drops = []
    for i in range(500):  # Rain intensity
        if slant < 0:
            x = np.random.randint(0, imshape[1] + slant)
        else:
            x = np.random.randint(0, imshape[1] - slant)
        y = np.random.randint(0, imshape[0] - drop_length)
        drops.append((x, y))
    return drops


def add_rain(image: np.ndarray, brightness_reduction: int = 50) -> np.ndarray:
    """Add raining and blurry effect to image."""
    imshape = image.shape
    slant_extreme = 20
    slant = np.random.randint(-slant_extreme, slant_extreme)
    drop_length = 20
    drop_width = 2
    drop_color = (200, 200, 200)  # Light gray for rain drops
    rain_drops = generate_random_lines(imshape, slant, drop_length)

    rain_image = image.copy()

    # Rain drops
    for rain_drop in rain_drops:
        cv2.line(
            rain_image,
            (rain_drop[0], rain_drop[1]),
            (rain_drop[0] + slant, rain_drop[1] + drop_length),
            drop_color,
            drop_width,
        )

    rain_image = cv2.blur(rain_image, (8, 8))  # Blury effect

    # Reduce brightness
    brightness_reduction = brightness_reduction
    rain_image = cv2.subtract(
        rain_image, np.full_like(rain_image, brightness_reduction)
    )

    return rain_image


def add_fog(image: str, fog_intensity: float = 0.5):

    # Create a white layer
    fog_layer = np.full_like(image, 150, dtype=np.uint8)
    foggy_image = cv2.addWeighted(image, 1 - fog_intensity, fog_layer, fog_intensity, 0)

    return foggy_image


def convert_labels(classes_txt: str, data_yaml: str, label_path: str) -> None:
    """Adjust label classes from roboflow labeling to KITTI classes."""
    with open(data_yaml, "r") as file:
        data = yaml.safe_load(file)

    classes_roboflow = data.get("names", [])

    with open(classes_txt) as f:
        classes_kitti = f.readlines()
        ## remove whitespace characters like `\n` at the end of each line
        classes_kitti = [x.strip() for x in classes_kitti]

    # Get all files in the directory
    files = [f for f in os.listdir(label_path) if f.endswith(".txt")]

    # Rename files
    for file in files:

        path = os.path.join(label_path, file)

        with open(path, "r") as f:
            labels = f.readlines()

        new_labels = []
        for label in labels:
            label = label.strip()
            class_id = int(label[0])
            class_name = classes_roboflow[class_id]
            new_class_id = classes_kitti.index(class_name)

            new_label = str(new_class_id) + label[1:]
            new_labels.append(new_label)

        with open(path, "w") as f:
            for new_label in new_labels:
                f.write(new_label + "\n")

        new_name = file.split("_")[0]
        if not new_name.endswith(".txt"):
            new_name = new_name + ".txt"

        new_path = os.path.join(label_path, new_name)

        os.rename(path, new_path)
        print(f"Renamed: {file} -> {new_name}")
