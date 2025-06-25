import cv2
from ultralytics import YOLO
from pathlib import Path
import time
import argparse
import copy

def display_img(img):
    cv2.namedWindow("YOLOv8 Inference", cv2.WINDOW_NORMAL)
    # Resize the window
    cv2.resizeWindow("YOLOv8 Inference", 800, 600)
    cv2.imshow("YOLOv8 Inference",img)
    cv2.waitKey(1)

def get_sector(x,y,img_shape,grid):
    num_row,num_col = [int(x) for x in grid.split("x")]
    row_sec_size = img_shape[0] //num_row
    col_sec_size = img_shape[1] //num_col
    row = y //row_sec_size
    col = x//col_sec_size
    return row,col


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_path", type=str, default=Path.home() / "runs" / "obb" / "train5" / "weights" / "best.pt")
    parser.add_argument("--use_example_img", type=bool, default=False)
    parser.add_argument("--img_path", type=str, default="/home/lukashebio/dev_ws/src/Projektmodul_Elektronik/Kamera/zustandserkennung/test_data/test1.jpg")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--grid", type=str, default="2x2")
    args = parser.parse_args()

    model = YOLO(str(args.model_path))
    confidence = args.confidence
    grid_format = args.grid
    nrow, ncol = [int(x) for x in grid_format.split("x")]

    if not args.use_example_img:
        cap = cv2.VideoCapture(0)
        cam_idx = 0
        while not cap.isOpened():
            cam_idx += 1
            cap = cv2.VideoCapture(cam_idx)
        print(f"accessing camera at idx {cam_idx}")

    cv2.namedWindow("YOLOv8 Inference", cv2.WINDOW_NORMAL)

    Zustaende = {}
    unstable_zustaende = {}
    DETECTION_THRESHOLD = 3

    for row in range(nrow):
        for col in range(ncol):
            key = f"Platte({row}|{col})"
            Zustaende[key] = ["leer", time.monotonic()]
            unstable_zustaende[key] = {"state": "leer", "count": 0}

    while True:
        if args.use_example_img:
            frame = cv2.imread(args.img_path)
            if frame is None:
                print(f"Error: Could not load image {args.img_path}")
                break
            success = True
        else:
            success, frame = cap.read()
            time.sleep(0.1)

        if not success:
            continue

        results = model(frame, conf=confidence)
        annotated_frame = results[0].plot()
        u, v, _ = annotated_frame.shape
        line_thickness = int(u / 110)

        for row in range(1, nrow):
            cv2.line(annotated_frame, (0, int(row * u / nrow)), (v, int(row * u / nrow)), color=(0, 0, 0), thickness=line_thickness)
        for col in range(1, ncol):
            cv2.line(annotated_frame, (int(col * v / ncol), 0), (int(col * v / ncol), u), color=(0, 0, 0), thickness=line_thickness)

        herdplatte_idx = 0
        for row in range(nrow):
            for col in range(ncol):
                herdplatte_idx += 1
                key = f"Platte({row}|{col})"
                zustand, zeit = Zustaende[key]
                elapsed = time.monotonic() - zeit

                # Dynamische Maße je nach Bildgröße und Grid
                sector_height = u / nrow
                sector_width = v / ncol
                font_scale = min(sector_width, sector_height) / 400  # passt Größe an Grid & Bild an
                thickness = max(1, int(min(u, v) / 600))  # sicherstellen, dass immer sichtbar
                y_offset = int(sector_height * 0.1)  # ca. oberes Drittel im Sektor

                x_pos = int(col * sector_width + 10)  # kleine linke Einrückung
                y_pos = int(row * sector_height + y_offset)

                text = f"Platte{herdplatte_idx}: {zustand} seit {elapsed:.1f}s"
                cv2.putText(annotated_frame, text, (x_pos, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)

        if results[0].obb is not None:
            boxes = results[0].obb
            xywhr = boxes.xywhr.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
            class_names = results[0].names

            # temporäres Mapping Platte -> erkannter Zustand in diesem Frame
            erkannte_zustaende = {}

            for i in range(len(xywhr)):
                x_center = int(xywhr[i][0])
                y_center = int(xywhr[i][1])
                class_id = classes[i]
                class_name = class_names[class_id]
                row, col = get_sector(x_center, y_center, (u, v), grid_format)
                platte_key = f"Platte({row}|{col})"
                erkannte_zustaende[platte_key] = class_name

                #cv2.drawMarker(annotated_frame, (x_center, y_center), color=(0, 0, 255), markerSize=100, thickness=20)

            for key in Zustaende:
                erkannter_zustand = erkannte_zustaende.get(key, "leer")
                aktueller_zustand = Zustaende[key][0]

                if erkannter_zustand == aktueller_zustand:
                    unstable_zustaende[key] = {"state": erkannter_zustand, "count": 0}
                else:
                    if unstable_zustaende[key]["state"] == erkannter_zustand:
                        unstable_zustaende[key]["count"] += 1
                    else:
                        unstable_zustaende[key] = {"state": erkannter_zustand, "count": 1}

                    if unstable_zustaende[key]["count"] >= DETECTION_THRESHOLD:
                        print(f"{key} Zustand geändert von {aktueller_zustand} zu {erkannter_zustand}")
                        Zustaende[key] = [erkannter_zustand, time.monotonic()]
                        unstable_zustaende[key]["count"] = 0

        display_img(annotated_frame)

        if args.use_example_img:
            input("Ende?")
            break

    if not args.use_example_img:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()