import cv2
from ultralytics import YOLO
from pathlib import Path
import time
import argparse

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
    parser.add_argument("--model_path",type=str,default= Path.home() / "runs" / "obb" / "train5" / "weights" / "best.pt")
    parser.add_argument("--use_example_img",type=bool,default=False,help="set True to test loadd model on a example image")
    parser.add_argument("--img_path",type=str,default="/mnt/nova_ssd/dataset_boiling_pot/IMG_20250501_193208.jpg",help="full path to example image (only if use_example_img is set True)")
    parser.add_argument("--confidence",type=float,default=0.5)
    parser.add_argument("--grid",type=str,default="2x2")

    args = parser.parse_args()
    model_path = args.model_path
    model = YOLO(str(model_path))
    example_img = args.img_path
    USE_EXAMPLE_IMG = args.use_example_img
    confidence = args.confidence
    grid_format = args.grid
    print(f"using {grid_format} grid")
    nrow,ncol = [int(x) for x in grid_format.split("x")]
    print(f"row: {nrow} | col: {ncol}")

    if not USE_EXAMPLE_IMG: 
        cap = cv2.VideoCapture(0)
        cam_idx = 0
        while not cap.isOpened():
            cam_idx +=1
            cap = cv2.VideoCapture(cam_idx)
        print(f"accessing camera at idx {cam_idx}")

    cv2.namedWindow("YOLOv8 Inference", cv2.WINDOW_NORMAL)

    while True:
        if USE_EXAMPLE_IMG:
            frame = cv2.imread(example_img)
            if frame is None:
                print(f"Error: Could not load image {example_img}")
                break
            success = True
        else:
            success, frame = cap.read()
            time.sleep(0.1)
        if success:
            results = model(frame, conf = confidence)
            annotated_frame = results[0].plot()
            u,v,_ = annotated_frame.shape # height,width,3
            line_thickness = int(u/110)
            # horizontal lines:
            for row in range(1,nrow):
                cv2.line(annotated_frame,(0,int(row*u/nrow)),(v,int(row*u/nrow)),color=(0,0,0),thickness=line_thickness)
            # vertical lines
            for col in range(1,ncol):
                cv2.line(annotated_frame,(int(col*v/ncol),0),(int(col*v/ncol),u),color=(0,0,0),thickness=line_thickness)

            herdplatte_idx = 0
            for row in range(0,nrow):
                for col in range(0,ncol):
                    herdplatte_idx +=1
                    cv2.putText(annotated_frame,f"Herdplatte {herdplatte_idx}",(int(col*v/ncol)+20,int(row*u/nrow)+20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)

            for det_nr, result in enumerate(results):
                xywhr = result.obb.xywhr.cpu().numpy()  # center-x, center-y, width, height, angle (radians)
                xyxyxyxy = result.obb.xyxyxyxy.cpu().numpy()  # polygon format with 4-points
                names = [result.names[cls.item()] for cls in result.obb.cls.int()]  # class name of each box
                confs = result.obb.conf  # confidence score of each box
                for detection in names:
                    x_center = int(xywhr[det_nr][0])
                    y_center = int(xywhr[det_nr][1])
                    row,col = get_sector(x_center,y_center,(u,v),grid_format)
                    print(f"{detection} pot found at Herdpaltte {row}|{col}")
                    cv2.drawMarker(annotated_frame,(int(xywhr[det_nr][0]),int(xywhr[det_nr][1])),color=(0,0,255),markerSize=100,thickness=20)
            display_img(annotated_frame)
        if USE_EXAMPLE_IMG:
            time.sleep(5)
            break

    if not USE_EXAMPLE_IMG:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()