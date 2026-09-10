import cv2
import time
from img_processing import detect_color, detect_objects, get_distance
from img_draw import draw_obj

CAM_WIDTH = 1920
CAM_HEIGHT = 1080
CAM_FPS = 30
TOLERANCE_SIZE = 150



def test_color_detection():
    """Run a live camera preview to manually test color and object detection.
    Stop when 'q' is pressed or a frame cannot be read, then release
    the camera and close all OpenCV windows.
    """
    
    cam = cv2.VideoCapture(0, cv2.CAP_MSMF)
    color_tolerance = 0.085
    target_color = [255,0,0]
    obj_size = 1000
    gap_size = 25

    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cam.set(cv2.CAP_PROP_FPS, CAM_FPS)

    print(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    print(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(cam.get(cv2.CAP_PROP_FPS))


    while True:
        ret, frame = cam.read()

        if not ret:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        detected = detect_color(frame, target_color, color_tolerance)
        objects = detect_objects(detected, obj_size, gap_size)
        target_cam_movement = get_distance(objects, CAM_WIDTH, CAM_HEIGHT, TOLERANCE_SIZE)

        if target_cam_movement:
            print(target_cam_movement)

        frame = draw_obj(frame, objects)
        frame = draw_obj(frame, [[CAM_WIDTH//2, CAM_HEIGHT//2, TOLERANCE_SIZE*2, TOLERANCE_SIZE*2]], (0, 0, 255))
        

        cv2.imshow("Kamera-Stream", frame)

    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_color_detection()