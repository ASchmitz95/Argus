import cv2
from img_processing import detect_color, detect_objects
from img_draw import draw_obj



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
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cam.set(cv2.CAP_PROP_FPS, 30)

    print(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    print(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(cam.get(cv2.CAP_PROP_FPS))


    while True:
        ret, frame = cam.read()

        if not ret:
            break

        
        detected = detect_color(frame, target_color, color_tolerance)
        objects = detect_objects(detected, obj_size, gap_size)

        frame = draw_obj(frame, objects)



        cv2.imshow("Kamera-Stream", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_color_detection()