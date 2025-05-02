import cv2
import numpy as np
import cameraInterface
import os

folder_contents = os.listdir("/home/Sert/syncthing/spring 2025/slam py/")


# previous_video_number: int = -1

# for item in folder_contents:
#     item = item[0 : len(item) - 4]
#     if item[0:11] == "outputvideo":
#         for index, char in enumerate(item[::-1]):
#             if (
#                 not char.isnumeric()
#                 and int(item[len(item) - index :]) > previous_video_number
#             ):
#                 previous_video_number = int(item[len(item) - index :])


current_frame: bytes = b""


writer = cv2.VideoWriter(
    f"ramp1.mp4",
    cv2.VideoWriter.fourcc("m", "p", "4", "v"),
    30,
    (640, 480),
)

camera = cameraInterface.RealsenseCameraInterface()

current_frame: bytes = b""


try:
    while True:

        frame = camera.get_frame()

        if frame is None:
            continue

        # img_np = cv2.imdecode(
        #     color_image, cv2.IMREAD_COLOR
        # )  # cv2.IMREAD_COLOR in OpenCV 3.1
        img_np = cv2.resize(frame.color_image, (640, 480))
        writer.write(img_np)
        current_frame = b""
except Exception as e:
    print(e)
    writer.release()
