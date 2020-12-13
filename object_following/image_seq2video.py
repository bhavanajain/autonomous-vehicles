"""
This code captures frames from web cam and writes to a video file.
"""

import cv2

capture = cv2.VideoCapture(0)

frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*"XVID")
video_writer = cv2.VideoWriter("output.avi", fourcc, 20, (frame_width, frame_height))

while True:
    has_frame, frame = capture.read()
    if not has_frame:
        print("Can't get frame")
        break

    video_writer.write(frame)

    cv2.imshow("Video frame", frame)
    key = cv2.waitKey(3)
    if key == 27:
        print("Pressed Esc")
        break

capture.release()
video_writer.release()
cv2.destroyAllWindows()
