import dlib
import cv2
import argparse as ap
import imutils


def run(source=0, dispLoc=False):
    cam = cv2.VideoCapture(source)
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cnt = 0
    while True:
        cnt += 1
        retval, img = cam.read()
        
        img = imutils.resize(img, width=min(400, img.shape[1]))
        
        # (rects, weights) = hog.detectMultiScale(img, winStride=(4, 4), padding=(8, 8), scale=1.05)
        (rects, weights) = hog.detectMultiScale(img)
        
        print "rectangles detected: ", rects
        if len(rects) == 0:
            continue
        
        # rects = sorted(rects)
        (x, y, w, h) = rects[0]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.imwrite(str(cnt)+"-rect.png", img)
        break
        
    # points = get_points.run(img) 
    # points[0] = [26, 73, 94, 208]
    points = [[x, y, (x+w), (y+h)]]

    tracker = dlib.correlation_tracker()
    tracker.start_track(img, dlib.rectangle(*points[0]))

    cnt = 0
    prev_rect = None
    while True:
        retval, img = cam.read()
        if img == None:
            break
        
        cnt += 1
        if cnt % 50 == 0:
            continue

        img = imutils.resize(img, width=min(400, img.shape[1]))
        tracker.update(img)
        rect = tracker.get_position()
        top_left_pt = (int(rect.left()), int(rect.top()))
        bot_right_pt = (int(rect.right()), int(rect.bottom()))
        
        if prev_rect:
            prev_ar = (prev_rect.bottom() - prev_rect.top()) * (prev_rect.right() - prev_rect.left())
            ar = (rect.bottom() - rect.top()) * (rect.right() - rect.left())
            
            # print "hor-left diff: {0:.2f}".format(rect.left() - prev_rect.left())
            # print "hor-right diff: {0:.2f}".format(rect.right() - prev_rect.right())
            
            # print "ver-top diff: {0:.2f}".format(rect.top() - prev_rect.top())
            # print "ver-bottom diff: {0:.2f}".format(rect.bottom() - prev_rect.bottom())
            
            # print "linear hor-line diff: {0:.2f}".format((rect.right() - rect.left()) - (prev_rect.right() - prev_rect.left()))
            # print "linear ver-line diff: {0:.2f}".format((rect.bottom() - rect.top()) - (prev_rect.bottom() - prev_rect.top()))
            
            print "area diff: {0:.2f}".format(ar - prev_ar)

        prev_rect = rect

        cv2.rectangle(img, top_left_pt, bot_right_pt, (0, 0, 255), 2)
        print "Object tracked at [{}, {}] \r".format(top_left_pt, bot_right_pt)
        
        if dispLoc:
            loc = (int(rect.left()), int(rect.top()-20))
            txt = "Object tracked at [{}, {}]".format(top_left_pt, bot_right_pt)
            cv2.putText(img, txt, loc , cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0,255), 2)
        
        # cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        # cv2.imshow("Image", img)
        cv2.imwrite(str(cnt)+".png", img)

    cam.release()


if __name__ == "__main__":
    parser = ap.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-v', "--videoFile", help="Path to Video File")
    parser.add_argument('-l', "--dispLoc", dest="dispLoc", action="store_true")
    args = vars(parser.parse_args())

    run(args["videoFile"], args["dispLoc"])


### dependencies: pip install imutils dlib cv2
### run command: python tracker_dlib.py --videoFile ../data/sample_pedestrian.mp4 --dispLoc
