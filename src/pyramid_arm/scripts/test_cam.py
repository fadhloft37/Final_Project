# test_webcam.py   pakai: python3 test_webcam.py
import cv2
import vision

cap = cv2.VideoCapture("http://10.161.126.195:8080/video")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, vision.IMG_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vision.IMG_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    raise SystemExit("Kamera tidak bisa dibuka")

H = vision.buat_H()
warna = "orange"
daftar = {ord("1"): "orange", ord("2"): "black", ord("3"): "beige"}

while True:
    ok, frame = cap.read()
    if not ok:
        break

    h, w = frame.shape[:2]
    if (w, h) != (vision.IMG_W, vision.IMG_H):
        print("PERINGATAN: resolusi kamera %dx%d, beda dari IMG_W/IMG_H di vision.py" % (w, h))

    found, x, y, info = vision.detect(frame, warna, H, vision.HSV_HW)
    if found:
        tampil = vision.draw(frame, info, x, y)
        print("%s: x=%.3f y=%.3f  (lebar bbox=%d px)" % (warna, x, y, info["bbox"][2]))
    else:
        tampil = frame.copy()

    cv2.putText(tampil, "warna: " + warna, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("webcam", tampil)
    cv2.imshow("mask", info["mask"])

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    if k in daftar:
        warna = daftar[k]

cap.release()
cv2.destroyAllWindows()