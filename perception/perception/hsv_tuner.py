import cv2
import numpy as np


def nothing(x):
    pass


cap = cv2.VideoCapture(4)  # RealSense RGB - probaj 0,2,4 če ne dela

cv2.namedWindow("HSV Tuner")
cv2.createTrackbar("H min", "HSV Tuner", 0, 255, nothing)
cv2.createTrackbar("H max", "HSV Tuner", 255, 255, nothing)
cv2.createTrackbar("S min", "HSV Tuner", 0, 255, nothing)
cv2.createTrackbar("S max", "HSV Tuner", 255, 255, nothing)
cv2.createTrackbar("V min", "HSV Tuner", 0, 255, nothing)
cv2.createTrackbar("V max", "HSV Tuner", 200, 255, nothing)

while True:
    ret, img = cap.read()
    if not ret:
        print("Napaka pri branju kamere!")
        break

    h_min = cv2.getTrackbarPos("H min", "HSV Tuner")
    h_max = cv2.getTrackbarPos("H max", "HSV Tuner")
    s_min = cv2.getTrackbarPos("S min", "HSV Tuner")
    s_max = cv2.getTrackbarPos("S max", "HSV Tuner")
    v_min = cv2.getTrackbarPos("V min", "HSV Tuner")
    v_max = cv2.getTrackbarPos("V max", "HSV Tuner")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv, np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])
    )

    result = cv2.bitwise_and(img, img, mask=mask)

    tekst = f"H:[{h_min}-{h_max}] S:[{s_min}-{s_max}] V:[{v_min}-{v_max}]"
    cv2.putText(
        result, tekst, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
    )

    cv2.imshow("HSV Tuner", result)
    cv2.imshow("Original", img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        print(f"lower = [{h_min}, {s_min}, {v_min}]")
        print(f"upper = [{h_max}, {s_max}, {v_max}]")
        break

cap.release()
cv2.destroyAllWindows()
