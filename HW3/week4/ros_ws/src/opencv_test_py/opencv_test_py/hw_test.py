import cv2 # OpenCV library
import numpy as np

from skimage.feature import peak_local_max

img = cv2.imread("oi.png")
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


# corners
img_corners = cv2.cornerHarris(img_gray,4,7,0.04)
cv2.imshow("corners_pure",img_corners)
coords = peak_local_max(img_corners,min_distance=3,threshold_abs=0.01)
#coords = np.argwhere(img_corners>0).astype(int)
print(coords.shape)
img_with_corners = img.copy()

for coord in coords:
    cv2.drawMarker(img_with_corners,[coord[1],coord[0]],color=[165,42,42])

cv2.imshow("corners",img_with_corners)

# canny
canny = cv2.Canny(img_gray, 200,300)
cv2.imshow("canny", canny)


# hough circles
circles = cv2.HoughCircles(img_gray, 
                        cv2.HOUGH_GRADIENT,1,20, 
                        param1=100,
                        param2=30,
                        minRadius=0,
                        maxRadius=0)

circles = np.uint16(np.around(circles))
img_circles = img.copy()
for i in circles[0,:]:
    print(f"center:{(i[0],i[1])}")
    cv2.circle(img_circles,(i[0],i[1]),i[2],(165,42,42),2)
    cv2.circle(img_circles,(i[0],i[1]),2,(165,42,42),3)

cv2.imshow("circles",img_circles)
cv2.waitKey(0)
