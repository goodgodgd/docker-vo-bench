import cv2

image = cv2.imread("sample.png", cv2.IMREAD_ANYDEPTH)
print(image.dtype)
cv2.imshow("img", image)
cv2.waitKey(0)
