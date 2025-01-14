import cv2
import numpy as np


def reshape_image_find(image):
    '''归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主'''
    width, height = image.shape[1], image.shape[0]
    min_len = width
    scale = width * 1.0 / 1000
    new_width = 1000

    new_height = int(height / scale)
    if new_height > 1000:
        new_height = 1000
        scale = height * 1.0 / 1000
        new_width = int(width / scale)
    out = cv2.resize(image, (new_width, new_height))
    return out, new_width, new_height

smarties = cv2.imread("mapwithpoint.jpg")
smarties = reshape_image_find(smarties)[0]
gray_img= cv2.cvtColor(smarties,cv2.COLOR_BGR2GRAY)
cv2.imshow("gray",gray_img)
#进行中值滤波
img = cv2.medianBlur(gray_img,5)
cv2.imshow("median",img)
#进行霍夫圆变换

circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,circles=None,param1=50,param2=26,minRadius=5,maxRadius=35)
print(circles)
#对数据进行四舍五入变为整数
circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    #画出来圆的边界
    cv2.circle(smarties,(i[0],i[1]),i[2],(0,255,0),2)
    #画出来圆心
    cv2.circle(smarties,(i[0],i[1]),2,(0,255,255),3)
    print(i)
cv2.imshow("Circle",smarties)
cv2.waitKey()
cv2.destroyAllWindows()

