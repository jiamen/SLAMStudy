
import cv2
import numpy as np
import matplotlib.pyplot as plt


img = cv2.imread('./lena.jpg')
lena_img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

# 灰度化处理图像
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Roberts算子
kernel_x = np.array([[-1,0], [0,1]], dtype=int)
kernel_y = np.array([[0,-1], [1,0]], dtype=int)

x = cv2.filter2D(grayImage, cv2.CV_16S, kernel_x)
y = cv2.filter2D(grayImage, cv2.CV_16S, kernel_y)

# 转uint8
absX = cv2.convertScaleAbs(x)
absY = cv2.convertScaleAbs(y)
Roberts = cv2.addWeighted(absX, 0.5, absY, 0.5, 0)

# 用来正常显示中文标签
plt.rcParams['font.sans-serif']=['SimHei']

# 显示图形
titles = ['initial image', 'Roberts operator']
images = [lena_img, Roberts]

for i in range(2):
    plt.subplot(1, 2, i+1), plt.imshow(images[i], 'gray')
    plt.title(titles[i])
    plt.xticks([]), plt.yticks([])
plt.show()



