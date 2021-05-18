
import cv2


print('cv2.__version__: ', cv2.__version__)

# 读取图像
image = cv2.imread('./res2.jpg', cv2.IMREAD_GRAYSCALE)

h, w = image.shape
print("h = ", h, ", w = ", w)

# Sobel 滤波器 进行边的检测
sobel_horizontal = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=5)   # 水平方向
sobel_vertical   = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=5)      # 垂直方向

cv2.imshow('sobel_H', sobel_horizontal)     # 水平方向
cv2.imshow('sobel_V', sobel_vertical)       # 垂直方向


# 拉普拉斯算子 进行边的检测，  64F代表每一个像素点元素占64位浮点数
laplacian = cv2.Laplacian(image, cv2.CV_64F, ksize=5)
cv2.imshow('laplacian', laplacian)


# Canny 边检测器
canny = cv2.Canny(image, 50, 240)
cv2.imshow('Canny', canny)


# print(image.shape)        # (379, 600)
# print(image.dtype)        # 格式为uint8
# 进行图片的裁剪
image1 = image[:, :200]     # 随便进行裁剪

# cv2.namedWindow("image")
cv2.imshow('image', image)
# cv2.imshow('cut_image', image1)
cv2.waitKey(0)

# 释放窗口
cv2.destroyAllWindows()
