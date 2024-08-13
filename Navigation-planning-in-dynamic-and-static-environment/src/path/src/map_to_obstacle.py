from PIL import Image

# 打开PGM文件
image_path = '/code/1.pgm'
img = Image.open(image_path)

# 将图像转换为灰度模式
img = img.convert('L')

# 初始化障碍物坐标数组
obstacle_x = []
obstacle_y = []

# 遍历图像中的每个像素

width, height = img.size
print(width, height)
for y in range(height):
    for x in range(width):
        # 获取当前像素的灰度值
        pixel_value = img.getpixel((x, y))
        # 假设灰度值小于某个阈值（例如，100）的像素表示障碍物
        if pixel_value < 30:
            obstacle_x.append(x)
            obstacle_y.append(y)
path = [[x, y] for x, y in zip(obstacle_x, obstacle_y)]
print(len(path))