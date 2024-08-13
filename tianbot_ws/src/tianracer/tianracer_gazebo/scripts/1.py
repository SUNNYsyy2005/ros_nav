from PIL import Image

# 打开地图文件
img = Image.open('/home/sunny/tianbot_ws/src/tianracer/tianracer_gazebo/maps/my_map.pgm')

# 旋转90度，并扩展图像尺寸
img_rotated = img.rotate(90, expand=True)

# 保存旋转后的地图
img_rotated.save('/home/sunny/tianbot_ws/src/tianracer/tianracer_gazebo/maps/my_map_rotated.pgm')