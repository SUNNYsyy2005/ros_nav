class AMCLLaserData:
    def __init__(self):
        self.range_count = 0 #测量点
        self.range_max = 0.0 #最大测量距离
        self.ranges = [] #距离和角度
        self.angle_min = 0.0 #最小角度
        self.angle_max = 6.28 #最大角度
        self.angle_increment = 0.0
    def update_from_scan(self, scan):
        # 更新最大测量距离
        self.range_max = scan.range_max
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        angle_max = scan.angle_max
        range_min = scan.range_min
        # 更新距离列表，小于range_min的值改为0，大于range_max的值改为range_max
        self.ranges = [0 if r < range_min else scan.range_max if r > scan.range_max else r for r in scan.ranges]
        for i in range(len(self.ranges)):
            self.ranges[i] /= 0.05
        # 更新测量点数量
        self.range_count = len(scan.ranges)