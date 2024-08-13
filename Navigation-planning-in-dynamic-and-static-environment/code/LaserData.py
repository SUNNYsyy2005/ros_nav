class AMCLLaserData:
    def __init__(self):
        self.range_count = 0 #测量点
        self.range_max = 0.0 #最大测量距离
        self.ranges = [] #距离和角度