import cv2
import numpy as np
import serial
import math
import threading
import queue



class Point:
    def __init__(self, angle, Distance):
        self.angle = angle
        self.Distance = Distance


class LidarDriver:
    def __init__(self, 
                 port, 
                 image_size=(640, 640), # 图像大小(H,W)
                 baudrate=150000,
                 scale=0.1,
                 queue_size=1000,
                 ):
        self.port = port
        self.baudrate = baudrate
        self.scale = scale
        self.image_size = image_size
        self.queue = queue.Queue(queue_size)
        self.queue_size = queue_size
        self.running = False
        self.CPoint = []
        self.LPoint = []

        # 打开串口
        self.OpenSerial()
        # 启动线程
        self.thread = threading.Thread(target=self.ReadData, daemon=True)

    def Is2(self,data):
        return len(data) == 2 

    def IsHeader(self,data):
        return self.Is2(data) and data[0] == 0xAA and data[1] == 0x55

    def IsBegin(self,data):
        return self.Is2(data) and data[0] & 0x01 == 1

    def DecodeAngle(self,data):
        LSB = data[0]
        MSB = data[1]
        return (((MSB << 8) | LSB) >> 1) / 64

    def GetDataLength(self,data):
        return data[1]

    def DecodeDistance(self,data):
        LSB = data[0]
        MSB = data[1]
        return (MSB << 8 | LSB) / 4

    def DecodeMediumAngle(self,FSA, LSA, DataLength, PointDis):
        diff = (LSA - FSA) % 360.0
        return diff / (DataLength - 1) * PointDis + FSA
    def projection(self,P):
        angle = P.angle
        Distance = P.Distance
        rad = math.radians((angle-90) % 360)
        x_orig = -math.sin(rad) * Distance
        y_orig = math.cos(rad) * Distance
        x = x_orig
        y = y_orig
    
        return [x, y]
    

    def OpenSerial(self):
        try: 
            self.ser = serial.Serial(
                port=self.port, 
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.running = True
            print("串口打开成功,开始运行.......")
        except:
            print("串口打开失败，程序关闭。")
            exit()
    def StartThread(self):
        self.thread.start()
    def DestroyDriver(self):
        self.running = False
        self.ser.close()
        self.thread.join()

    
    def ReadData(self):
        while self.running:
            try:
                data = self.ser.read(2)
                while len(data) == 2 and not self.IsHeader(data):
                    data = data[1:] + self.ser.read(1)
                if not self.IsHeader(data):
                    continue
                
                data = self.ser.read(2)
                # ----------------
                # 2获取数据长度
                # ----------------
                DataLength = self.GetDataLength(data)

                data = self.ser.read(2)
                # -------------------------
                # 3获取起始角度
                # -------------------------
                FSA = self.DecodeAngle(data)

                data = self.ser.read(2)
                # -------------------------
                # 4获取结束角度
                # -------------------------
                LSA = self.DecodeAngle(data)

                # -------------------------
                # 5获取数据点距离
                # -------------------------
                scan_buffer = []
                for i in range(DataLength):
                    data = self.ser.read(3)
                    if i == 0:
                        scan_buffer.append(Point(FSA, self.DecodeDistance(data)))
                    elif i == DataLength - 1:
                        scan_buffer.append(Point(LSA, self.DecodeDistance(data)))
                    else:
                        scan_buffer.append(Point(
                        self.DecodeMediumAngle(FSA, LSA, DataLength, i),
                        self.DecodeDistance(data)
                        ))
                if self.queue.full():
                    self.queue.get()
                self.queue.put(scan_buffer)             
            except:
                print("数据读取失败")
                exit(1)
    def GetData(self):
        while not self.IsEmptyQueue():
            scan_buffer = self.queue.get()
            self.CPoint += [self.projection(scan_buffer[i]) for i in range(len(scan_buffer))]
        if len(self.CPoint) >400 :
            for i in self.CPoint:
                cx = self.image_size[1] /2
                cy = self.image_size[0] /2
                i[0] = int(i[0] * self.scale + cx)
                i[1] = int(i[1] * self.scale + cy)
            self.LPoint =self.CPoint.copy()
            temp = self.CPoint.copy()
            self.CPoint =[]
            return temp
        else:
            if len( self.LPoint) == 0:
                 self.LPoint = self.CPoint.copy()
                 for i in self.LPoint:
                    cx = self.image_size[1] /2
                    cy = self.image_size[0] /2
                    i[0] = int(i[0] * self.scale + cx)
                    i[1] = int(i[1] * self.scale + cy)
            
            return self.LPoint

        
        
    def IsEmptyQueue(self):
        return self.queue.empty()
        
     
        


def main():
    lidar = LidarDriver(port="COM5",
                        baudrate=150000,
                        image_size=(640, 640),
                        queue_size=1000,
                        scale= 0.1)
    lidar.StartThread()
    Mat_H = np.zeros((640, 640, 3), np.uint8)
    while True:
        Point = lidar.GetData()
        for i in range(5):
            cv2.circle(Mat_H, (320, 320), int(640*(i+1)/5/2), (255, 255, 255), 1)
        if Point is not None:
            for x,y in Point:
                if 0 <= x < 640 and 0 <= y < 640:
                    cv2.circle(Mat_H, (x, y), 1, (0, 255, 0), -1)
        
        cv2.imshow("Lidar", Mat_H)
        Mat_H[:] = (0, 0, 0)
        if cv2.waitKey(1) == ord("q"): 
            break

    print("正在退出...")
    lidar.DestroyDriver()
    cv2.destroyAllWindows()

        
        
    




if __name__ == "__main__":
    main()