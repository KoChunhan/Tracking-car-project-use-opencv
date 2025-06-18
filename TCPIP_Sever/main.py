import socket
import struct
from collections import deque
import time
import cv2
import numpy as np


# 接收固定長度資料的函數
def recv_all(sock, length):
    data = b''
    while len(data) < length:
        packet = sock.recv(length - len(data))
        if not packet:
            return None  # 連線中斷
        data += packet
    return data
#舊版影像處理代碼
# def process_frame(frame):
#     h, w = frame.shape[:2]
#     roi = frame[ROI_TOP:ROI_TOP+ROI_HEIGHT, :]
#
#     if ROI_TOP + ROI_HEIGHT > h:
#         print("ROI超出影像範圍")
#         return frame, 0, frame
#
#     gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
#     blur = cv2.GaussianBlur(gray, (5, 5), 0)
#     #_, binary = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)
#     #自適應二值化
#     binary = cv2.adaptiveThreshold(blur, 255,
#                                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
#                                    cv2.THRESH_BINARY_INV, 11, 3)
#     edges = cv2.Canny(binary, 30, 100)          #50,150 ->
#
#     lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=20, maxLineGap=30)
#     offset = 0
#     cx = w // 2        # ROI 中央
#     if lines is not None:
#         points = []
#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
#             points.append((x1 + x2) // 2)
#         if points:
#             lane_center = sum(points) // len(points)
#             offset = lane_center - cx
#             cv2.circle(roi, (lane_center, ROI_HEIGHT // 2), 5, (255, 0, 0), -1)
#     # 畫中心線與 ROI 方框
#     cv2.line(frame, (cx, ROI_TOP), (cx, ROI_TOP+ROI_HEIGHT), (0, 0, 255), 1)
#     cv2.rectangle(frame, (0, ROI_TOP), (w, ROI_TOP+ROI_HEIGHT), (255, 255, 0), 1)
#     cv2.putText(frame, f"Offset: {offset}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
#
#     return frame, offset, roi
# ROI 層級區塊定義（top, height）

# 参数配置 - 经过严格测试的稳定值
ROI_TOP = 30  # 感興区域顶部位置 (根据摄像头高度调整)
ROI_HEIGHT = 50  # 感興区域高度
ROI_WIDTH = 320
ROI_LEFT = 0
THRESHOLD = 20



# 主要影像處理與偏移計算邏輯
def process_frame(frame):
    h, w = frame.shape[:2]
    cx = w // 2

    # 擷取 ROI 區域
    roi = frame[ROI_TOP:ROI_TOP + ROI_HEIGHT, ROI_LEFT:ROI_LEFT + ROI_WIDTH]
    if ROI_TOP + ROI_HEIGHT > h:
        print("ROI超出影像範圍")
        return frame, 0, cx

    # 圖像預處理
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)
    edges = cv2.Canny(binary, 50, 150)

    # 霍夫線檢測
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, minLineLength=20, maxLineGap=20)
    offset = 0

    if lines is not None:
        points = []
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
            mid_x = (x1 + x2) // 2
            points.append(mid_x)
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            angles.append(angle)

        if len(points) >= 2:
            lane_center = sum(points) // len(points)
            offset = lane_center - (ROI_WIDTH // 2)

            # 畫出車道中心點
            cv2.circle(roi, (lane_center, ROI_HEIGHT // 2), 5, (255, 0, 0), -1)

            # 判斷是否是彎道
            angle_variation = max(angles) - min(angles)
            if angle_variation > 40:  # 可調整
                cv2.putText(roi, f"Turn Detected", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    else:
        offset = 0
        cv2.putText(roi, "Line Not Found", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return frame, offset, cx

# 建立 TCP 伺服器等待 ESP32CAM 連線
server = socket.socket()
server.bind(('0.0.0.0', 5000))
server.listen(1)
print("等待 ESP32CAM 連線...")
client, _ = server.accept()
print("連線成功")

# 主迴圈接收影像與處理
while True:
    header = recv_all(client, 4)
    if not header:
        print("Header接收失敗")
        break

    img_len = struct.unpack('<I', header)[0]
    print(f"影像長度: {img_len}")

    img_data = recv_all(client, img_len)
    if not img_data:
        print("影像資料接收失敗")
        break

    np_arr = np.frombuffer(img_data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img is None:
        print("影像解碼失敗")
        continue

    processed, offset, cx = process_frame(img)
    print("偏移量:", offset)

    try:
        print(f"傳給 client 的偏移值: {offset}")
        client.send((str(offset) + '\n').encode())
        print(" 偏移值已送出")

        #print(f"已送出給 ESP32-CAM: {offset.strip()}")
    except Exception as e:
        print("回傳偏移失敗:", e)
        break

    cv2.line(processed, (cx, ROI_TOP), (cx, ROI_TOP + ROI_HEIGHT), (0, 0, 255), 1)
    cv2.rectangle(processed, (ROI_LEFT, ROI_TOP), (ROI_LEFT + ROI_WIDTH, ROI_TOP + ROI_HEIGHT), (255, 255, 0), 1)
    cv2.putText(processed, f"Offset: {offset}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("Processed", processed)

    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
client.close()
server.close()