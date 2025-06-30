import serial
import time
from datetime import datetime
from PIL import Image

# UART 설정 (포트와 보드레이트를 환경에 맞게 수정)
ser = serial.Serial('COM8', 115200, timeout=1)

# 이미지 크기 설정 (프로젝트에 맞게 수정)
width = 160
height = 120
image_size = width * height * 2  # RGB565: 픽셀당 2바이트

def receive_image():
    """UART로부터 이미지 데이터 수신"""
    # 시작 마커 대기
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line == 'IMG_START':
            break

    # 이미지 데이터 수신
    image_data = bytearray()
    while len(image_data) < image_size:
        remaining = image_size - len(image_data)
        chunk = ser.read(1024 if remaining > 1024 else remaining)
        if not chunk:
            break
        image_data.extend(chunk)

    # 종료 마커 대기
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line == 'IMG_END':
            break

    return image_data

def rgb565_to_rgb888(data):
    """RGB565를 RGB888로 변환"""
    rgb888 = bytearray()
    for i in range(0, len(data), 2):
        pixel = (data[i] << 8) | data[i+1]
        r = ((pixel >> 11) & 0x1F) << 3
        g = ((pixel >> 5) & 0x3F) << 2
        b = (pixel & 0x1F) << 3
        rgb888.extend([r, g, b])
    return rgb888

def rgb565_to_grayscale(data):
    """RGB565를 Grayscale로 변환"""
    grayscale = bytearray()
    for i in range(0, len(data), 2):
        pixel = (data[i] << 8) | data[i+1]
        
        # RGB 채널 분리
        r = (pixel >> 11) & 0x1F  # 5-bit Red (0-31)
        g = (pixel >> 5) & 0x3F   # 6-bit Green (0-63)
        b = pixel & 0x1F           # 5-bit Blue (0-31)
        
        # RGB 채널 8비트로 확장
        r8 = (r * 255) // 31
        g8 = (g * 255) // 63
        b8 = (b * 255) // 31
        
        # Grayscale 변환 (ITU-R BT.709 표준)
        gray = (r8 * 2126 + g8 * 7152 + b8 * 722) // 10000
        grayscale.append(gray)
    
    return grayscale


def save_image(data, grayscale=False):
    """이미지 저장 함수"""
    if grayscale:
        gray_data = rgb565_to_grayscale(data)
        img = Image.frombytes('L', (width, height), bytes(gray_data))
    else:
        rgb_data = rgb565_to_rgb888(data)
        img = Image.frombytes('RGB', (width, height), bytes(rgb_data))
    
    # 파일명 생성: traindata_년월일_시분초.jpg
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'traindata_{timestamp}.jpg'
    
    # 이미지 저장
    img.save(filename, 'JPEG')
    print(f'이미지 저장 완료: {filename}')

def save_grayscale_image(data):
    """Grayscale 이미지 저장 함수"""
    gray_data = rgb565_to_grayscale(data)
    img = Image.frombytes('L', (width, height), bytes(gray_data))
    
    # 파일명 생성: grayscale_년월일_시분초.jpg
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'grayscale_{timestamp}.jpg'
    
    # 이미지 저장
    img.save(filename, 'JPEG')
    print(f'Grayscale 이미지 저장 완료: {filename}')

if __name__ == '__main__':
    print('이미지 수신 대기 중...')
    print('Grayscale 이미지로 저장됩니다.')
    try:
        while True:
            img_data = receive_image()
            if img_data and len(img_data) == image_size:
                # Grayscale 이미지로 저장
                save_grayscale_image(img_data)
                # 또는 RGB 이미지로 저장하려면: save_image(img_data, grayscale=False)
            else:
                print('유효하지 않은 이미지 데이터 수신')
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        ser.close()
