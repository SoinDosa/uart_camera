import serial
import time
from datetime import datetime
from PIL import Image
import numpy as np
import struct

# UART 설정 (포트와 보드레이트를 환경에 맞게 수정)
ser = serial.Serial('COM8', 115200, timeout=1)

# 이미지 크기 설정 (프로젝트에 맞게 수정)
width = 96
height = 96
image_size = width * height * 2  # RGB565: 픽셀당 2바이트

def create_bmp_header(width, height, bits_per_pixel=24):
    """BMP 헤더 생성"""
    # 8비트 Grayscale의 경우 색상 팔레트가 필요 (256 * 4 = 1024바이트)
    if bits_per_pixel == 8:
        palette_size = 256 * 4  # 256색상 * 4바이트 (BGR + 예약)
        header_size = 54 + palette_size
        file_size = header_size + width * height * (bits_per_pixel // 8)
        pixel_offset = header_size
    else:
        header_size = 54
        file_size = header_size + width * height * (bits_per_pixel // 8)
        pixel_offset = header_size
    
    # BMP 파일 헤더 (14 bytes) - 더 간단한 방법
    header = bytearray()
    header.extend([0x42, 0x4D])  # 'BM' 시그니처
    
    # 파일 크기 (4바이트, 리틀 엔디안)
    header.extend([file_size & 0xFF, (file_size >> 8) & 0xFF, 
                   (file_size >> 16) & 0xFF, (file_size >> 24) & 0xFF])
    
    # 예약됨 (4바이트)
    header.extend([0x00, 0x00, 0x00, 0x00])
    
    # 픽셀 데이터 오프셋 (4바이트, 리틀 엔디안)
    header.extend([pixel_offset & 0xFF, (pixel_offset >> 8) & 0xFF,
                   (pixel_offset >> 16) & 0xFF, (pixel_offset >> 24) & 0xFF])
    
    # BMP 정보 헤더 (40 bytes) - 더 간단한 방법
    info_header = bytearray()
    
    # 정보 헤더 크기 (4바이트)
    info_header.extend([40, 0, 0, 0])
    
    # 너비 (4바이트)
    info_header.extend([width & 0xFF, (width >> 8) & 0xFF, 
                       (width >> 16) & 0xFF, (width >> 24) & 0xFF])
    
    # 높이 (4바이트)
    info_header.extend([height & 0xFF, (height >> 8) & 0xFF,
                       (height >> 16) & 0xFF, (height >> 24) & 0xFF])
    
    # 색상면 수 (2바이트)
    info_header.extend([1, 0])
    
    # 비트당 픽셀 (2바이트)
    info_header.extend([bits_per_pixel, 0])
    
    # 압축 방식 (4바이트)
    info_header.extend([0, 0, 0, 0])
    
    # 이미지 크기 (4바이트)
    image_data_size = width * height * (bits_per_pixel // 8)
    info_header.extend([image_data_size & 0xFF, (image_data_size >> 8) & 0xFF,
                       (image_data_size >> 16) & 0xFF, (image_data_size >> 24) & 0xFF])
    
    # 수평 해상도 (4바이트)
    info_header.extend([0, 0, 0, 0])
    
    # 수직 해상도 (4바이트)
    info_header.extend([0, 0, 0, 0])
    
    # 색상 테이블 크기 (4바이트)
    info_header.extend([0, 0, 0, 0])
    
    # 중요 색상 수 (4바이트)
    info_header.extend([0, 0, 0, 0])
    
    return header + info_header

def receive_image():
    """UART로부터 이미지 데이터 수신"""
    print("이미지 수신 대기 중...")
    
    # 시작 마커 대기
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"수신된 라인: {line}")
        if line == 'IMG_START':
            print("이미지 전송 시작 감지")
            break

    # 이미지 데이터 수신
    image_data = bytearray()
    bytes_received = 0
    
    while bytes_received < image_size:
        # 한 번에 1바이트씩 읽기 (더 안정적)
        byte_data = ser.read(1)
        if byte_data:
            image_data.extend(byte_data)
            bytes_received += 1
        else:
            print(f"데이터 수신 타임아웃. 수신된 바이트: {bytes_received}/{image_size}")
            break
    
    print(f"수신 완료: {bytes_received} 바이트")

    # 종료 마커 대기
    timeout_count = 0
    while timeout_count < 100:  # 최대 100번 시도
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"종료 마커 확인: {line}")
            if line == 'IMG_END':
                print("이미지 전송 종료 감지")
                break
        else:
            time.sleep(0.01)  # 10ms 대기
            timeout_count += 1

    return image_data

def rgb565_to_rgb888(data):
    """RGB565를 RGB888로 변환"""
    rgb888 = bytearray()
    for i in range(0, len(data), 2):
        if i + 1 < len(data):
            # STM32에서 전송한 순서대로 처리 (리틀 엔디안)
            pixel = data[i+1] | (data[i] << 8)  # high byte | low byte
            r = ((pixel >> 11) & 0x1F) << 3
            g = ((pixel >> 5) & 0x3F) << 2
            b = (pixel & 0x1F) << 3
            rgb888.extend([b, g, r])  # BMP는 BGR 순서
    return rgb888

def rgb565_to_grayscale(data):
    """RGB565를 Grayscale로 변환 (STM32 Get_Data 함수와 동일한 로직)"""
    grayscale = bytearray()
    for i in range(0, len(data), 2):
        if i + 1 < len(data):
            # STM32에서 전송한 순서대로 처리 (리틀 엔디안)
            pixel = data[i+1] | (data[i] << 8)  # high byte | low byte
            
            # RGB 채널 분리 (STM32 Get_Data 함수와 동일)
            r = ((pixel >> 11) & 0x1F) << 3  # 8-bit Red
            g = ((pixel >> 5) & 0x3F) << 2   # 8-bit Green  
            b = (pixel & 0x1F) << 3          # 8-bit Blue
            
            # STM32 Get_Data 함수와 동일한 Grayscale 변환 공식
            gray = (3 * r + 6 * g + b) // 10
            grayscale.append(gray)
    
    return grayscale

def save_bmp_rgb(data, filename):
    """RGB BMP 파일로 저장"""
    if len(data) != image_size:
        print(f"이미지 크기 오류: 예상 {image_size}, 실제 {len(data)}")
        return False
    
    # RGB565를 RGB888로 변환
    rgb_data = rgb565_to_rgb888(data)
    
    # BMP 헤더 생성
    bmp_header = create_bmp_header(width, height, 24)
    
    # BMP 파일은 아래에서 위로 저장되므로 행을 뒤집어야 함
    bmp_data = bytearray()
    for y in range(height - 1, -1, -1):  # 아래에서 위로
        row_start = y * width * 3
        row_end = row_start + width * 3
        bmp_data.extend(rgb_data[row_start:row_end])
        
        # 4바이트 정렬을 위한 패딩
        padding = (4 - (width * 3) % 4) % 4
        bmp_data.extend([0] * padding)
    
    # BMP 파일 저장
    with open(filename, 'wb') as f:
        f.write(bmp_header)
        f.write(bmp_data)
    
    print(f'RGB BMP 파일 저장 완료: {filename}')
    return True

def save_bmp_grayscale(data, filename):
    """Grayscale BMP 파일로 저장"""
    if len(data) != image_size:
        print(f"이미지 크기 오류: 예상 {image_size}, 실제 {len(data)}")
        return False
    
    # RGB565를 Grayscale로 변환
    gray_data = rgb565_to_grayscale(data)
    
    # BMP 헤더 생성 (8비트 Grayscale)
    bmp_header = create_bmp_header(width, height, 8)
    
    # 색상 팔레트 추가 (256색상 Grayscale)
    palette = bytearray()
    for i in range(256):
        palette.extend([i, i, i, 0])  # BGR + 예약바이트
    
    # BMP 파일은 아래에서 위로 저장되므로 행을 뒤집어야 함
    bmp_data = bytearray()
    for y in range(height - 1, -1, -1):  # 아래에서 위로
        row_start = y * width
        row_end = row_start + width
        bmp_data.extend(gray_data[row_start:row_end])
        
        # 4바이트 정렬을 위한 패딩
        padding = (4 - width % 4) % 4
        bmp_data.extend([0] * padding)
    
    # BMP 파일 저장
    with open(filename, 'wb') as f:
        f.write(bmp_header)
        f.write(palette)
        f.write(bmp_data)
    
    print(f'Grayscale BMP 파일 저장 완료: {filename}')
    return True

def save_numpy_array(data):
    """NumPy 배열로 저장 (모델 입력용)"""
    if len(data) != image_size:
        print(f"이미지 크기 오류: 예상 {image_size}, 실제 {len(data)}")
        return False
        
    gray_data = rgb565_to_grayscale(data)
    
    # 1차원 배열을 2차원으로 변환
    img_array = np.array(gray_data, dtype=np.uint8).reshape(height, width)
    
    # 정규화 (0-255 -> 0-1)
    img_normalized = img_array.astype(np.float32) / 255.0
    
    # 파일명 생성: numpy_년월일_시분초.npy
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'numpy_{timestamp}.npy'
    
    # NumPy 배열 저장
    np.save(filename, img_normalized)
    print(f'NumPy 배열 저장 완료: {filename}')
    return True

if __name__ == '__main__':
    print('BMP 이미지 수신 대기 중...')
    print('Grayscale BMP 파일로 저장됩니다.')
    print(f'예상 이미지 크기: {width}x{height} ({image_size} 바이트)')
    
    try:
        while True:
            img_data = receive_image()
            if img_data and len(img_data) == image_size:
                # 파일명 생성
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                
                # Grayscale BMP 파일로 저장
                bmp_filename = f'grayscale_{timestamp}.bmp'
                save_bmp_grayscale(img_data, bmp_filename)
                
                # RGB BMP 파일로도 저장 (선택사항)
                rgb_bmp_filename = f'rgb_{timestamp}.bmp'
                save_bmp_rgb(img_data, rgb_bmp_filename)
                
                # NumPy 배열로도 저장 (모델 입력용)
                save_numpy_array(img_data)
            else:
                print(f'유효하지 않은 이미지 데이터 수신: {len(img_data)} 바이트')
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        ser.close() 