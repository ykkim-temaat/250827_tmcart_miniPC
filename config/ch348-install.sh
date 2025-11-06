#!/bin/bash

# --- 설정 ---
CURRENT_KERNEL=$(uname -r)
SOURCE_KO="/home/temaat/tmcart/config/LINUX/driver/ch9344.ko"
TARGET_DIR="/lib/modules/${CURRENT_KERNEL}/kernel/drivers/usb/serial"
TARGET_KO_PATH="${TARGET_DIR}/ch9344.ko"
MODULE_NAME="ch9344"

# --- 1. 현재 커널에 드라이버가 설치되어 있는지 확인 ---
if [ ! -f "${TARGET_KO_PATH}" ]; then
    
    echo "ch9344.ko not found for kernel ${CURRENT_KERNEL}. Installing..."
    
    # 2. 설치 폴더 생성
    mkdir -p "${TARGET_DIR}"
    
    # 3. 보관해둔 .ko 파일 복사 (설치)
    cp "${SOURCE_KO}" "${TARGET_KO_PATH}"
    
    # 4. 커널 모듈 의존성 갱신 (필수)
    depmod -a
    
    echo "Installation complete."
fi

# --- 5. 모듈 로드 ---
# 이미 로드되어 있는지 확인하고, 없으면 로드
if ! lsmod | grep -q "${MODULE_NAME}"; then
    modprobe "${MODULE_NAME}"
fi
