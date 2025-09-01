#!/bin/bash

echo "=== Jetson Orin 성능 최적화 스크립트 ==="

# 1. CPU 성능 모드 설정
echo "1. CPU 성능 모드 설정 중..."
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
echo "CPU 성능 모드 설정 완료"

# 2. Jetson 최대 성능 모드
echo "2. Jetson 최대 성능 모드 설정 중..."
sudo nvpmodel -m 0
sudo jetson_clocks
echo "Jetson 최대 성능 모드 설정 완료"

# 3. 스왑 메모리 비활성화
echo "3. 스왑 메모리 비활성화 중..."
sudo swapoff -a
echo "스왑 메모리 비활성화 완료"

# 4. 캐시 정리
echo "4. 시스템 캐시 정리 중..."
sudo sh -c "echo 3 > /proc/sys/vm/drop_caches"
echo "캐시 정리 완료"

# 5. 불필요한 서비스 비활성화
echo "5. 불필요한 서비스 비활성화 중..."
sudo systemctl stop lightdm 2>/dev/null || true
sudo systemctl stop bluetooth 2>/dev/null || true
sudo systemctl stop avahi-daemon 2>/dev/null || true
echo "서비스 비활성화 완료"

# 6. 현재 상태 출력
echo "=== 최적화 완료 ==="
echo "CPU 주파수:"
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | head -1
echo "Jetson 모드:"
sudo nvpmodel -q
echo "메모리 상태:"
free -h
echo "CPU 사용률:"
top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1

echo "=== 최적화 완료! ==="
