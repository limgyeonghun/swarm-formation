#!/bin/bash

echo "=== Jetson Orin 성능 모니터링 ==="
echo "Ctrl+C로 종료"

while true; do
    clear
    echo "=== $(date) ==="
    
    # CPU 정보
    echo "CPU 주파수: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | head -1) Hz"
    echo "CPU 사용률: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    
    # 메모리 정보
    echo "메모리 사용률:"
    free -h | grep -E "Mem|Swap"
    
    # 온도 정보
    echo "CPU 온도: $(cat /sys/class/thermal/thermal_zone*/temp | head -1 | awk '{print $1/1000}')°C"
    
    # Jetson 모드
    echo "Jetson 모드:"
    sudo nvpmodel -q | grep "Power Mode"
    
    # 프로세스 정보
    echo "ROS 프로세스:"
    ps aux | grep -E "path_manager|path_optimizer" | grep -v grep
    
    sleep 2
done
