# 1. 切到 DInput
for h in /sys/class/hidraw/hidraw*; do
  if grep -q "0DB0" "$h/device/uevent" 2>/dev/null; then
    dev="/dev/$(basename $h)"
    echo "Trying $dev ..."
    sudo python3 -c "
f = open('$dev', 'wb')
f.write(bytes([0x0f, 0x00, 0x00, 0x3c, 0x24, 0x02, 0x00, 0x00]))
f.close()
print('Sent to $dev')
" 2>/dev/null
  fi
done

# 2. 等待重枚举
sleep 3

# 3. 确认是 1902
cat /sys/class/hidraw/hidraw*/device/uevent | grep 0DB0

# 4. 看 input 设备
cat /proc/bus/input/devices | grep -A 8 -i "0db0"

# 1. 先试切回 XInput
for h in /sys/class/hidraw/hidraw*; do
  if grep -q "0DB0" "$h/device/uevent" 2>/dev/null; then
    dev="/dev/$(basename $h)"
    echo "Trying switch to XInput on $dev ..."
    sudo python3 -c "
f = open('$dev', 'wb')
f.write(bytes([0x0f, 0x00, 0x00, 0x3c, 0x24, 0x01, 0x00, 0x00]))
f.close()
print('Sent to $dev')
"
  fi
done

# 2. 等 3 秒
sleep 3

# 3. 检查是否回到 1901
cat /sys/class/hidraw/hidraw*/device/uevent | grep 0DB0
