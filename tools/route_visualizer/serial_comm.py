"""串口通信模块

负责与 STM32 的串口收发，只管数据搬运，不管解析。
"""

import serial


class SerialComm:
    """串口通信封装"""

    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        """初始化串口

        Args:
            port: 串口号（如 'COM3' 或 '/dev/ttyUSB0'）
            baudrate: 波特率
            timeout: 读取超时（秒）
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: serial.Serial | None = None

    def open(self) -> bool:
        """打开串口

        Returns:
            True 成功，False 失败
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            return True
        except Exception as e:
            print(f"无法打开串口 {self.port}: {e}")
            return False

    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, message: str) -> bool:
        """发送字符串（自动加 \\n）

        Args:
            message: 要发送的字符串（不含 \\n）

        Returns:
            True 成功，False 失败或未连接
        """
        if not self.ser or not self.ser.is_open:
            print(f"[离线模式] TX to stm32: {message}")
            return False

        try:
            print(f"TX to stm32: {message}")
            self.ser.write((message + "\n").encode('utf-8'))
            return True
        except Exception as e:
            print(f"发送失败：{e}")
            return False

    def read_line(self) -> str | None:
        """读取一行（阻塞直到收到完整行或超时）

        Returns:
            读取到的字符串（已 strip），或 None
        """
        if not self.ser or not self.ser.is_open:
            return None

        try:
            # 等待数据到达
            if self.ser.in_waiting == 0:
                # 短暂等待数据
                import time
                time.sleep(0.01)
                if self.ser.in_waiting == 0:
                    return None

            # 读取直到遇到换行符或超时
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            return line if line else None
        except Exception as e:
            print(f"串口读取错误：{e}")

        return None
