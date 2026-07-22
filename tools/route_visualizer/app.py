"""主循环模块

负责启动和运行主循环：读串口 → 解析 → 更新视图。
"""

from . import config
from .serial_comm import SerialComm
from .parser import parse_line
from .visualizer import Visualizer


def main():
    """主函数"""
    # 创建串口
    serial = SerialComm(config.COM_PORT, config.BAUD_RATE)
    if serial.open():
        print(f"已打开串口：{config.COM_PORT} @ {config.BAUD_RATE}")
    else:
        print(f"无法打开串口 {config.COM_PORT}，将以离线模式运行")

    # 创建可视化
    visualizer = Visualizer()
    visualizer.set_send_callback(serial.send)
    visualizer.show()

    # 主循环
    try:
        while True:
            # 读取串口数据
            line = serial.read_line()
            if line:
                # 只打印关键信息（PATH/CNT/MAP）
                if line.startswith('PATH:') or line.startswith('CNT:') or line.startswith('MAP:'):
                    print(f"RX: {line}")

                result = parse_line(line)
                if result:
                    if result['type'] == 'PATH':
                        # STM32 输出格式是 (x=列, y=行)，直接绘制
                        visualizer.patrol_path = result['coords']
                        print(f"收到巡逻路径：{len(visualizer.patrol_path)} 个点")
                        for i, (x, y) in enumerate(visualizer.patrol_path):
                            print(f"  [{i}] ({x}, {y})")
                    elif result['type'] == 'RETURN':
                        visualizer.return_path = result['coords']
                        print(f"收到返航路径：{len(visualizer.return_path)} 个点")
                        for i, (x, y) in enumerate(visualizer.return_path):
                            print(f"  [{i}] ({x}, {y})")
                    elif result['type'] == 'MAP':
                        # MAP 消息忽略（由上位机自己管理）
                        pass

            # 更新可视化
            visualizer.update()
            visualizer.pause(config.UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        serial.close()
        visualizer.close()
        print("串口已关闭")
