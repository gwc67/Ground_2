"""协议解析模块

解析 STM32 发送的 ASCII 命令：
  MAP:x1,y1,x2,y2,x3,y3  — 禁飞区坐标
  PATH:x1,y1,x2,y2,...    — 巡逻路径
  RETURN:x1,y1,x2,y2,...  — 返航路径
"""


def parse_line(line: str) -> dict | None:
    """解析一行串口数据

    Args:
        line: 以 \\n 结尾的字符串（已 strip）

    Returns:
        解析结果字典，格式：
        - {'type': 'MAP', 'zones': [(x1,y1), (x2,y2), (x3,y3)]}
        - {'type': 'PATH', 'coords': [(x1,y1), (x2,y2), ...]}
        - {'type': 'RETURN', 'coords': [(x1,y1), (x2,y2), ...]}
        - None: 空行或未知格式

    Examples:
        >>> parse_line("MAP:4,2,4,3,4,4")
        {'type': 'MAP', 'zones': [(4, 2), (4, 3), (4, 4)]}
        >>> parse_line("PATH:9,1,4,1,4,2")
        {'type': 'PATH', 'coords': [(9, 1), (4, 1), (4, 2)]}
        >>> parse_line("")
        None
    """
    if not line:
        return None

    if line.startswith("MAP:"):
        return _parse_map(line[4:])
    elif line.startswith("PATH:"):
        return _parse_coords(line[5:], 'PATH')
    elif line.startswith("RETURN:"):
        return _parse_coords(line[7:], 'RETURN')

    return None


def _parse_coords(text: str, msg_type: str) -> dict:
    """解析坐标对：x1,y1,x2,y2,...

    跳过非数字部分（处理串口缓冲粘连问题）
    """
    parts = [p for p in text.split(',') if p]
    coords = []
    for i in range(0, len(parts) - 1, 2):
        try:
            x = int(parts[i])
            y = int(parts[i + 1])
            coords.append((x, y))
        except ValueError:
            # 跳过非数字部分（如粘连的 "MAP:1"）
            continue
    return {'type': msg_type, 'coords': coords}


def _parse_map(text: str) -> dict:
    """解析禁飞区：x1,y1,x2,y2,x3,y3"""
    parts = [p for p in text.split(',') if p]
    zones = []
    for i in range(0, len(parts) - 1, 2):
        try:
            x = int(parts[i])
            y = int(parts[i + 1])
            zones.append((x, y))
        except ValueError:
            continue
    return {'type': 'MAP', 'zones': zones}
