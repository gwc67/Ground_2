"""配置管理"""

# 串口配置
COM_PORT = 'COM8'
BAUD_RATE = 500000

# 网格配置
GRID_WIDTH = 9       # 列数
GRID_HEIGHT = 7      # 行数
CELL_SIZE_CM = 50    # 每格 50cm

# ============== 世界坐标系（cm） ==============
# grid_x = 列 (0 ~ GRID_WIDTH-1)  → world_x = grid_x * CELL_SIZE_CM
# grid_y = 行 (0 ~ GRID_HEIGHT-1) → world_y = grid_y * CELL_SIZE_CM
WORLD_X_MAX = (GRID_WIDTH - 1) * CELL_SIZE_CM   # 400
WORLD_Y_MAX = (GRID_HEIGHT - 1) * CELL_SIZE_CM  # 300

# Y 轴方向：True = 向下增加（屏幕坐标系，原点在左上），False = 向上增加（数学坐标系，原点在左下）
WORLD_Y_INVERTED = False

# 坐标轴标签
X_LABEL = 'X（列）'
Y_LABEL = 'Y（行）'

# 刷新间隔（秒）
UPDATE_INTERVAL = 0.1

# 颜色配置
COLOR_PATROL = 'blue'
COLOR_RETURN = 'green'
COLOR_PENDING = 'yellow'
COLOR_CONFIRMED = 'red'

# 标记大小
MARKER_SIZE = 10
MARKER_SIZE_START = 14
MARKER_SIZE_END = 14

# 字体大小
LABEL_FONT_SIZE = 9
TICK_FONT_SIZE = 8

# 窗口配置
FIG_WIDTH = 10       # 图形宽度（英寸）
FIG_HEIGHT = 9       # 图形高度（英寸）
FIG_BOTTOM = 0.28    # 底部留白比例（给按钮留空间）

# 按钮配置 [x, y, width, height]（比例坐标 0-1）
BTN_WIDTH = 0.15
BTN_HEIGHT = 0.035
BTN_Y = 0         # 按钮 y 位置
BTN_CONFIRM_X = 0.1  # 确认发送按钮 x 位置
BTN_CLEAR_X = 0.3    # 清除全部按钮 x 位置
BTN_REQUEST_X = 0.5  # 请求路径按钮 x 位置
BTN_RETURN_X = 0.7   # 返航按钮 x 位置
