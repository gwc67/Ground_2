"""可视化模块

负责网格绘制、障碍物显示、路径绘制、事件处理。
"""

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
import numpy as np

# 中文字体配置
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DengXian', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

from . import config


class Visualizer:
    """路径规划可视化"""

    def __init__(self):
        self.confirmed_obstacles = []  # 已确认障碍物 [(x,y), ...]
        self.pending_obstacles = []    # 待确认障碍物 [(x,y), ...]
        self.patrol_path = []          # 巡逻路径 [(x,y), ...]
        self.return_path = []          # 返航路径 [(x,y), ...]

        self.fig = None
        self.ax = None
        self.btn_confirm = None
        self.btn_clear = None
        self.btn_request = None
        self.btn_return = None

        self._send_callback = None  # 外部设置的发送回调

    def set_send_callback(self, callback):
        """设置发送回调函数

        Args:
            callback: 函数，接收字符串参数，发送给 STM32
        """
        self._send_callback = callback

    def setup(self):
        """创建图形和按钮"""
        self.fig, self.ax = plt.subplots(figsize=(config.FIG_WIDTH, config.FIG_HEIGHT))
        plt.subplots_adjust(bottom=config.FIG_BOTTOM)

        # 添加按钮（使用配置中的位置和大小）
        ax_confirm = plt.axes([config.BTN_CONFIRM_X, config.BTN_Y, config.BTN_WIDTH, config.BTN_HEIGHT])
        ax_clear = plt.axes([config.BTN_CLEAR_X, config.BTN_Y, config.BTN_WIDTH, config.BTN_HEIGHT])
        ax_request = plt.axes([config.BTN_REQUEST_X, config.BTN_Y, config.BTN_WIDTH, config.BTN_HEIGHT])
        ax_return = plt.axes([config.BTN_RETURN_X, config.BTN_Y, config.BTN_WIDTH, config.BTN_HEIGHT])

        self.btn_confirm = Button(ax_confirm, '确认发送')
        self.btn_clear = Button(ax_clear, '清除全部')
        self.btn_request = Button(ax_request, '请求路径')
        self.btn_return = Button(ax_return, '返航')

        self.btn_confirm.on_clicked(self._on_confirm_click)
        self.btn_clear.on_clicked(self._on_clear_click)
        self.btn_request.on_clicked(self._on_request_click)
        self.btn_return.on_clicked(self._on_return_click)

        # 添加点击事件
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)

    def update(self):
        """更新可视化"""
        self._draw_grid()
        self._draw_confirmed_obstacles()
        self._draw_pending_obstacles()
        self._draw_path(self.patrol_path, config.COLOR_PATROL, '巡逻路径', 'o')
        self._draw_path(self.return_path, config.COLOR_RETURN, '返航路径', 's')

        if self.patrol_path or self.return_path:
            self.ax.legend(loc='upper right', fontsize=10)

        self.ax.set_title(
            f'路径规划可视化 ({config.GRID_WIDTH}×{config.GRID_HEIGHT} 网格)\n'
            f'黄色=待确认 | 红色=已确认',
            fontsize=14, fontweight='bold'
        )
        self.ax.set_xlabel(config.X_LABEL, fontsize=12)
        self.ax.set_ylabel(config.Y_LABEL, fontsize=12)

        # 显示统计信息
        info_text = f"已确认：{len(self.confirmed_obstacles)} | 待确认：{len(self.pending_obstacles)}"
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                     fontsize=10, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    def _draw_grid(self):
        """绘制网格"""
        self.ax.clear()
        self.ax.set_xlim(-0.5, config.GRID_WIDTH - 0.5)
        self.ax.set_ylim(-0.5, config.GRID_HEIGHT - 0.5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle='-', linewidth=1.5, color='gray')
        self.ax.set_xticks(range(config.GRID_WIDTH))
        self.ax.set_yticks(range(config.GRID_HEIGHT))
        self.ax.set_xticklabels(range(config.GRID_WIDTH))
        self.ax.set_yticklabels(range(config.GRID_HEIGHT))
        if config.WORLD_Y_INVERTED:
            self.ax.invert_yaxis()
        self.ax.set_facecolor('#f0f0f0')

    def _draw_confirmed_obstacles(self):
        """绘制已确认障碍物（红色大圆）"""
        for (x, y) in self.confirmed_obstacles:
            circle = plt.Circle((x, y), 0.35, color=config.COLOR_CONFIRMED, alpha=0.8, zorder=10)
            self.ax.add_patch(circle)
            self.ax.text(x, y, '✗', ha='center', va='center',
                         fontsize=16, fontweight='bold', color='white', zorder=11)
            self.ax.text(x, y-0.45, f'({x},{y})', ha='center', va='top',
                         fontsize=config.TICK_FONT_SIZE, color=config.COLOR_CONFIRMED, zorder=11)

    def _draw_pending_obstacles(self):
        """绘制待确认障碍物（黄色虚线框）"""
        for (x, y) in self.pending_obstacles:
            rect = patches.Rectangle((x-0.35, y-0.35), 0.7, 0.7,
                                     linewidth=2, edgecolor='orange',
                                     facecolor=config.COLOR_PENDING, alpha=0.6,
                                     linestyle='--', zorder=9)
            self.ax.add_patch(rect)
            self.ax.text(x, y, '?', ha='center', va='center',
                         fontsize=14, fontweight='bold', color='black', zorder=10)
            self.ax.text(x, y-0.45, f'({x},{y})', ha='center', va='top',
                         fontsize=config.TICK_FONT_SIZE, color='orange', zorder=10)

    def _draw_path(self, path, color, label, marker):
        """绘制路径（0-indexed 坐标）"""
        if len(path) == 0:
            return

        path_0idx = path  # 已经是 0-indexed

        xs = [p[0] for p in path_0idx]
        ys = [p[1] for p in path_0idx]

        # 绘制路径线
        self.ax.plot(xs, ys, color=color, linewidth=2, linestyle='-',
                     marker=marker, markersize=config.MARKER_SIZE, label=label, zorder=5)

        # 标注起点
        if len(path_0idx) > 0:
            self.ax.plot(path_0idx[0][0], path_0idx[0][1], 'go',
                         markersize=config.MARKER_SIZE_START, zorder=6)
            self.ax.text(path_0idx[0][0], path_0idx[0][1], 'S', ha='center', va='center',
                         fontsize=10, fontweight='bold', color='white', zorder=7)

        # 标注终点
        if len(path_0idx) > 1:
            self.ax.plot(path_0idx[-1][0], path_0idx[-1][1], 'ro',
                         markersize=config.MARKER_SIZE_END, zorder=6)
            self.ax.text(path_0idx[-1][0], path_0idx[-1][1], 'E', ha='center', va='center',
                         fontsize=10, fontweight='bold', color='white', zorder=7)

        # 标注序号和箭头
        for i, (x, y) in enumerate(path_0idx):
            # 序号标在右上方
            self.ax.text(x + 0.15, y - 0.15, str(i + 1),
                         ha='left', va='top',
                         fontsize=config.LABEL_FONT_SIZE, color=color, zorder=8)

            # 箭头（从当前点指向下一个点）
            if i < len(path_0idx) - 1:
                next_x, next_y = path_0idx[i + 1]
                mid_x = (x + next_x) / 2
                mid_y = (y + next_y) / 2
                dx = next_x - x
                dy = next_y - y
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    # 在中间位置画小箭头
                    self.ax.annotate('',
                                     xy=(mid_x + dx * 0.1, mid_y + dy * 0.1),
                                     xytext=(mid_x - dx * 0.1, mid_y - dy * 0.1),
                                     arrowprops=dict(arrowstyle='->',
                                                     color=color,
                                                     lw=1.5),
                                     zorder=7)

    # ============== 事件处理 ==============

    def _on_click(self, event):
        """鼠标点击事件：添加/移除待确认障碍物"""
        if event.inaxes is not self.ax:
            return

        x = int(round(event.xdata))
        y = int(round(event.ydata))

        if 0 <= x < config.GRID_WIDTH and 0 <= y < config.GRID_HEIGHT:
            if (x, y) in self.pending_obstacles:
                self.pending_obstacles.remove((x, y))
                print(f"移除待确认障碍物：({x}, {y})")
            elif (x, y) in self.confirmed_obstacles:
                print(f"已确认障碍物 ({x}, {y})，请先点击'清除全部'再重新设置")
            else:
                self.pending_obstacles.append((x, y))
                print(f"添加待确认障碍物：({x}, {y})")

            self.update()
            plt.draw()

    def _on_confirm_click(self, event):
        """确认按钮：将待确认障碍物转为已确认并发送"""
        if len(self.pending_obstacles) == 0:
            print("没有待确认的障碍物")
            return

        all_obstacles = []
        for obs in self.confirmed_obstacles + self.pending_obstacles:
            if obs not in all_obstacles:
                all_obstacles.append(obs)

        if len(all_obstacles) > 3:
            print(f"警告：共有 {len(all_obstacles)} 个障碍物，但飞控只支持 3 个禁飞区")
            print(f"将只发送前 3 个：{all_obstacles[:3]}")

        no_fly_zones = all_obstacles[:3]
        self.pending_obstacles.clear()
        self.confirmed_obstacles = list(no_fly_zones)

        if self._send_callback and len(no_fly_zones) > 0:
            coords = []
            for (x, y) in no_fly_zones:
                # 统一格式：(x=列, y=行)
                coords.extend([x, y])

            while len(coords) < 6:
                coords.extend([0, 0])

            map_message = f"MAP:{','.join(map(str, coords))}"
            self._send_callback(map_message)
            print(f"已发送禁飞区 (col,row)：{no_fly_zones}")

        self.update()
        plt.draw()

    def _on_clear_click(self, event):
        """清除按钮：清除所有障碍物和路径"""
        self.confirmed_obstacles.clear()
        self.pending_obstacles.clear()
        self.patrol_path.clear()
        self.return_path.clear()
        print("已清除所有障碍物和路径")
        self.update()
        plt.draw()

    def _on_request_click(self, event):
        """请求路径按钮：请求飞控发送巡逻路径"""
        if self._send_callback:
            self._send_callback("REQUEST_PATH")
            print("已请求飞控发送巡逻路径")

    def _on_return_click(self, event):
        """返航按钮：请求飞控发送返航路径"""
        if self._send_callback:
            self._send_callback("REQUEST_RETURN")
            print("已请求飞控发送返航路径")

    def show(self):
        """显示图形"""
        self.setup()
        self.update()
        plt.tight_layout()

    def pause(self, interval: float):
        """暂停刷新"""
        plt.pause(interval)

    def close(self):
        """关闭图形"""
        plt.close(self.fig)
