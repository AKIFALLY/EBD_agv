"""
座標轉換工具函數

提供 KUKA 座標系統兼容的座標轉換功能，支援實際座標與像素座標之間的轉換。
用於統一 Node 和 KUKA 系統之間的座標處理邏輯。
"""

from typing import Tuple


def unit_to_px(y: float, x: float) -> Tuple[float, float]:
    """
    將實際單位座標轉換為像素座標
    使用與 KUKA 系統相同的轉換比例: 12.5mm = 1px

    Args:
        y: 實際 y 座標 (mm)
        x: 實際 x 座標 (mm)

    Returns:
        tuple: (pixel_y, pixel_x) 像素座標

    Note:
        此函數與 KUKA 的 kuka_unit_2_px 函數邏輯完全相同
        轉換公式: pixel_y = y / 12.5, pixel_x = x / 12.5
    """
    return y / 12.5, x / 12.5


def px_to_unit(pixel_y: float, pixel_x: float) -> Tuple[float, float]:
    """
    將像素座標轉換為實際單位座標
    使用與 KUKA 系統相同的轉換比例: 12.5mm = 1px

    Args:
        pixel_y: 像素 y 座標
        pixel_x: 像素 x 座標

    Returns:
        tuple: (y, x) 實際座標 (mm)

    Note:
        此函數是 unit_to_px 的反向操作
        轉換公式: y = pixel_y * 12.5, x = pixel_x * 12.5
    """
    return pixel_y * 12.5, pixel_x * 12.5


def calculate_pixel_coordinates(x: float, y: float) -> Tuple[float, float]:
    """
    計算節點的像素座標
    便捷函數，用於統一處理節點座標轉換

    Args:
        x: 節點 X 座標 (mm)
        y: 節點 Y 座標 (mm)

    Returns:
        tuple: (pixel_x, pixel_y) 像素座標

    Note:
        與 unit_to_px 函數相比，此函數返回的座標順序為 (pixel_x, pixel_y)
        這樣更符合一般的 (x, y) 座標習慣
    """
    pixel_y, pixel_x = unit_to_px(y, x)
    return pixel_x, pixel_y


class CoordinateConverter:
    """
    座標轉換器類別
    提供批量座標轉換和更複雜的座標處理功能
    """

    # 座標轉換比例常數
    CONVERSION_RATIO = 12.5  # 12.5mm = 1px

    @classmethod
    def convert_node_coordinates(cls, node_data: dict) -> dict:
        """
        轉換節點資料中的座標，自動計算像素座標

        Args:
            node_data: 包含 x, y 座標的節點資料字典

        Returns:
            dict: 添加了 pixel_x, pixel_y 的節點資料
        """
        if 'x' in node_data and 'y' in node_data:
            pixel_x, pixel_y = calculate_pixel_coordinates(
                node_data['x'], node_data['y']
            )
            node_data['pixel_x'] = pixel_x
            node_data['pixel_y'] = pixel_y

        return node_data

    @classmethod
    def batch_convert_coordinates(cls, nodes_data: list) -> list:
        """
        批量轉換多個節點的座標

        Args:
            nodes_data: 節點資料列表，每個元素包含 x, y 座標

        Returns:
            list: 添加了像素座標的節點資料列表
        """
        return [cls.convert_node_coordinates(node) for node in nodes_data]