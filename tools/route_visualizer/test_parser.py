"""parser 模块测试"""

from .parser import parse_line


def test_parse_map():
    """测试 MAP 解析"""
    result = parse_line("MAP:4,2,4,3,4,4")
    assert result['type'] == 'MAP'
    assert result['zones'] == [(4, 2), (4, 3), (4, 4)]
    print("[PASS] test_parse_map")


def test_parse_path():
    """测试 PATH 解析"""
    result = parse_line("PATH:9,1,4,1,4,2")
    assert result['type'] == 'PATH'
    assert result['coords'] == [(9, 1), (4, 1), (4, 2)]
    print("[PASS] test_parse_path")


def test_parse_return():
    """测试 RETURN 解析"""
    result = parse_line("RETURN:4,2,9,1")
    assert result['type'] == 'RETURN'
    assert result['coords'] == [(4, 2), (9, 1)]
    print("[PASS] test_parse_return")


def test_parse_empty():
    """测试空行"""
    result = parse_line("")
    assert result is None
    print("[PASS] test_parse_empty")


def test_parse_unknown():
    """测试未知格式"""
    result = parse_line("UNKNOWN:data")
    assert result is None
    print("[PASS] test_parse_unknown")


if __name__ == '__main__':
    test_parse_map()
    test_parse_path()
    test_parse_return()
    test_parse_empty()
    test_parse_unknown()
    print("\nAll tests passed!")
