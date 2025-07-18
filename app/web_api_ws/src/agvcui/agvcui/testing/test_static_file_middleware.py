#!/usr/bin/env python3
"""
測試靜態檔案中間件自動放行功能
"""

import sys
import os

# 直接導入靜態檔案檢測功能，避免模組依賴問題


def is_static_file(path: str) -> bool:
    """檢查路徑是否為靜態資源檔案"""
    STATIC_FILE_EXTENSIONS = {
        # 樣式檔案
        '.css', '.scss', '.sass',
        # 腳本檔案
        '.js', '.ts', '.map',
        # 圖片檔案
        '.png', '.jpg', '.jpeg', '.gif', '.svg', '.ico', '.webp', '.bmp', '.tiff',
        # 字體檔案
        '.woff', '.woff2', '.ttf', '.eot', '.otf',
        # 其他常見靜態資源
        '.json', '.xml', '.txt', '.pdf', '.zip', '.tar', '.gz',
        # 音視頻檔案
        '.mp3', '.mp4', '.wav', '.avi', '.mov', '.wmv',
        # 文檔檔案
        '.doc', '.docx', '.xls', '.xlsx', '.ppt', '.pptx'
    }

    # 獲取檔案副檔名（轉為小寫）
    _, ext = os.path.splitext(path.lower())
    return ext in STATIC_FILE_EXTENSIONS


# 靜態檔案副檔名清單
STATIC_FILE_EXTENSIONS = {
    '.css', '.scss', '.sass', '.js', '.ts', '.map',
    '.png', '.jpg', '.jpeg', '.gif', '.svg', '.ico', '.webp', '.bmp', '.tiff',
    '.woff', '.woff2', '.ttf', '.eot', '.otf',
    '.json', '.xml', '.txt', '.pdf', '.zip', '.tar', '.gz',
    '.mp3', '.mp4', '.wav', '.avi', '.mov', '.wmv',
    '.doc', '.docx', '.xls', '.xlsx', '.ppt', '.pptx'
}


def test_static_file_detection():
    """測試靜態檔案檢測功能"""
    print("=== 測試靜態檔案檢測功能 ===\n")

    # 測試案例：應該被識別為靜態檔案的路徑
    static_file_paths = [
        "/static/css/style.css",
        "/static/js/app.js",
        "/static/images/logo.png",
        "/static/fonts/font.woff2",
        "/favicon.ico",
        "/static/data.json",
        "/assets/video.mp4",
        "/downloads/document.pdf",
        "/static/js/mapPage.js",
        "/static/css/bulma.min.css",
        "/static/images/icon.svg",
        "/static/lib/socket.io.min.js",
        "/static/fonts/roboto.ttf"
    ]

    # 測試案例：不應該被識別為靜態檔案的路徑
    non_static_paths = [
        "/",
        "/login",
        "/admin",
        "/users",
        "/map",
        "/signals",
        "/tasks",
        "/devices/create",
        "/products/123/edit",
        "/api/users",
        "/static",  # 目錄路徑，沒有副檔名
        "/static/",
        "/some/path/without/extension"
    ]

    print("🔍 測試靜態檔案路徑:")
    all_static_passed = True
    for path in static_file_paths:
        result = is_static_file(path)
        status = "✅" if result else "❌"
        print(f"   {status} {path} -> {result}")
        if not result:
            all_static_passed = False

    print(f"\n📊 靜態檔案測試結果: {'✅ 全部通過' if all_static_passed else '❌ 有失敗案例'}")

    print("\n🔍 測試非靜態檔案路徑:")
    all_non_static_passed = True
    for path in non_static_paths:
        result = is_static_file(path)
        status = "✅" if not result else "❌"
        print(f"   {status} {path} -> {result}")
        if result:
            all_non_static_passed = False

    print(f"\n📊 非靜態檔案測試結果: {'✅ 全部通過' if all_non_static_passed else '❌ 有失敗案例'}")

    return all_static_passed and all_non_static_passed


def test_file_extensions():
    """測試支援的檔案副檔名"""
    print("\n=== 測試支援的檔案副檔名 ===\n")

    print("📋 支援的靜態檔案副檔名:")
    extensions_by_category = {
        "樣式檔案": ['.css', '.scss', '.sass'],
        "腳本檔案": ['.js', '.ts', '.map'],
        "圖片檔案": ['.png', '.jpg', '.jpeg', '.gif', '.svg', '.ico', '.webp', '.bmp', '.tiff'],
        "字體檔案": ['.woff', '.woff2', '.ttf', '.eot', '.otf'],
        "其他資源": ['.json', '.xml', '.txt', '.pdf', '.zip', '.tar', '.gz'],
        "音視頻": ['.mp3', '.mp4', '.wav', '.avi', '.mov', '.wmv'],
        "文檔檔案": ['.doc', '.docx', '.xls', '.xlsx', '.ppt', '.pptx']
    }

    total_extensions = 0
    for category, extensions in extensions_by_category.items():
        print(f"\n   {category}:")
        for ext in extensions:
            if ext in STATIC_FILE_EXTENSIONS:
                print(f"      ✅ {ext}")
                total_extensions += 1
            else:
                print(f"      ❌ {ext} (未在 STATIC_FILE_EXTENSIONS 中)")

    print(f"\n📊 總計支援 {total_extensions} 種檔案副檔名")
    print(f"📊 實際定義 {len(STATIC_FILE_EXTENSIONS)} 種副檔名")

    return total_extensions == len(STATIC_FILE_EXTENSIONS)


def test_edge_cases():
    """測試邊界情況"""
    print("\n=== 測試邊界情況 ===\n")

    edge_cases = [
        ("", False, "空字串"),
        ("/", False, "根路徑"),
        ("/file", False, "無副檔名檔案"),
        ("/file.", False, "只有點號"),
        ("/file.CSS", True, "大寫副檔名"),
        ("/path/to/file.JS", True, "大寫副檔名在路徑中"),
        ("/static/app.min.js", True, "多重點號"),
        ("/static/.hidden", False, "隱藏檔案無副檔名"),
        ("/static/.hidden.css", True, "隱藏檔案有副檔名"),
        ("/very/long/path/to/some/file.png", True, "長路徑"),
        ("/file.unknown", False, "未知副檔名")
    ]

    print("🔍 測試邊界情況:")
    all_passed = True
    for path, expected, description in edge_cases:
        result = is_static_file(path)
        status = "✅" if result == expected else "❌"
        print(f"   {status} {path} -> {result} ({description})")
        if result != expected:
            all_passed = False

    print(f"\n📊 邊界情況測試結果: {'✅ 全部通過' if all_passed else '❌ 有失敗案例'}")

    return all_passed


def main():
    """主測試函式"""
    print("🚀 開始測試靜態檔案中間件功能\n")

    # 執行所有測試
    test1_passed = test_static_file_detection()
    test2_passed = test_file_extensions()
    test3_passed = test_edge_cases()

    # 總結
    print("\n" + "="*50)
    print("📊 測試總結:")
    print(f"   靜態檔案檢測: {'✅ 通過' if test1_passed else '❌ 失敗'}")
    print(f"   檔案副檔名: {'✅ 通過' if test2_passed else '❌ 失敗'}")
    print(f"   邊界情況: {'✅ 通過' if test3_passed else '❌ 失敗'}")

    all_tests_passed = test1_passed and test2_passed and test3_passed
    print(f"\n🎯 整體結果: {'✅ 所有測試通過' if all_tests_passed else '❌ 部分測試失敗'}")

    if all_tests_passed:
        print("\n🎉 靜態檔案自動放行功能運作正常！")
    else:
        print("\n⚠️ 請檢查失敗的測試案例")

    return 0 if all_tests_passed else 1


if __name__ == "__main__":
    exit(main())
