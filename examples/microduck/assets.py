"""固定版本资产准备与完整性校验。"""

import argparse
import hashlib
import json
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent
DEFAULT_CACHE = ROOT / ".cache"


def verify(cache=DEFAULT_CACHE, entries=None):
    # SHA-256 同时检查文件完整性与固定版本；该函数不访问网络。
    cache = Path(cache).resolve()
    for item in entries if entries is not None else lock()["files"]:
        path = asset_path(cache, item["path"])
        if not path.is_file():
            raise ValueError(f"missing asset {path}; run assets.py prepare")
        if digest(path) != item["sha256"]:
            raise ValueError(
                f"checksum/version mismatch: {path}; run assets.py prepare"
            )
    return cache


def lock():
    return json.loads((ROOT / "assets.lock.json").read_text())


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def asset_path(cache, name):
    # 解析相对路径和符号链接后检查边界，确保文件仍位于缓存目录内。
    path = (cache / name).resolve()
    if not path.is_relative_to(cache) or path == cache:
        raise ValueError(f"unsafe asset path: {name}")
    return path


def prepare(cache=DEFAULT_CACHE):
    cache = Path(cache).resolve()

    def fetch(item):
        path = asset_path(cache, item["path"])
        if path.is_file() and digest(path) == item["sha256"]:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = None
        try:
            # 先写同目录临时文件，全部下载且哈希匹配后再替换正式文件。
            with urlopen(item["url"], timeout=30) as response:
                with tempfile.NamedTemporaryFile(
                    dir=path.parent, delete=False
                ) as stream:
                    temp = Path(stream.name)
                    while chunk := response.read(1024 * 1024):
                        stream.write(chunk)
            if digest(temp) != item["sha256"]:
                raise ValueError(f"download checksum mismatch: {item['path']}")
            temp.replace(path)
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)

    with ThreadPoolExecutor(max_workers=4) as pool:
        # 并行获取独立资产；消费 map 结果以等待完成并传播下载异常。
        list(pool.map(fetch, lock()["files"]))
    return verify(cache)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["prepare", "verify"])
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    args = parser.parse_args()
    try:
        result = (
            prepare(args.cache) if args.command == "prepare" else verify(args.cache)
        )
        print(json.dumps({"status": "verified", "cache": str(result)}))
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
