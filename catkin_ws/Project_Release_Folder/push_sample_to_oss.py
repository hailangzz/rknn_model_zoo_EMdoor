# -*- coding: utf-8 -*-

import os
from concurrent.futures import ThreadPoolExecutor, as_completed

import oss2

# ============================================================
# 创建 OSS Bucket Client
# ============================================================

# AccessKey 建议通过环境变量读取
ACCESS_KEY_ID = os.getenv("OSS_ACCESS_KEY_ID")
ACCESS_KEY_SECRET = os.getenv("OSS_ACCESS_KEY_SECRET")

OSS_ENDPOINT = "https://oss-cn-shenzhen.aliyuncs.com"

OSS_BUCKET = "robot-ai-platform"

if not ACCESS_KEY_ID or not ACCESS_KEY_SECRET:
    raise RuntimeError(
        "未找到 OSS AccessKey，请设置环境变量："
        "OSS_ACCESS_KEY_ID / OSS_ACCESS_KEY_SECRET"
    )

# 创建 OSS Auth
auth = oss2.Auth(
    ACCESS_KEY_ID,
    ACCESS_KEY_SECRET
)

# 创建 OSS Bucket
bucket = oss2.Bucket(
    auth,
    OSS_ENDPOINT,
    OSS_BUCKET
)


# ============================================================
# 获取所有图片文件
# ============================================================


def get_all_files(local_dir):
    files = []

    image_exts = {
        ".jpg",
        ".jpeg",
        ".png",
        ".bmp",
        ".webp"
    }

    for root, dirs, filenames in os.walk(local_dir):

        for filename in filenames:

            file_path = os.path.join(
                root,
                filename
            )

            ext = os.path.splitext(
                filename
            )[1].lower()

            if os.path.isfile(file_path) and ext in image_exts:
                files.append(file_path)

    return files


# ============================================================
# 获取 OSS Object Key
# ============================================================


def get_oss_key(
        local_file,
        local_dir,
        oss_prefix
):
    relative_path = os.path.relpath(
        local_file,
        local_dir
    )

    relative_path = relative_path.replace(
        "\\",
        "/"
    )

    oss_key = (
            oss_prefix.rstrip("/")
            + "/"
            + relative_path
    )

    return oss_key


# ============================================================
# 判断文件是否存在
# ============================================================


def oss_file_exists(
        oss_bucket,
        key,
        local_file
):
    try:

        # OSS HEAD Object
        response = oss_bucket.head_object(key)

        local_size = os.path.getsize(
            local_file
        )

        # oss2 HeadObjectResult
        oss_size = response.content_length

        return local_size == oss_size

    except oss2.exceptions.NoSuchKey:

        return False

    except oss2.exceptions.NotFound:

        return False

    except oss2.exceptions.ServerError as e:

        # 某些 OSS 版本 / 情况下可能通过 ServerError 返回 404
        if e.status == 404:
            return False

        raise


# ============================================================
# 上传单个文件
# ============================================================


def upload_file(local_file):
    oss_key = get_oss_key(
        local_file,
        LOCAL_DIR,
        OSS_PREFIX
    )

    try:

        # ----------------------------------------------------
        # 判断 OSS 是否已经存在
        # ----------------------------------------------------

        if oss_file_exists(
                bucket,
                oss_key,
                local_file
        ):
            return {
                "status": "skip",
                "file": local_file
            }

    except Exception as e:

        return {
            "status": "error",
            "file": local_file,
            "error": str(e)
        }

    # --------------------------------------------------------
    # 上传重试
    # --------------------------------------------------------

    for attempt in range(
            1,
            MAX_RETRIES + 1
    ):

        try:

            # OSS 上传本地文件
            bucket.put_object_from_file(
                oss_key,
                local_file
            )

            return {
                "status": "success",
                "file": local_file
            }

        except (
                oss2.exceptions.OssError,
                OSError
        ) as e:

            if attempt == MAX_RETRIES:
                return {
                    "status": "error",
                    "file": local_file,
                    "error": str(e)
                }

    return {
        "status": "error",
        "file": local_file,
        "error": "unknown"
    }


# ============================================================
# 同步
# ============================================================


def sync_to_oss():
    print("=" * 60)

    print("OSS IMAGE SYNC")

    print("=" * 60)

    print(
        f"LOCAL : {LOCAL_DIR}"
    )

    print(
        f"OSS   : oss://{OSS_BUCKET}/{OSS_PREFIX}"
    )

    print(
        f"THREADS : {MAX_WORKERS}"
    )

    print("=" * 60)

    # --------------------------------------------------------
    # 获取所有图片
    # --------------------------------------------------------

    files = get_all_files(
        LOCAL_DIR
    )

    total = len(files)

    print(
        f"发现图片数量: {total}"
    )

    if total == 0:
        return

    success = 0

    skipped = 0

    failed = 0

    failed_files = []

    # --------------------------------------------------------
    # 多线程上传
    # --------------------------------------------------------

    with ThreadPoolExecutor(
            max_workers=MAX_WORKERS
    ) as executor:

        futures = [
            executor.submit(
                upload_file,
                f
            )
            for f in files
        ]

        for idx, future in enumerate(
                as_completed(futures),
                1
        ):

            result = future.result()

            status = result["status"]

            # ------------------------------------------------
            # 上传成功
            # ------------------------------------------------

            if status == "success":

                success += 1

                print(
                    f"[{idx}/{total}] "
                    f"上传成功 "
                    f"{result['file']}"
                )

            # ------------------------------------------------
            # 已存在
            # ------------------------------------------------

            elif status == "skip":

                skipped += 1

                print(
                    f"[{idx}/{total}] "
                    f"已存在 "
                    f"{result['file']}"
                )

            # ------------------------------------------------
            # 上传失败
            # ------------------------------------------------

            else:

                failed += 1

                failed_files.append(
                    result
                )

                print(
                    f"[{idx}/{total}] "
                    f"上传失败 "
                    f"{result['file']}"
                )

                print(
                    result["error"]
                )

    # ========================================================
    # 结果统计
    # ========================================================

    print()

    print("=" * 60)

    print("完成")

    print("=" * 60)

    print(
        f"总数     : {total}"
    )

    print(
        f"成功     : {success}"
    )

    print(
        f"跳过     : {skipped}"
    )

    print(
        f"失败     : {failed}"
    )

    # --------------------------------------------------------
    # 失败列表
    # --------------------------------------------------------

    if failed_files:

        print("\n失败列表:")

        for item in failed_files:
            print(
                item["file"]
            )


# ============================================================
# 配置
# ============================================================


# 注意：
# 指向 carpet_detect 目录
LOCAL_DIR = (
    "/home/robot/share/AI_Program/"
    "CarpetSegmentProject/"
    "spatial_location_val_images/"
    "carpet_detect"
)

OSS_BUCKET = "robot-ai-platform"

OSS_PREFIX = (
    "datasets/"
    "carpet_detection/"
    "source/images"
)

MAX_WORKERS = 4

MAX_RETRIES = 3

# ============================================================
# main
# ============================================================


if __name__ == "__main__":
    sync_to_oss()
