import os
from concurrent.futures import ThreadPoolExecutor, as_completed

import boto3
from botocore.exceptions import BotoCoreError, ClientError

# ============================================================
# 创建 S3 Client
# ============================================================

s3 = boto3.client("s3")


# ============================================================
# 获取所有图片文件
# ============================================================


def get_all_files(local_dir):
    files = []

    image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

    for root, dirs, filenames in os.walk(local_dir):

        for filename in filenames:

            file_path = os.path.join(root, filename)

            ext = os.path.splitext(filename)[1].lower()

            if os.path.isfile(file_path) and ext in image_exts:
                files.append(file_path)

    return files


# ============================================================
# 获取 S3 Key
# ============================================================


def get_s3_key(local_file, local_dir, s3_prefix):
    relative_path = os.path.relpath(local_file, local_dir)

    relative_path = relative_path.replace("\\", "/")

    s3_key = s3_prefix.rstrip("/") + "/" + relative_path

    return s3_key


# ============================================================
# 判断文件是否存在
# ============================================================


def s3_file_exists(bucket, key, local_file):
    try:

        response = s3.head_object(Bucket=bucket, Key=key)

        local_size = os.path.getsize(local_file)

        s3_size = response["ContentLength"]

        return local_size == s3_size

    except ClientError as e:

        code = e.response["Error"]["Code"]

        if code in ("404", "NoSuchKey"):
            return False

        raise


# ============================================================
# 上传单个文件
# ============================================================


def upload_file(local_file):
    s3_key = get_s3_key(local_file, LOCAL_DIR, S3_PREFIX)

    try:

        if s3_file_exists(S3_BUCKET, s3_key, local_file):
            return {"status": "skip", "file": local_file}

    except Exception as e:

        return {"status": "error", "file": local_file, "error": str(e)}

    for attempt in range(1, MAX_RETRIES + 1):

        try:

            s3.upload_file(local_file, S3_BUCKET, s3_key)

            return {"status": "success", "file": local_file}

        except (BotoCoreError, ClientError) as e:

            if attempt == MAX_RETRIES:
                return {"status": "error", "file": local_file, "error": str(e)}

    return {"status": "error", "file": local_file, "error": "unknown"}


# ============================================================
# 同步
# ============================================================


def sync_to_s3():
    print("=" * 60)
    print("S3 IMAGE SYNC")
    print("=" * 60)

    print(f"LOCAL : {LOCAL_DIR}")

    print(f"S3    : s3://{S3_BUCKET}/{S3_PREFIX}")

    print(f"THREADS : {MAX_WORKERS}")

    print("=" * 60)

    files = get_all_files(LOCAL_DIR)

    total = len(files)

    print(f"发现图片数量: {total}")

    if total == 0:
        return

    success = 0
    skipped = 0
    failed = 0

    failed_files = []

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:

        futures = [executor.submit(upload_file, f) for f in files]

        for idx, future in enumerate(as_completed(futures), 1):

            result = future.result()

            status = result["status"]

            if status == "success":

                success += 1

                print(f"[{idx}/{total}] 上传成功 {result['file']}")

            elif status == "skip":

                skipped += 1

                print(f"[{idx}/{total}] 已存在 {result['file']}")

            else:

                failed += 1

                failed_files.append(result)

                print(f"[{idx}/{total}] 上传失败 {result['file']}")

                print(result["error"])

    print()

    print("=" * 60)
    print("完成")
    print("=" * 60)

    print(f"总数     : {total}")

    print(f"成功     : {success}")

    print(f"跳过     : {skipped}")

    print(f"失败     : {failed}")

    if failed_files:

        print("\n失败列表:")

        for item in failed_files:
            print(item["file"])


# ============================================================
# 配置
# ============================================================


# 注意：
# 指向 carpet_detect 目录
LOCAL_DIR = (
    "/home/robot/share/AI_Program/CarpetSegmentProject/spatial_location_val_images/carpet_detect"
)

S3_BUCKET = "robot-ai-platform"

S3_PREFIX = "datasets/" "carpet_detection/" "source/images"

MAX_WORKERS = 32

MAX_RETRIES = 3

# ============================================================
# main
# ============================================================

if __name__ == "__main__":
    sync_to_s3()
