Import("env")
import os
import subprocess
import sys
from os.path import join

def merge_bin(source, target, env):
    # 获取构建目录和项目名
    build_dir = env.subst("$BUILD_DIR")
    progname = env.subst("$PROGNAME")
    chip = env.get("BOARD_MCU")
    flash_size = env.BoardConfig().get("upload.flash_size", "8MB")
    
    # 获取 esptool 路径
    platform = env.PioPlatform()
    esptool_path = platform.get_package_dir("tool-esptoolpy")
    esptool_py = join(esptool_path, "esptool.py")
    
    # 定义源文件和地址 (请根据你的分区表确认 bootloader 地址)
    bootloader_bin = join(build_dir, "bootloader.bin")
    partitions_bin = join(build_dir, "partitions.bin")
    firmware_bin = join(build_dir, "firmware.bin")
    boot_app0_bin = join(env.PioPlatform().get_package_dir("framework-arduinoespressif32"), "tools", "partitions", "boot_app0.bin")
    
    output_bin = join(build_dir, f"{progname}.combineBin.bin")
    
    # 构建 merge_bin 命令 (ESP32-S3 的 bootloader 地址通常为 0x0)
    cmd = [
        sys.executable, esptool_py,
        "--chip", chip,
        "merge_bin",
        "-o", output_bin,
        "--flash_size", flash_size,
        "0x0", bootloader_bin,
        "0x8000", partitions_bin,
        "0xe000", boot_app0_bin,
        "0x10000", firmware_bin
    ]
    
    print(f"正在合并固件到: {output_bin}")
    subprocess.run(cmd, check=True)
    print(f"合并完成: {output_bin}")

# 在 firmware.bin 生成后执行合并操作
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_bin)