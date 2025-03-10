
# 用于检测一个为止的 pcm 文件的位深是多少
# 将需要解析的 .pcm 文件与当前文件放在同一目录下即可
# 如果报错没有 numpy 库可以手动下载： pip install numpy

import numpy as np

# 参数（根据你的文件调整）
filename = "16k_10.pcm"  # 替换为你的 PCM 文件路径
bit_depth = 16              # 假设 16-bit，可能是 8、24 或 32
signed = True               # 假设有符号，False 为无符号
channels = 1                # 假设单声道，立体声为 2
byte_order = "little"       # 假设小端字节序，"big" 为大端

# 根据位深选择数据类型
if bit_depth == 8:
    dtype = np.uint8 if not signed else np.int8
elif bit_depth == 16:
    dtype = np.uint16 if not signed else np.int16
elif bit_depth == 24:  # 24-bit 需要特殊处理
    dtype = np.int32  # 用 32-bit 读取，再截取
elif bit_depth == 32:
    dtype = np.uint32 if not signed else np.int32
else:
    raise ValueError("不支持的位深")

# 读取 PCM 文件
with open(filename, "rb") as f:
    raw_data = np.fromfile(f, dtype=dtype)

# 如果是多通道，分割数据
if channels > 1:
    raw_data = raw_data.reshape(-1, channels)

# 计算数值范围
min_value = np.min(raw_data)
max_value = np.max(raw_data)
print(f"数值范围: {min_value} 到 {max_value}")
print(f"样本数量: {len(raw_data)}")

# 可选：打印前几个样本值
print("前 10 个样本:", raw_data[:10])