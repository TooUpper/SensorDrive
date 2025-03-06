
# 这是一个python编写的TCP测试工具
# 模拟了一个服务端的环境用于测试客户端是否可以正常连接与数据收发
# 修改其中的 IP 和 端口 即可使用

import socket
import threading
import queue
import sys
import select  # 添加这一行

# 用于存储用户输入的队列
input_queue = queue.Queue()

# 线程函数：读取用户输入并放入队列
def input_thread():
    while True:
        try:
            user_input = input("> ")
            input_queue.put(user_input)
        except KeyboardInterrupt:
            input_queue.put(None)  # Ctrl+C 时退出
            break
        except Exception as e:
            print(f"Input error: {e}")
            break

# 主程序
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("192.168.106.181", 8080))  # 确保 IP 正确，或使用 "0.0.0.0"
server.listen(1)
print("Listening on 192.168.106.181:8080...")

conn, addr = server.accept()
print(f"Connected by {addr}")

# 启动输入线程
thread = threading.Thread(target=input_thread, daemon=True)
thread.start()

print("Enter messages to send to MCU (close window or Ctrl+C to exit):")
while True:
    try:
        # 检查 socket 是否有数据
        readable, _, _ = select.select([conn], [], [], 0.1)
        if conn in readable:
            data = conn.recv(1024)
            if not data:
                print("Connection closed by MCU")
                break
            print(f"Received: {data.decode()}")

        # 检查队列是否有用户输入
        try:
            user_input = input_queue.get_nowait()
            if user_input is None:  # Ctrl+C 退出
                print("Server stopped by user")
                break
            if user_input:
                conn.send(user_input.encode())
                print(f"Sent: {user_input}")
        except queue.Empty:
            pass  # 队列为空，继续循环

    except Exception as e:
        print(f"Error: {e}")
        break

conn.close()
server.close()
print("Server closed")