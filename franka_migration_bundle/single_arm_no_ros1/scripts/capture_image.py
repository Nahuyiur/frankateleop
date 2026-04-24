import os
import sys
import time
import cv2

# 确保能找到你的自定义模块
sys.path.append("/home/ubuntu/FR3/")
from envs.camera.rs_capture import RSCapture
from fr3_single_arm_config import SingleArmConfig

def create_cameras():
    cameras = {}
    for name, kwargs in SingleArmConfig.REALSENSE_CAMERAS.items():
        cameras[name] = RSCapture(name=name, **kwargs)
    return cameras

def close_cameras(cameras):
    for camera in cameras.values():
        camera.close()

def main():
    # 设置图片保存路径
    save_dir = "/home/ubuntu/FR3/saved_images"
    os.makedirs(save_dir, exist_ok=True)

    print("正在初始化相机，请稍候...")
    cameras = create_cameras()
    if not cameras:
        print("未找到相机配置！请检查 SingleArmConfig。")
        return

    # 获取所有相机的名称
    camera_names = list(cameras.keys())
    
    # 取列表里“中间”的那个相机。如果配置了3个(比如 left, front, right)，它会选 front
    # 如果只有一个相机，它就会选那唯一的一个
    target_cam_name = camera_names[len(camera_names) // 2]
    
    print(f"--> 将要显示并抓拍的相机画面: [{target_cam_name}]")
    print("--> [提示] 鼠标点击弹出的 [Preview] 窗口后，按键：\n    [空格键] = 截图保存\n    [q] = 退出")

    try:
        while True:
            # 读取所有相机的帧 (保持底层 buffer 清空，防止延迟)
            frames = {}
            for name, camera in cameras.items():
                rgb, _ = camera.read()
                frames[name] = rgb

            # 获取目标相机的当前帧
            current_frame = frames[target_cam_name]
            
            # 显示画面
            cv2.imshow("Preview", current_frame)

            # 等待按键输入 (1ms 延迟)
            key = cv2.waitKey(1) & 0xFF

            # 按下 'q' 退出 (ASCII 113)
            if key in (ord('q'), ord('Q')):
                break
            
            # 按下 [空格键] (ASCII 32) 截图
            if key == 32:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(save_dir, f"{target_cam_name}_{timestamp}.png")
                
                # ==== 如果你说的"截取中间"是指物理裁剪画面的正中心(比如裁个 400x400 的图) ====
                # 可以把下面这三行代码取消注释：
                # h, w = current_frame.shape[:2]
                # crop_size = 400
                # current_frame = current_frame[h//2 - crop_size//2 : h//2 + crop_size//2, 
                #                               w//2 - crop_size//2 : w//2 + crop_size//2]
                # =====================================================================
                
                # 保存图片 (注意：cv2.imwrite 默认接受 BGR 格式，realsense 读取的刚好也是 BGR)
                cv2.imwrite(filename, current_frame)
                print(f"[{timestamp}] 成功截取图片并保存至: {filename}")

    finally:
        # 安全退出，释放资源
        cv2.destroyAllWindows()
        close_cameras(cameras)
        print("相机已关闭。")

if __name__ == "__main__":
    main()