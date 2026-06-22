import cv2
from ultralytics import YOLO

def run_yolo_inference(model_path, source_path):
    # 1. Khởi tạo model (Thêm task="segment" để hỗ trợ NCNN và .pt)
    model = YOLO(model_path, task="segment")
    
    # Kiểm tra xem nguồn là ảnh hay video/webcam dựa vào phần mở rộng
    is_image = str(source_path).lower().endswith(('.png', '.jpg', '.jpeg', '.webp'))

    if is_image:
        # --- XỬ LÝ ẢNH TĨNH ---
        frame = cv2.imread(source_path)
        if frame is None:
            print("Không thể đọc được ảnh. Vui lòng kiểm tra lại đường dẫn.")
            return

        # Chạy inference
        results = model(frame)
        
        # Vẽ kết quả lên ảnh (lưu vào biến annotated_frame, không lưu ra file)
        annotated_frame = results[0].plot()
        
        # Hiển thị
        cv2.imshow("YOLO Inference - Nhan 'q' de thoat", annotated_frame)
        cv2.waitKey(0) # Đợi người dùng nhấn phím bất kỳ
        cv2.destroyAllWindows()

    else:
        # --- XỬ LÝ VIDEO HOẶC WEBCAM ---
        # Nếu source_path là 0, nó sẽ mở webcam mặc định
        cap = cv2.VideoCapture(source_path)
        
        if not cap.isOpened():
            print("Không thể mở video hoặc webcam.")
            return

        while True:
            success, frame = cap.read()
            if not success:
                break # Hết video hoặc mất kết nối webcam
                
            # Chạy inference trên từng frame
            results = model(frame)
            
            # Vẽ bounding box
            annotated_frame = results[0].plot()
            
            # Hiển thị frame
            cv2.imshow("YOLO Inference - Nhan 'q' de thoat", annotated_frame)
            
            # Nhấn 'q' để dừng quá trình hiển thị
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
                
        # Dọn dẹp bộ nhớ
        cap.release()
        cv2.destroyAllWindows()

# ==========================================
# CÁCH SỬ DỤNG
# ==========================================
if __name__ == "__main__":
    # Thay bằng đường dẫn tới model weights (.pt hoặc thư mục NCNN model)
    MY_MODEL = "models/yolo26-best_ncnn_model" 
    
    # Chọn nguồn dữ liệu:
    # - Webcam: Để là số 0
    # - Video: "test/test_video/video_test1.mp4" hoặc đường dẫn khác
    # - Ảnh: "duong_dan_toi_anh.jpg"
    MY_SOURCE = "/home/goln/dat_git/SimpleRobot-ROS2/demo_data/2026-06-11_05-57-19/videos/simplerobot_drive_20260611_053854.mp4.mp4" 
    
    run_yolo_inference(MY_MODEL, MY_SOURCE)