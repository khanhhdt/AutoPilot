# Phase 1 — đo lại FPS và xác định điểm nghẽn

## Kết luận

**Chưa đạt tiêu chí 19–21 FPS của Phase 1 với cấu hình hiện tại.**
Đo 60 giây bằng `SensorManager` production nguyên bản, không bọc profiler callback:
735 frame / 60,085 s = **12,23 FPS**, không timeout. Không chạy perception,
PID, Traffic Manager, GUI hoặc ghi ảnh/video trong khoảng đo.

Điểm nghẽn chính nằm ở **pipeline render/capture/readback nhiều camera phía
CARLA trên cảnh Town10HD_Opt đầy đủ**, trước khi Python nhận đủ bộ ảnh đúng tick.
Các phép đo đối chứng bên dưới được củng cố bởi 10 mẫu `nvidia-smi` người dùng
cung cấp: GPU utilization trung bình 93,5%, thường ở 94–99%. Chưa có dữ liệu
VRAM đã dùng hoặc trace từng đoạn render/RHI để kết luận sâu hơn.

Ctrl+C/SIGINT đã được thử thực tế: không còn actor của dự án, world trở về async,
no-rendering được tắt. Map đã khôi phục đủ lớp Town10HD_Opt.

## Điều kiện đo

- CARLA 0.9.16, Town10HD_Opt, spawn 22, ClearNoon, một ego đứng yên.
- Sync, fixed delta = 0,05 s; đợi tất cả sensor có ID bằng đúng tick yêu cầu.
- RGB 512×288; seg/depth 192×108; collision và lane invasion vẫn được gắn.
- Mỗi vòng được giới hạn tối đa 20 Hz; nếu công việc vượt 50 ms thì FPS giảm.
- JSON/CSV ghi sau khi đo. Profiler chỉ ghi mốc callback và thời gian chạy các
  hàm decode production. Có một bài 60 s dùng production nguyên bản để loại trừ
  tác động đáng kể của wrapper profiler.
- Các phép đo tách thành phần chạy 20 s sau 20 tick warmup; các bài baseline
  60 s cold-start dùng warmup 0. Không chạy nhiều client tick đồng thời.

## Số đo

- **Baseline qua Tailscale, ba camera, 60 s:** 12,87 FPS. Trung bình tick 7,78 ms,
  chờ sensor 69,82 ms, work 77,64 ms. Decode RGB/seg/depth lần lượt khoảng
  2,85 / 0,73 / 0,51 ms; tổng khoảng 4,1 ms/tick. Depth về cuối ở 752/773 tick.
- **Cùng cấu hình qua LAN trực tiếp, 60 s:** 12,89 FPS. Chênh lệch quá nhỏ để
  giải thích việc thiếu khoảng 7 FPS bằng overhead Tailscale.
- **Không camera:** 19,95 FPS; work trung bình 7,21 ms.
- **Chỉ RGB / chỉ seg / chỉ depth:** lần lượt 19,30 / 19,58 / 19,77 FPS.
- **RGB + seg:** 15,10 FPS; work 65,94 ms.
- **Ba camera, bỏ toàn bộ decode Python:** 12,22 FPS; work 81,79 ms. Payload
  giống hệt baseline; wrapper raw chỉ cập nhật ID và timestamp.
- **Ba camera, giảm cả chiều rộng/cao còn một nửa:** payload còn 1/4,
  nhưng chỉ 12,14 FPS, không khôi phục được mục tiêu.
- **Một / hai worker streaming thay default:** 13,53 / 12,66 FPS.
- **Đảo thứ tự tạo camera thành depth → seg → RGB:** 14,56 FPS; RGB về cuối
  ở 291/292 tick. Vì vậy không phải riêng decoder depth chậm: camera
  về cuối thay đổi theo thứ tự tạo.
- **Tắt postprocess RGB:** 13,84 FPS; **no-rendering + raw:** 13,67 FPS.
  Cả hai vẫn có callback với cùng lượng byte ảnh; không đủ để đạt mục tiêu.
- **Điều kiện chờ frame kiểu cũ (`all IDs changed`):** 11,83 FPS, không có
  frame mix hoặc stale trong bài này. Đổi điều kiện đồng bộ không giải quyết FPS.

Thử nghiệm quyết định là thay riêng độ phức tạp cảnh, giữ map, vị trí, ba camera,
độ phân giải, kiểu truyền ảnh và kích thước payload:

- **Đủ lớp map:** 12,89 FPS; work trung bình 77,49 ms.
- **Cảnh nền tối giản (`MapLayer.NONE`):** 18,39 FPS; work 45,18 ms.
  Nếu nhận raw, không decode: **19,48 FPS**, work 41,26 ms.
- **Khôi phục đủ lớp map, đo lại 60 s:** 11,58 FPS; work 86,26 ms.

Mỗi tick vẫn mang **755.712 byte ảnh raw** ở cả cảnh đầy đủ và cảnh tối giản.
Chi phí tick gần như giữ nguyên: khoảng 6,2–6,7 ms; phần chờ ảnh giảm mạnh
từ 70–80 ms xuống 35–39 ms khi đơn giản hóa cảnh. Kết quả này chỉ ra tác động
chính ở quá trình tạo/capture ảnh theo độ phức tạp cảnh phía server, không phải
băng thông ảnh hay xử lý detector/PID phía client.

Bài cuối cùng không có wrapper profiler: production SensorManager đạt
**12,23 FPS**, work 81,68 ms, chờ sensor 72,61 ms. Profiler không phải nguyên nhân
làm harness từ 20 FPS xuống khoảng 12–13 FPS.

## Diễn giải và giới hạn

Ba camera là ba lượt tạo ảnh/readback, không chỉ là ba mảng numpy. Mã CARLA
0.9.16 có đường đọc ảnh với GPU copy, RHI flush, render query chờ đồng bộ và đợi
readback sẵn sàng trước khi gửi buffer:
[FPixelReader::WritePixelsToBuffer](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/PixelReader.cpp).
Đây là cơ chế phù hợp với độ trễ tăng theo số camera và phụ thuộc cảnh đã đo.
Chưa đo trực tiếp thời lượng từng đoạn RHI/GPU fence nên không gán toàn bộ
70 ms cho một dòng code hoặc kết luận GPU/driver cụ thể bị lỗi.

Tailscale hiện dùng endpoint LAN trực tiếp `192.168.80.87:41641`, không suy luận
đang relay chỉ từ trường home DERP `sin`. Wi-Fi client 5 GHz, power-save off.
Không thay đổi cấu hình mạng để làm các phép đo. Một mẫu process CPU phía
client dùng khoảng 0,16 core; bỏ decode và giảm payload cũng không khôi phục FPS.
SSH batch tới server không xác thực được; telemetry GPU sau đó do người dùng cung cấp.

### Telemetry GPU bổ sung

10 mẫu từ `nvidia-smi --query-gpu=utilization.gpu,utilization.memory,power.draw,clocks.sm`:

- GPU utilization: **trung bình 93,5%, min 74%, max 99%**; 8/10 mẫu từ 94% trở lên.
- Memory utilization: trung bình 27,1%, khoảng 22–32%. Đây là tỷ lệ thời gian
  bộ nhớ thiết bị đang được đọc/ghi, **không phải phần trăm dung lượng VRAM đã dùng**,
  theo [tài liệu NVIDIA](https://docs.nvidia.com/deploy/nvidia-smi/index.html).
- Công suất: trung bình **203,91 W**, khoảng 195,64–208,07 W.
- SM clock: cả 10 mẫu đều **2775 MHz**. Không thấy xung giảm trong mẫu này;
  chưa có nhiệt độ/throttle flags để kết luận tình trạng thermal/power throttling.

Dữ liệu này phù hợp với nút thắt render đa camera phía GPU/server đã khoanh vùng
bằng đối chứng cảnh vẽ. Đây là tổng utilization của GPU, chưa tách riêng tiến
trình CARLA; các dòng không có timestamp nên chưa ghép chính xác với từng case.
Mẫu gốc và thống kê nằm trong
[server_gpu_samples.json](../output/phase1_diagnosis/server_gpu_samples.json).

Các bài này không loại trừ mọi độ trễ mạng hoặc mọi biến động tải của server.
Kết luận có cơ sở là **chi phí camera phía server phụ thuộc cảnh là phần chính**;
chưa phân bổ sâu thành raster GPU, render-thread CPU và RHI synchronization.

Không thay cấu hình production sang cảnh tối giản/raw hoặc nới đồng bộ để
gọi là PASS. Cảnh tối giản và raw chỉ là phép đo chẩn đoán. Muốn đạt Phase 1 với
ba camera đúng tick, hướng cần tối ưu tiếp là render/capture server hoặc thiết kế
lịch sensor; phải đo lại 60 s với dữ liệu ảnh đầy đủ sau mỗi thay đổi.

## Artifact và tái lập

- [Tổng hợp JSON](../output/phase1_diagnosis/summary.json), có toàn bộ đường dẫn
  báo cáo gốc và cấu hình từng case.
- [Biểu đồ FPS](../output/phase1_diagnosis/fps_diagnosis.png).
- [Kết quả Ctrl+C](../output/phase1_diagnosis/cleanup_check.json).
- `scripts/benchmark_harness.py` là công cụ đo; production pipeline không bị sửa.

```bash
conda activate carla

# Harness thuần bằng SensorManager nguyên bản, không profiling callback.
python -m scripts.benchmark_harness --duration 60 --warmup 0 \
  --production --label production_60s

# Profile callback, đối chiếu cùng camera qua Tailscale và LAN.
python -m scripts.benchmark_harness --duration 60 --warmup 0 --label tailscale_full
python -m scripts.benchmark_harness --host 192.168.80.87 --duration 60 \
  --warmup 0 --label lan_full

# Tách camera và decode; dùng cùng map/spawn/weather cho các lần đo.
python -m scripts.benchmark_harness --host 192.168.80.87 --duration 20 \
  --sensors rgb_seg --label rgb_seg
python -m scripts.benchmark_harness --host 192.168.80.87 --duration 20 \
  --raw --label all_raw
```

Giữ một client tick tại một thời điểm. JSON/CSV mỗi lần chạy có timestamp riêng
trong `output/phase1_diagnosis/`. Ctrl+C sẽ lưu báo cáo `error=interrupted` và
khôi phục settings/dọn actor; báo cáo bị ngắt không dùng làm bài verify 60 s.
