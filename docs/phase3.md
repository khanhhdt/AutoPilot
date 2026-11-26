# Phase 3 — giữ làn bằng pure pursuit và speed PID

## Phần triển khai

- `src/control/pure_pursuit.py`: chiếu tâm của hai fit trong `LaneState` xuống
  mặt đường phẳng cục bộ, chọn điểm nhìn trước tính từ trục sau, tính góc lái
  theo pure pursuit, chuẩn hóa theo góc lái và steering curve của xe. Dương là
  phải theo CARLA. Giới hạn thay đổi steer 0,5 đơn vị/giây mô phỏng.
- `src/control/pid.py`: PID tốc độ theo km/h, dùng dt mô phỏng; derivative trên
  tốc độ đo để tránh kick khi đổi setpoint, lọc derivative và chống windup khi
  bão hòa. Throttle/brake loại trừ nhau, giới hạn mỗi loại 0,6.
- `src/main.py --driver lane`: lấy frame đúng tick → lane detector → pure pursuit
  + PID → `ego.apply()`, trước tick tiếp theo. Không bật Traffic Manager trong
  chế độ này. Mất lane target hoặc timeout sensor thì throttle 0, brake 1,
  giữ steer cuối và reset PID tốc độ. Có log, video và cleanup như chế độ live.
- `LaneState` có thêm độ phân giải ảnh và ROI để chiếu hình học đúng hệ tọa độ.
  Road mask vẫn đến từ semantic segmentation của CARLA như Phase 2.
- `src/metrics.py`: ghi steer/throttle/brake, target nhìn trước, sai số tốc độ,
  sai số làn/góc tham chiếu và kết luận nghiệm thu trong `report.json.phase3`.
  Waypoint chỉ dùng chọn spawn/đo tham chiếu, không dùng tính steer hay ga/phanh.

Phương pháp pure pursuit dựa trên
[Coulter, CMU-RI-TR-92-01](https://www.ri.cmu.edu/publication_view.html?pub_id=278).
Đơn vị trục tốc độ của SteeringCurve là km/h, theo
[mã nguồn CARLA 0.9.16](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleMovementComponentNW.h).

## Tham số đang kiểm chứng

- Tesla Model 3; RGB 512×288, seg/depth 192×108; FOV 90°; dt = 0,05 s.
- Tốc độ mục tiêu 10 km/h, lookahead 8 m tính từ trục sau.
- PID: Kp = 0,18; Ki = 0,06; Kd = 0,01.
- Wheelbase đo từ wheel physics khoảng 3,005 m; max steer khoảng 70°.
  Steering curve lấy từ physics của từng xe, không thay physics để làm bài test.
- Camera x = 1,6 m, chiều cao chiếu giả định 1,7 m; trục sau xấp xỉ ở nửa
  wheelbase sau gốc xe. Giả định mặt đường phẳng cục bộ; chưa hiệu chỉnh pitch,
  roll và cao độ bằng depth. Các tham số hình học thực dùng nằm trong report.

Town03 có mặt đường sáng khiến ngưỡng trắng cố định của Phase 2 nhận cả nền đường.
Đã thêm ngưỡng trắng theo percentile độ sáng road region, giữ ngưỡng tối thiểu
195. Kiểm tra hồi quy trên bốn session Town10 mới ghi vẫn đạt **218/246 = 88,6%,
PASS M1**, không đổi so với trước sửa.

## Tiêu chí nghiệm thu

Hai phút được tính theo **thời gian mô phỏng**: 120 s = 2.400 tick. Thời gian
thực có thể dài hơn vì client/server chưa đạt 20 FPS. Không dùng `--duration 120`
để thay cho bài 120 s mô phỏng.

- Đủ 120 s liên tục; không timeout/nhảy tick.
- Lane invasion = 0 và collision = 0, tính cả khởi hành.
- Sai số tốc độ tuyệt đối lớn nhất **<3 km/h sau 5 s ổn định**. Sai số từ lúc
  khởi hành cũng được báo riêng; lúc đứng yên ban đầu không thể đạt ngay 10 km/h.
- Mỗi bước steer thay đổi ≤0,025 ở dt = 0,05 s. Báo thêm p95 bước steer và số
  đảo chiều vượt deadband ±0,02 trong cửa sổ 2 s; từ ba lần đảo trở lên là fail.
  Đây là tiêu chí định lượng dùng trong báo cáo, không chứng minh ổn định ở mọi
  tốc độ hoặc mọi loại đường.

Các bài tune ngắn luôn có verdict FAIL ở mục đủ thời lượng, dù các điều kiện
khác đạt. Chỉ bài 120 s có thể kết luận PASS toàn bộ.

## Tune đã thực hiện

- **Town03, spawn 170, đường thẳng, 20 s mô phỏng:** 400 frame, lane invasion 0,
  collision 0, không mất làn/timeout. Sai số tốc độ lớn nhất sau ổn định
  0,206 km/h; MAE 0,057 km/h. Bước steer lớn nhất 0,000864; không có đảo chiều
  lặp nhanh. Sai số làn tham chiếu lớn nhất khoảng 0,031 m.
- **Town05, spawn 261, đoạn cua, 30 s mô phỏng:** 600 frame, lane invasion 0,
  collision 0, không mất làn/timeout. Sai số tốc độ lớn nhất sau ổn định
  0,214 km/h; MAE 0,047 km/h. Bước steer lớn nhất 0,001578; không có đảo chiều
  lặp nhanh. Sai số làn tham chiếu lớn nhất khoảng 0,059 m.
- Đã khảo sát map: đoạn Town03 được chọn dài khoảng 136 m trước giao lộ/đổi
  hướng; Town05 có hơn 450 m không giao lộ, đủ cho bài 120 s ở 10 km/h.
  Các profile spawn lưu ở `output/phase3/town03_spawns.json` và `town05_spawns.json`.
- **56 tests qua**, gồm PID, chiều lái, dt/rate limit, chuẩn hóa độ phân giải,
  mô hình bicycle lý tưởng, nối main không dùng TM, phanh khi mất làn/sensor,
  tiêu chí nghiệm thu và hồi quy Phase 2. Tests không thay bài CARLA thực tế.

## Kết quả bài xác minh 120 s trên Town05

**PASS**, với cùng tham số đã tune, spawn 261, trời quang, mục tiêu 10 km/h:

- Đủ **2.400 frame / 120 s mô phỏng**, đối chiếu thêm với timestamp camera.
  Thời gian thực 184,90 s, trung bình 12,98 FPS; không đồng nghĩa hệ đạt 20 FPS thực.
- **Lane invasion = 0, collision = 0**, không timeout hoặc nhảy tick.
- **Sai số tốc độ lớn nhất sau 5 s = 0,212 km/h**, MAE = 0,040 km/h.
  Sai số lớn nhất tính cả khởi hành = 9,848 km/h, được báo riêng.
- **Không có đảo chiều steer lặp nhanh** theo tiêu chí đã định; bước steer lớn
  nhất 0,008410, p95 = 0,000707, đều thấp hơn giới hạn 0,025/tick.
- Sai số làn tham chiếu lớn nhất khoảng **0,078 m**. Không có frame mất lane target;
  detector có 2.399 frame đo mới và 1 frame giữ fit. Tất cả 2.400 frame đều có
  lệnh điều khiển hợp lệ; 2.392 frame xe chạy ít nhất 1 km/h. Không qua giao lộ.
- Sau bài chạy, actor của dự án đã được dọn và world trở về async.

Nguồn:

- [Report gốc](../output/phase3/town05_verify/session_20261116_154829_766259/report.json)
  và [chuỗi điều khiển từng tick](../output/phase3/town05_verify/session_20261116_154829_766259/frames.csv).
- [Video giữ làn đủ 120 s](../output/phase3/phase3_town05_120s.mp4).
- [Biểu đồ tốc độ, steer và sai số làn](../output/phase3/control_trace.png).
- [Tổng hợp xác minh và hash mã nguồn](../output/phase3/verification_summary.json).

Town03 là bài tune thẳng 20 s; Town05 là bài xác minh liên tục đủ 120 s. Chưa có
bài 120 s trên Town03 hoặc kiểm chứng nhiều lần/mưa/ban đêm.

## Chạy lại

`--map` nạp map và reset world. Mỗi lần chạy tạo session riêng dưới `--out`;
chương trình khôi phục settings/thời tiết của map đang chạy và dọn actor khi dừng.

```bash
conda activate carla

# Tune thẳng: Town03, tránh đi quá đoạn thẳng 136 m khi chưa có planning giao lộ.
python -m src.main --driver lane --map Town03 --spawn-index 170 \
  --sim-duration 20 --target-speed 10 --lookahead 8 --weather clear \
  --no-display --save-video --out output/phase3/town03_tune

# Tune cua trước khi chạy bài dài.
python -m src.main --driver lane --map Town05 --spawn-index 261 \
  --sim-duration 30 --target-speed 10 --lookahead 8 --weather clear \
  --no-display --save-video --out output/phase3/town05_tune

# Xác minh đủ hai phút mô phỏng, dùng cùng tham số đã tune.
python -m src.main --driver lane --map Town05 --spawn-index 261 \
  --sim-duration 120 --target-speed 10 --lookahead 8 --weather clear \
  --no-display --save-video --out output/phase3/town05_verify

python -m unittest discover -s tests -v
```

`report.json` ghi các check và cực trị; `frames.csv` chứa toàn bộ chuỗi điều khiển
để xem dao động. `live.mp4` được ghi ngay trong vòng lặp điều khiển, với RGB cạnh
overlay và steer/throttle/brake. Pixel vàng trên overlay là điểm pursuit.
Video phát ở 20 FPS theo thời gian mô phỏng; tốc độ thực ở `avg_fps_wall`.

Phạm vi kiểm tra hiện tại: một xe, trời quang, các đoạn đã chọn. Chưa bao gồm
planning giao lộ, tránh vật cản hoặc kiểm chứng tổng quát ở tốc độ cao/mưa/ban đêm.
