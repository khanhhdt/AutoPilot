## Kiểm chứng PoC

Các bài chạy sử dụng CARLA 0.9.16 qua `localhost:3000`, đồng bộ ở 20 Hz.

### Tốc độ trên đường trống — PASS

Nghiệm thu chạy `clear-road` trên một đoạn thẳng ở Town10HD_Opt với mục tiêu
**50 km/h**. Ca chỉ đạt khi tốc độ đo được giữ trong dải **50 ± 1,5 km/h**
liên tục ít nhất **1 giây**, trong trạng thái `CRUISE`, còn giữ làn bằng camera
và không phát hiện vật cản phía trước. Giới hạn tốc độ vẫn tính theo tầm nhìn
và khoảng cách có thể phanh trước cua.

Lượt kiểm tra ngày 27/11/2026 đạt **49,46 km/h** và duy trì trong dải yêu cầu
**1,45 giây**, với **0 va chạm**, **0 chạm vạch** và **0 timeout cảm biến**.
[Báo cáo ca đường trống](../output/poc_50kmh/verify/run_20261127_135052_456789/clear_road_50/run_20261127_135053_333556/summary.json)

Bộ nghiệm thu mới đạt **16/16 bước**. Ca đường trống trong lượt ghi Full HD
cũng giữ được **1,45 giây** trong dải yêu cầu. [Báo cáo tổng](../output/poc_50kmh/verify/run_20261127_135052_456789/summary.json) · [Demo 60 giây](../output/poc_50kmh/verify/run_20261127_135052_456789/demo_hd_60s.mp4)

```bash
python -m scripts.run_suite --host localhost --port 3000 --case clear-road
```

### Kết quả nghiệm thu trước đó

Các ca vật cản và đường cong trên Town10HD_Opt dùng tốc độ mục tiêu 10 km/h.
Ca TTC mở rộng trên Town05 dùng mục tiêu 30 km/h.

### 1. Kiểm tra điểm mù bằng LiDAR — PASS

Chế độ `avoid` dùng LiDAR ray-cast 360° không semantic, khớp đúng ID tick với
camera. Planner kiểm tra đám mây điểm trong hành lang làn đích trước và trong
khi chuyển làn. Dữ liệu actor được dùng ở khâu dựng và chấm kịch bản. Khi thiếu
scan hợp lệ, bộ kiểm tra không xác nhận làn trống. [Cảm biến CARLA 0.9.16](https://carla.readthedocs.io/en/0.9.16/ref_sensors/#lidar-sensor)

| Ca | Kết quả |
|---|---|
| Làn phải trống | Vượt xe đỗ sang phải và hoàn thành thao tác. [Báo cáo](../output/improvements/lidar_verify/run_20261126_164336_612566/summary.json) |
| Làn trái trống | Vượt xe đỗ sang trái và hoàn thành thao tác. [Báo cáo](../output/improvements/lidar_verify/run_20261126_164347_612003/summary.json) |
| Xe áp sát từ sau | LiDAR chặn làn phải trong 60 tick khi xe còn ở phía sau. Ego không chuyển làn trong vùng đang bị chiếm. [Báo cáo](../output/improvements/lidar_verify/run_20261126_164358_632147/summary.json) |
| Làn kề bị chặn | Ego giữ làn và dừng trước rào chắn, khoảng cách nhỏ nhất 5,95 m. [Báo cáo](../output/improvements/lidar_verify/run_20261126_164412_142163/summary.json) |

Tổng cộng **815/815 tick khớp LiDAR và camera**, 0 va chạm, 0 timeout và 0 thao
tác chuyển làn khi làn bị chặn. Unit test kiểm tra cả scan thiếu và scan cũ.

### 2. Nhận diện làn thuần RGB — PASS

`--lane-input rgb` dùng ảnh RGB và bổ sung bước tách vạch theo tương phản cục bộ
khi ngưỡng màu không tìm được cặp vạch. Nhánh này dùng nhãn CARLA để chấm offline.
Hai chế độ được chấm trên cùng 600 ảnh ghi mới sau khi chốt detector. M1-v2 yêu
cầu sai số dưới 15 px ở độ rộng chuẩn 512 px trên ít nhất 80% khung có ground
truth của từng phiên, tính cả các khung bỏ lỡ.

| Đoạn đường | Thời tiết | RGB có road mask | RGB thuần |
|---|---|---:|---:|
| Thẳng | Quang | 58/66 (87,9%) | 57/66 (86,4%) |
| Thẳng | Mưa | 15/66 (22,7%) | 62/66 (93,9%) |
| Cua | Quang | 55/57 (96,5%) | 55/57 (96,5%) |
| Cua | Mưa | 56/57 (98,2%) | 55/57 (96,5%) |
| Tổng | | 184/246 (74,8%) | 229/246 (93,1%) |

RGB thuần đạt **4/4 phiên**. Kết quả áp dụng cho nhận diện làn. Chế độ `avoid`
vẫn dùng semantic/depth cho vật cản phía trước và vùng mặt đường quan sát được.
[Kết quả RGB](../output/improvements/lane_verify_rgb.json) · [Đối chứng có mask](../output/improvements/lane_verify_semantic.json)

### 3. Tracker và TTC — PASS

Tracker Kalman 2D ghép các quan sát theo vị trí dọc và ngang dự đoán, giữ ID qua
những khoảng mất quan sát ngắn. Log `closing_mps` và `ttc_s` dùng cùng ước tính
của tracker. Ngưỡng phanh khẩn cấp vẫn dùng khoảng cách đo trực tiếp.

- Trên **289 mẫu chung** với TTC tham chiếu dưới 8 giây, sai số tuyệt đối p95
  giảm từ **0,81 s** với cách tính 5 tick xuống **0,10 s**. Tập mẫu này chủ yếu
  gồm các lần tiến gần vật cản tĩnh, nên kết quả định lượng áp dụng cho phạm vi
  đó. Unit test kiểm tra giữ ID khi hai xe đổi thứ tự khoảng cách. [Đối chứng TTC](../output/improvements/ttc_verification.json)
- Trong ca xe cắt vào trên Town05 ở mục tiêu **30 km/h**, ego phản ứng và giữ
  khoảng cách dọc tối thiểu **20,56 m** sau khi xe nhập làn. Ca này xác nhận
  hành vi phản ứng với xe đang di chuyển. [Báo cáo Town05](../output/improvements/adaptive_verify/cut_in/verification.json)

### 4. Ca kiểm thử mưa — PASS

Suite nhận `--weather rain` để chạy `HardRainNoon`. Mỗi ca mưa có lượt đối
chứng trời quang với cùng cấu hình. Ca xe cắt vào dùng chuyển động thử nghiệm
có kiểm soát để tái lập lúc xe nhập làn giữa các lượt chạy.

| Ca | Trời quang | Trời mưa |
|---|---|---|
| Giữ làn trên cua bằng RGB | 19,71 m, nhận diện 160/160 khung. [Báo cáo](../output/improvements/weather_verify/clear/run_20261126_164419_278889/summary.json) | 19,70 m, nhận diện 160/160 khung. [Báo cáo](../output/improvements/weather_verify/rain/run_20261126_164445_167896/summary.json) |
| Xe cắt vào, có road mask | Ego giảm tốc và bám theo, khoảng cách dọc nhỏ nhất 7,18 m. [Báo cáo](../output/improvements/weather_final/clear/run_20261127_090534_316642/summary.json) | Ego giảm tốc và bám theo, khoảng cách dọc nhỏ nhất 7,19 m. [Báo cáo](../output/improvements/weather_final/rain/run_20261127_090547_839692/summary.json) |

Cả **4/4 lượt** đạt, với 0 va chạm, 0 lần chạm vạch và 0 timeout cảm biến.

### Hồi quy và tái lập

Phase 4 đạt **20/20 ca**, gồm **10/10 lần dừng trước rào chắn**, 0 va chạm và
0 lần bắt đầu chuyển làn khi làn đích bị chặn. Toàn bộ **19 unit test** trong
workspace đã đạt. [Báo cáo Phase 4](../output/improvements/phase4_final/run_20261127_090225_276785/summary.json)

```bash
python -m scripts.eval_lane --session output/improvements/lane_verify_data/session_* --lane-input rgb
python -m scripts.eval_lane --session output/improvements/lane_verify_data/session_* --lane-input semantic
python -m scripts.run_suite --host localhost --port 3000 --case curve-road --lane-input rgb --weather rain
python -m scripts.run_suite --host localhost --port 3000 --case cut-in --weather rain
python -m scripts.run_suite --host localhost --port 3000 --case avoid-rear
python -m scripts.scenarios --host localhost --port 3000 --suite
python -m unittest discover -s tests -v
```
