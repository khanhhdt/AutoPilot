# Autonomous Driving (CARLA) — Lane Detection + Object Avoidance

Mô phỏng xe tự hành trên CARLA: lane detection (OpenCV + semantic segmentation),
quyết định real-time và tránh vật cản.

## Yêu cầu

- CARLA server 0.9.16 chạy trên máy GPU (RPC `:3000`, streaming `:3001`)
- Python 3.11 + `carla` 0.9.16, OpenCV, numpy
- Mạng giữa client và server ≥ 20 MB/s (đo bằng `iperf3`), WiFi nên tắt power save

## Cài đặt

```bash
conda activate carla
pip install -r requirements.txt
# carla: cài từ wheel của server
pip install carla-0.9.16-cp311-cp311-manylinux_2_31_x86_64.whl
```

## Chạy

```bash
# đổi host nếu cần (mặc định 100.85.59.63)
python -m src.main --duration 60            # ba ô RGB + semantic + lane overlay
python -m src.main --duration 60 --no-display

CARLA_HOST=192.168.80.87 python -m src.main --duration 60

# Nhận diện làn live khi Traffic Manager lái xe; đo tốc độ không ghi video.
python -m src.main --duration 60 --driver tm --target-speed 10 --spawn-index 22 --no-display

# Video ba ô RGB + semantic + lane overlay được tạo ngay trong vòng lặp live.
python -m src.main --frames 150 --driver tm --spawn-index 62 --weather clear \
  --no-display --save-video --out output/live_demo
```

Phím `q`/`ESC` để thoát. Chương trình khôi phục world settings/thời tiết và hủy actor.

`src.main` chạy detector trên bộ RGB/seg/depth đúng tick trước khi gọi tick tiếp
theo. Mặc định `--driver none` giữ xe đứng yên; `--driver tm` dùng Traffic Manager.
`--driver lane` dùng LaneState → pure pursuit → steer và speed PID → throttle/brake.
Cửa sổ và video hiển thị ba ô: RGB phía trước → semantic toàn cảnh → overlay giữ làn.
Ô semantic dùng ảnh nhãn màu do CARLA cung cấp từ sensor hiện có, phóng theo
nearest-neighbor để giữ nguyên màu nhãn; không thêm camera render.
Mỗi lần chạy tạo thư mục session mới dưới `--out` (mặc định `output/live`), gồm
`frames.csv`, `report.json` và `live.mp4` nếu có `--save-video`. Báo cáo tách thời
gian tick server, chờ sensor, dựng road mask, detector, hiển thị/ghi video,
`work_ms` trước sleep và `loop_ms` có sleep. Ghi log CSV/JSON sau khi dừng; không
ghi PNG/NPY từng frame trong vòng lặp này. Video dùng FPS mô phỏng, còn tốc độ
thực tế lấy từ `avg_fps_wall`. Lane loss/coverage là trạng thái detector, chưa
phải accuracy so với GT; báo riêng loss khi xe chạy ít nhất 1 km/h và ngoài giao
lộ. Fit cũ bị reset khi timeout sensor hoặc bỏ qua world tick. Xem
[kết quả kiểm chứng live](docs/phase2.md#nhận-diện-live-trong-vòng-lặp-sensor).

## Phase 2 — Dataset + lane detection

```bash
# tiền kiểm tra các spawn có route phù hợp trên map hiện tại
python -m scripts.record_dataset --route curve --list-spawns
python -m scripts.record_dataset --route intersection --list-spawns

# Town10HD_Opt: spawn 22 cho đoạn thẳng; spawn 62 cho đường cong ngoài giao lộ;
# spawn 2 cho route qua giao lộ (rẽ khoảng 50°).
# Mỗi session chỉ được tính vào benchmark nếu đủ frame, không va chạm và GT hợp lệ.
set -e
for route_spawn in "straight 22" "curve 62" "intersection 2"; do
  set -- $route_spawn
  route=$1
  spawn=$2
  for weather in clear rain; do
    python -m scripts.record_dataset --frames 150 --driver tm --target-speed 10 \
      --plan-length 400 --spawn-index "$spawn" --route "$route" \
      --weather "$weather" --out data/raw/m1
  done
done

# Chỉ session hoàn chỉnh được dùng cho verdict M1; incomplete bị bỏ qua mặc định.
# MP4 và ảnh overlay được lưu trong từng session/eval/.
python -m scripts.eval_lane --session data/raw/m1/session_* --save-video --save-overlay 20 \
  --report output/phase2/report.json --audit-dir output/phase2/audit
```

- Cấu hình chuẩn ghi mới: `rgb/` PNG 512×288, `seg/` tag PNG 192×108 và `depth/` NPY mét;
  mỗi `meta.json` lưu độ phân giải thực tế. Session cũ vẫn giữ nguyên độ phân giải lúc ghi.
  Lần ghi mới ngày 16/11/2026 đạt khoảng 11–15 FPS thực tế; bước thời gian mô phỏng
  vẫn là 20 Hz. Xem video và kết quả online trong [ghi chú Phase 2](docs/phase2.md).
- RGB/seg/depth phải có cùng frame ID với giá trị trả về từ `world.tick()`.
  Metadata ghi `sensor_sync=exact_tick_frame_id`, số timeout và p95 thời gian
  tick/chờ sensor, ghi dữ liệu. Dữ liệu cũ được báo `legacy_unverified`.
- Tag đã xác minh trên server: `Road = 1`, `Sidewalk = 2`, `RoadLines = 24`
- Lane detection: ROI → Gaussian blur/Canny hỗ trợ trích màu trắng-vàng → lọc road-region
  (seg) → IPM → sliding window (ưu tiên base gần đáy, bỏ candidate sát biên band) →
  chiếu point về camera → polyfit; hybrid road-edge/color chọn cặp theo width/center.
  Loại cặp vạch giao nhau hoặc có độ rộng bất hợp lý. Trả `LaneState.offset_px`
  (dương = tâm làn lệch phải), `fresh` và số frame giữ fit cũ `missed_frames`.
- `lane_offset_m` là độ lệch tới waypoint khoảng 3 m phía trước (chiếu tại đáy camera);
  `lane_gt_valid` đánh dấu GT hợp lệ.
- Recorder dừng sau va chạm hoặc 60 frame liên tiếp xe chạy dưới 1 km/h; phần đuôi đứng yên bị loại khỏi dataset. Session bị dừng sớm được gắn `complete=false` và evaluator loại khỏi M1.
- Eval báo cả **M1 legacy** (<15 px ảnh gốc) và **M1-v2**
  (`error_ref_px = error_px × 512 / image_width`, ngưỡng <15 px ở ảnh chuẩn 512,
  cùng FOV 90°). Mỗi session cần ít nhất 50 frame GT hợp lệ và ít nhất 80% frame
  đạt ngưỡng; detector bỏ lỡ tính là fail. Verdict tổng chỉ xét session đủ chuẩn:
  session giao lộ, incomplete, thiếu dữ liệu, thiếu nhãn giao lộ hoặc chỉ chạy một
  phần bằng `--start`/`--limit` đều là diagnostic. Aggregate toàn bộ session được
  báo riêng và không ảnh hưởng verdict của nhóm đủ chuẩn.
- `--audit-dir` xuất CSV từng frame, lý do không trích được GT, kết quả đo mới/giữ
  từ frame trước và ảnh lỗi lấy mẫu. Fit từ connected components của RoadLines
  chỉ dùng đối chiếu GT (màu tím), không thay nhãn benchmark. GT fit thất bại chưa
  đồng nghĩa thực tế không nhìn thấy vạch. Báo riêng detector coverage,
  accuracy-on-detected và p50/p95 thời gian detector (gồm overlay, chưa gồm I/O).
  Evaluator dùng một OpenCV worker mặc định; đổi bằng `--opencv-threads`.
- Map GT waypoint 3 m chỉ là diagnostic/control;
  trên curve nó lệch khỏi lane center ở đáy camera ~20 px nên không dùng làm verdict.
  Route giao lộ = `N/A`.

Kiểm tra offline, không cần cài CARLA:

```bash
python -m unittest discover -s tests -v
python -m scripts.eval_lane --session data/raw/m1/session_* data/raw/m1_1024/session_* \
  --report output/phase2/report.json --audit-dir output/phase2/audit
```

Giữ `smooth=0.85`, `max_missed=5`: thử giảm smoothing cải thiện một số đoạn cong
nhưng làm giảm kết quả trên đường thẳng trời mưa. Xem [ghi chú Phase 2](docs/phase2.md)
về benchmark, audit và cách ghi bộ dữ liệu kiểm tra tiếp theo.

## Kiểm tra Phase 1

- Mục tiêu: 60 s headless đạt 19–21 FPS, không timeout/rò actor, khôi phục async.
- Đo lại ngày 16/11/2026 bằng SensorManager nguyên bản, không detector/control/
  GUI/video: **12,23 FPS trong 60 s trên Town10HD_Opt đầy đủ — chưa đạt FPS**.
- Không timeout; đã thử Ctrl+C, không còn actor dự án và server trở về async.
- Các phép đo đối chứng xác định phần chậm chính ở render/capture/readback nhiều
  camera phía server, phụ thuộc độ phức tạp cảnh. Tailscale và LAN đều khoảng
  12–13 FPS; bỏ decode hoặc giảm payload còn 1/4 không khôi phục 20 FPS.
- Ghi nhận 19–21 FPS trước đây chưa tái lập được trong cấu hình hiện tại.
  Xem [báo cáo đo lại Phase 1](docs/phase1_performance.md) và
  `python -m scripts.benchmark_harness --help` để tái lập.

## Cấu trúc

```
configs/config.py       host/port, sync mode, resolution
src/main.py             vòng lặp sensor + lane detector live, đo ngân sách 50 ms/tick
src/sensors.py          RGB + semantic seg + depth + collision/lane invasion
src/vehicle.py          spawn ego, apply_control
src/perception/         lane_detection.py (Phase 2), segmentation, obstacles
scripts/                record_dataset.py, eval_lane.py
src/metrics.py          thống kê live + CSV/JSON
src/planning/           FSM (Phase 4)
src/control/            pure pursuit + PID (Phase 3)
third_party/agents/     CARLA 0.9.16 PythonAPI agents (MIT): LocalPlanner + PID
docs/                   sơ đồ kiến trúc
```

`--driver local` dùng `LocalPlanner`/PID vendored từ CARLA PythonAPI (không cần
cài `networkx`/`shapely`). `--plan-length` mặc định 400 m (~1400 frame ở 20 km/h),
tăng nếu ghi dài hơn. `--driver tm` (mặc định) vẫn dùng Traffic Manager.

## Phase 3 — Giữ làn

```bash
# Tune thẳng trên Town03, rồi cua trên Town05. --map nạp lại world khi đổi map.
python -m src.main --driver lane --map Town03 --spawn-index 170 \
  --sim-duration 20 --target-speed 10 --weather clear --no-display --save-video \
  --out output/phase3/town03_tune

# Bài nghiệm thu liên tục 120 giây mô phỏng = 2400 tick.
python -m src.main --driver lane --map Town05 --spawn-index 261 \
  --sim-duration 120 --target-speed 10 --lookahead 8 --weather clear \
  --no-display --save-video --out output/phase3/town05_verify
```

Pure pursuit/PID dùng dt mô phỏng 0,05 s; không dùng Traffic Manager trong chế độ
`lane`. Mất lane target hoặc timeout sensor thì phanh và reset PID. Báo cáo có
steer/throttle/brake, target pursuit, sai số làn tham chiếu và các check trong
`report.json.phase3`: đủ thời gian, lane invasion/collision bằng 0, sai số tốc độ
<3 km/h sau 5 s ổn định, giới hạn bước steer và đảo chiều lặp nhanh. Sai số khi
khởi hành vẫn được báo riêng. Bài tune ngắn chưa đủ thời lượng sẽ không PASS
toàn bộ. Xem [thiết kế, tham số và kết quả Phase 3](docs/phase3.md).
