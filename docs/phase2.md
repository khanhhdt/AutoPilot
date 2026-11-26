# Phase 2 — benchmark và kiểm tra dữ liệu

## Thay đổi đã triển khai

- `src/sensors.py`: chỉ ghép RGB/seg/depth có frame ID bằng đúng tick yêu cầu;
  trả timeout nếu không có bộ khớp, không lấy frame mới hơn để thay thế.
- `src/main.py`, `scripts/record_dataset.py`: truyền ID trả về từ `world.tick()`.
  Recorder ghi số timeout, p95 tick/chờ sensor, p95 ghi PNG/NPY và provenance đồng bộ.
- `configs/config.py`: thống nhất RGB 512×288, seg/depth 192×108. Dữ liệu cũ không
  bị resize hay ghi đè. Đã kiểm tra ghi online ngày 16/11/2026, đạt khoảng 11–15 FPS
  thực tế; kết quả chi tiết ở phần lần chạy online bên dưới.
- `src/perception/lane_detection.py`: kiểm tra cặp biên không giao nhau trong
  vùng gần xe, độ rộng đáy nằm trong 20–80% chiều rộng ảnh. Báo `fresh` để phân
  biệt đo mới với giữ fit cũ; hết `max_missed` frame thì bỏ fit. Giữ smoothing 0,85.
- `scripts/eval_lane.py`: báo metric pixel gốc và metric chuẩn hóa, xuất JSON,
  CSV/ảnh audit, lý do loại session/frame, thời gian detector và số frame đo mới/giữ.
- `scripts/lane_ground_truth.py`: fit riêng từng connected component của
  RoadLines để đối chiếu với GT sliding window. Kết quả này chỉ phục vụ audit,
  chưa đủ tin cậy để thay nhãn benchmark.

## Quy tắc đánh giá

Giữ cùng FOV 90° và tỷ lệ ảnh 16:9:

- **M1 legacy**: sai số tâm làn <15 px trên ảnh gốc.
- **M1-v2**: `error_ref_px = error_px × 512 / image_width`, ngưỡng <15 px chuẩn.
  Ví dụ 20 px ở ảnh rộng 1024 tương đương 10 px ở ảnh rộng 512.
- Một session cần ít nhất 50 frame có cả hai fit GT ngoài giao lộ. Điểm là số
  frame đạt ngưỡng chia cho toàn bộ frame GT hợp lệ, tính detector bỏ lỡ là fail.
  Mỗi session đủ chuẩn phải đạt ít nhất 80%; session lớn không bù được session fail.
- Chỉ session đầy đủ, không aborted, đủ ảnh/nhãn giao lộ và đánh giá toàn phiên
  mới có verdict. Route giao lộ chỉ là diagnostic, kể cả đoạn trước giao lộ.
  `--include-incomplete` cho phép xem dữ liệu lỗi nhưng không cho verdict M1.
- Điểm tổng hợp để kết luận chỉ cộng các session đủ chuẩn. Điểm toàn bộ session
  và các metric map GT vẫn được báo riêng để chẩn đoán.

`gt_fit_missing_*` nghĩa là thuật toán trích GT không tạo được fit, chưa khẳng
định vạch không xuất hiện trong ảnh. Road mask đầu vào detector là nhãn từ
simulator; kết quả này chưa đại diện cho hệ chỉ dùng RGB.

## Kết quả offline trên dữ liệu hiện có

Đối chiếu cùng evaluator và cùng một OpenCV worker, trên 12 session hoàn chỉnh,
1.800 frame, từ dữ liệu ngày 09/11/2026:

- Bốn session đủ chuẩn: 246 frame GT, 217 frame đạt ngưỡng; **88,2%, PASS** cho
  cả legacy và v2. Từng session đạt 89,4%, 86,0%, 86,4% và 91,2%.
- Tám session còn lại là diagnostic: ba session giao lộ, năm session độ phân
  giải 1024 không đủ 50 frame GT. Bốn session aborted được bỏ qua mặc định.
- Toàn bộ 12 session: 534 frame GT; detector coverage 99,1%; điểm pixel gốc
  77,3%, điểm chuẩn hóa 86,9%. Đây là aggregate diagnostic, không phải verdict.
- Các điểm trên giống baseline trước thay đổi detector. PASS thay cho FAIL
  trước đó là do sửa cách gộp session, không phải tăng accuracy của detector.
- P95 detector theo session khoảng 2,8–11,2 ms trên máy chạy kiểm tra, gồm tạo
  overlay nhưng chưa gồm đọc ảnh, dựng road mask và I/O. Chưa chứng minh 20 FPS
  của toàn pipeline online.
- Hai cách trích GT bất đồng ít nhất 10 px chuẩn ở 189/519 frame so sánh được.
  Đây là cờ để review, không phải 189 nhãn đã được xác nhận sai. Một số ảnh audit
  cho thấy vạch đứt bị bỏ sót hoặc fit sliding window trộn vạch làn kề.

Artifact trong `output/phase2/` (thư mục này được Git bỏ qua):

- `baseline.json`, `final.json`: metric, cấu hình, hash mã detector và provenance.
- `final_audit_verified/<session>/frames.csv`: lỗi từng frame, GT loại vì lý do
  nào, tâm từ hai cách fit, đo mới/giữ và latency.
- `final_audit_verified/<session>/*.png`: tối đa ba ảnh đầu mỗi loại lỗi.
  Pixel đỏ = RoadLines GT, xanh lá = mask detector, đường xanh dương = detector,
  cyan = fit GT sliding window, tím = fit connected components dùng audit.
- `ablation_clear.json`: kết quả thử các biến thể trên bốn session clear
  straight/curve ở hai độ phân giải.
- `trial_smooth05.json`: thử smoothing 0,5 trên toàn bộ dữ liệu.

## Các thử nghiệm chưa đưa vào mặc định

Giảm smoothing xuống 0,5 cải thiện curve clear 512 từ 86,0% lên 98,2%, nhưng làm
straight rain 512 giảm từ 86,4% xuống 77,3%, dưới M1. Vì vậy giữ 0,85. Đã thử cả
không smoothing, một IPM mặt đường phẳng với thông số camera, và fit bậc hai có
điều kiện đủ độ trải điểm. Các prototype IPM/bậc hai chưa cải thiện ổn định và
không được đưa vào mã chạy chính. Đây chưa phải kết luận rằng mọi IPM hiệu chỉnh
hoặc mọi cách fit bậc hai đều kém hơn.

Session clear dùng để thử phương án, rain dùng kiểm tra hồi quy. Dữ liệu đã xem
trong quá trình phát triển nên cần bộ ghi mới để đánh giá độc lập.

## Chạy lại kiểm tra

```bash
python -m unittest discover -s tests -v
python -m scripts.eval_lane --session data/raw/m1/session_* data/raw/m1_1024/session_* \
  --report output/phase2/report.json --audit-dir output/phase2/audit

# So sánh smoothing bằng cùng benchmark; không ghi đè report mặc định.
python -m scripts.eval_lane --session data/raw/m1/session_* data/raw/m1_1024/session_* \
  --smooth 0.5 --report output/phase2/smooth05.json
```

Kiểm tra offline chỉ cần numpy/OpenCV; unittest không tạo actor hay kết nối server.
Muốn so với baseline detector trước đợt sửa cần dùng snapshot mã tương ứng;
hash nguồn của mỗi lần chạy nằm trong JSON. Không so trực tiếp thời gian đo
với số worker hoặc máy khác nhau.

## Lần chạy online ngày 16/11/2026

Đã kết nối CARLA 0.9.16 tại `100.85.59.63:3000`, map Town10HD_Opt. Kết nối cần
chạy ngoài sandbox mạng; các lần timeout trong sandbox trước đó chưa xác định
được trạng thái thực của server.

- Ghi mới bốn session trong `data/raw/m1_v2/`: `session_20261116_145209`
  (straight/clear), `session_20261116_145247` (curve/clear),
  `session_20261116_145303` (straight/rain), `session_20261116_145320` (curve/rain).
- Mỗi session đủ 150 frame, `complete=true`, đồng bộ đúng tick, không va chạm
  và không timeout sensor. Cả bốn session đủ GT và PASS M1 legacy/M1-v2;
  tổng 218/246 frame GT đạt ngưỡng, tương ứng **88,6%**.
- Tốc độ ghi thực tế 11,49–14,53 FPS, p95 tick/chờ sensor 65,4–94,4 ms;
  p95 ghi dữ liệu khoảng 22,8–23,7 ms. Chưa đạt mục tiêu 20 FPS theo thời gian thực.
- Sau khi chạy: `synchronous_mode=false`, không còn actor `ego_project*`.

[Video camera gốc cạnh detector](../output/phase2/online_20261116/carla_lane_detection.mp4)
dài 30 giây thời gian mô phỏng, H.264, 1024×400, 20 FPS. Mỗi tình huống dài 7,5 giây
theo thứ tự straight/clear, curve/clear, straight/rain, curve/rain. Xe do Traffic
Manager điều khiển, detector chạy offline trên dữ liệu mới ghi với road mask
từ simulator. Video từng session có cả GT nằm ở `eval/overlay.mp4`.

Metric nằm trong [report.json](../output/phase2/online_20261116/report.json),
nguồn từng đoạn và tốc độ ghi nằm trong
[video_manifest.json](../output/phase2/online_20261116/video_manifest.json).

## Ghi thêm dữ liệu

Lệnh lặp lại ba lần từng tổ hợp, dùng thư mục mới nếu cần giữ nguyên tập vừa ghi:

```bash
set -e
for repeat in 1 2 3; do
  for route_spawn in "straight 22" "curve 62"; do
    set -- $route_spawn
    for weather in clear rain; do
      python -m scripts.record_dataset --frames 150 --driver tm --target-speed 10 \
        --spawn-index "$2" --route "$1" --weather "$weather" --out data/raw/m1_v2
    done
  done
done
python -m scripts.eval_lane --session data/raw/m1_v2/session_* \
  --report output/phase2/new_data.json --audit-dir output/phase2/new_data_audit
```

Kiểm tra từng session đủ ít nhất 50 frame GT, không va chạm, metadata đồng bộ
đúng tick và route hợp lệ. Nếu thiếu GT, review ảnh và chọn spawn/độ dài đoạn
ghi phù hợp; tăng số frame có thể làm xe đi vào giao lộ hoặc đổi đặc tính route.
Chốt danh sách session chỉnh tham số và kiểm tra độc lập trước lần tuning mới.
Để xác nhận 20 Hz, đo cả detector lẫn tick/chờ sensor và ghi dữ liệu; không dùng
`fixed_delta=0.05` hoặc p95 detector riêng để kết luận tốc độ toàn hệ thống.

## Nhận diện live trong vòng lặp sensor

`src.main` đã ghép `world.tick()` → chờ đúng RGB/seg/depth → dựng road mask →
`LaneDetector.detect()` → hiển thị/video → tick kế tiếp. Detector chạy trên
dữ liệu vừa nhận ngay trong vòng lặp; không đọc lại dataset để tạo kết quả.
`--driver tm` cho Traffic Manager lái xe, còn detector hiện chỉ quan sát.

Mỗi phiên tạo một thư mục session riêng dưới `--out`, chứa:

- `frames.csv`: ID tick/sensor, thời gian mô phỏng, tốc độ, giao lộ, collision,
  lane invasion, phát hiện mới/giữ/mất, offset pixel và thời gian từng bước.
- `report.json`: FPS theo thời gian thực, số timeout/nhảy tick, deadline vượt
  50 ms, tỷ lệ mất làn, tỷ lệ mất khi xe chạy ≥1 km/h, thống kê ngoài giao lộ,
  p50/p95/max của từng bước và kết quả cleanup.
- `live.mp4` khi bật `--save-video`: RGB cạnh overlay được ghi ngay trong loop.
  FPS video theo thời gian mô phỏng, không phải FPS theo thời gian thực.

`work_ms` gồm tick, chờ/giải mã sensor, road mask, detector, telemetry, hiển thị,
ghi video và log console. `loop_ms` thêm sleep giới hạn 20 Hz. CSV/JSON ghi sau
khi dừng để không đưa chi phí ghi dataset PNG/NPY vào vòng lặp live. Thời gian
khởi tạo/dọn actor và đóng video không nằm trong thời gian chạy được đo.

Khi timeout hoặc ID tick bị gián đoạn, detector reset fit cũ. Khi gặp lỗi hoặc
Ctrl+C, chương trình vẫn đóng video, dọn actor, khôi phục world/thời tiết và
lưu báo cáo lỗi. Mặc định `--driver none` giữ hành vi xe đứng yên của harness cũ.

### Kiểm chứng thực tế ngày 16/11/2026

- Headless 60 giây, spawn 22, clear: 714 frame, **11,89 FPS**, không timeout,
  không nhảy tick, không va chạm. Chỉ 8 frame xe chạy ≥1 km/h; phần lớn thời
  gian xe đứng yên, nên không dùng tỷ lệ phát hiện 100% ở phiên này để kết luận
  chất lượng nhận diện khi di chuyển. P95 tick 44,1 ms, chờ sensor 77,2 ms,
  detector 19,1 ms, toàn bộ work 115,5 ms.
- Headless đoạn cong, spawn 62, clear: 150 frame, **17,08 FPS**, 145 frame xe
  chạy ≥1 km/h; 126 phát hiện mới, 20 giữ fit, 4 mất làn. P95 detector 5,28 ms,
  toàn bộ work 77,66 ms. Báo cáo có đủ schema `live-v1`.
- Live có video, spawn 62, clear: 150 frame, **14,06 FPS**, 127 phát hiện mới,
  19 giữ fit, 4 mất làn. Đây là lần chạy trước khi thêm cột thời gian telemetry
  và thống kê moving; có thể đọc tốc độ từng frame từ CSV.
- Live có video, spawn 62, rain: 150 frame, **17,61 FPS**, 145 frame xe chạy
  ≥1 km/h; 117 phát hiện mới, 18 giữ fit, 15 mất làn. P95 detector 4,67 ms,
  toàn bộ work 80,20 ms; tỷ lệ mất làn khi di chuyển 15/145 = 10,34%.
- Tất cả các phiên trên không có timeout, nhảy tick hoặc va chạm; cleanup không
  báo lỗi. Kiểm tra server cuối cùng: `synchronous_mode=false`, không còn actor
  `ego_project*`. Các phiên diễn ra ở thời điểm/điều kiện khác nhau, chưa phải
  phép so sánh có kiểm soát về chi phí bật/tắt video.
- **31 tests qua** trên Python 3.11, gồm thứ tự tick → sensor → detector, reset
  khi mất sensor, phân biệt fit giữ/đo mới, thống kê deadline và dọn tài nguyên
  khi có lỗi. Thư mục `tests/` đang được `.gitignore` của workspace bỏ qua.

[Video live trời quang và mưa](../output/live_validation/live_demo.mp4) dài 15
giây thời gian mô phỏng: 7,5 giây clear, tiếp theo 7,5 giây rain. Video này chỉ
ghép/chuyển codec từ các clip được ghi trong loop, không chạy lại detector offline.
Nguồn clip trong [manifest](../output/live_validation/live_demo_manifest.json).
Báo cáo/CSV gốc nằm trong các thư mục session dưới `output/live_validation/`.

Mục tiêu 20 FPS thực tế **chưa đạt**. Trong các lần kiểm tra, tick/chờ sensor
chiếm phần lớn ngân sách; cần khảo sát tiếp server render và đường truyền.
Các percentile của từng bước không cộng trực tiếp để ra percentile toàn vòng.
Lane loss là trạng thái detector, không thay thế M1 accuracy có GT. Phần điều
khiển theo làn và đầu ra hình học theo mét/góc vẫn thuộc bước tích hợp tiếp theo.
