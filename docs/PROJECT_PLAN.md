Phase 0 — Nền tảng repo (0.5 ngày)
Làm:
- configs/config.py: host/port, sync mode, fixed_delta=0.05, resolution (RGB 640×360, seg/depth 320×180), ngưỡng FSM/PID, đường dẫn output.
- requirements.txt (carla đã cài, opencv-python, numpy, pyyaml), .gitignore (bỏ *.whl, output/).
- README.md khung: cách chạy, yêu cầu.
- Chuyển IP server hardcode trong step1_view.py sang config.py.
Verify: python -c "from configs.config import CFG; print(CFG)"; step1_view.py vẫn chạy.

Phase 1 — Harness CARLA 20 Hz (1 ngày)
Làm:
- src/vehicle.py: spawn ego (thử lần lượt các spawn point), apply_control, destroy an toàn.
- src/sensors.py: RGB + semantic seg + depth + collision + lane_invasion; queue maxsize=1 (bỏ frame cũ).
- src/main.py: vào sync mode, vòng lặp tick → lấy frame → (perception) → control → apply; finally khôi phục settings và hủy actor.
- Log FPS/latency mỗi giây.
Verify: chạy 60 s → 19–21 fps, không rò actor sau Ctrl+C, server trở lại async mode.

Phase 2 — Record + lane detection offline (2 ngày)
Làm:
- scripts/record_dataset.py: lưu RGB + seg + depth + ego state + timestamp; 3 route (thẳng, cua, ngã tư), 2 kiểu thời tiết → 500–1000 frame.
- src/perception/lane_detection.py: ROI → blur → Canny → IPM → sliding window → polyfit; nhánh seg lấy road/lane mask làm chuẩn.
- scripts/eval_lane.py: so centerline dự đoán với seg ground truth.
- Xuất video overlay để xem bằng mắt.
Verify: trên tập record, sai số tâm làn < 15 px ở >80% frame, có bảng IoU/độ chính xác. Đây là mốc M1.

Phase 3 — Giữ làn (1.5 ngày)
Làm:
- src/control/pure_pursuit.py + src/control/pid.py; nối vào main.py, LaneState → steer, speed PID → throttle/brake.
- Tune trên Town03 (thẳng) rồi Town05 (cua).
Verify: chạy 2 phút, lane_invasion = 0, steer không dao động, sai số tốc độ < 3 km/h.

Phase 4 — Phát hiện vật cản + tránh (2 ngày)
Làm:
- src/perception/segmentation.py: palette, mask Road/RoadLine/Vehicles/Pedestrians.
- src/perception/obstacles.py: connected components + depth → khoảng cách, vị trí làn.
- src/planning/decision.py: FSM CRUISE/FOLLOW/AVOID_L/R/STOP, kiểm tra làn kề trống bằng ROI seg.
- scripts/scenarios.py: vật cản tĩnh, xe cut-in (Traffic Manager), người đi bộ.
Verify: 10/10 lần dừng trước vật cản tĩnh; 20 lượt không va chạm; chỉ chuyển làn khi làn kề trống. Mốc M2.

Phase 5 — Scenario suite + metrics (1.5 ngày)
Làm:
- src/metrics.py: CSV (t, speed, steer, e_y, TTC, collision, lane_invasion).
- scripts/run_suite.py: chạy tự động 6 scenario, tổng hợp bảng.
- tests/: unit test hình học (IPM, polyfit, pure pursuit) — chạy được không cần server.
Verify: 1 lệnh sinh báo cáo: success rate, va chạm/km, lane invasion/km, min TTC, FPS.

Phase 6 — Demo + báo cáo (1 ngày)
Làm: video demo, README hoàn chỉnh, báo cáo (dùng sơ đồ trong docs/), phần hạn chế/hướng phát triển.
Verify: clone sạch + 3 lệnh là tái lập được một run. Mốc M3.