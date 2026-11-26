# Kiểm chứng tốc độ thích ứng trong CARLA

Chế độ `--driver avoid` với mục tiêu trên 20 km/h lấy giới hạn nhỏ nhất từ
tầm mặt đường quan sát được, độ cong bản đồ và trần mục tiêu 40 km/h. Giới hạn
phanh dùng độ trễ dự phòng 0,6 s và gia tốc hãm giả định 3 m/s². Semantic/depth
384×216 cung cấp ứng viên vật cản trong phạm vi lọc 65 m. Vận tốc khép khoảng
cách được ước tính từ chuỗi đo ngắn cùng loại vật cản và vị trí trong làn.

Khoảng chuẩn bị vượt thay đổi theo tốc độ thực và tốc độ khép khoảng cách.
Lớp phanh dùng ngưỡng dừng bảo thủ với giả định vật cản đứng yên. Vùng quét
làn đích được kiểm tra trong 4 s bằng hộp bao và vận tốc actor CARLA, với
phép ngoại suy tuyến tính. Pure pursuit và PID thực thi thao tác đã chọn.
Khi mất vạch, dự phòng bản đồ cùng làn chỉ hoạt động tối đa 0,85 s kể từ tick
nhìn rõ gần nhất và có tốc độ mục tiêu tối đa 10 km/h. Mất frame đồng bộ
yêu cầu phanh và xóa lịch sử vật cản.

## Kết quả trên Town05, trời quang

Bốn mức tốc độ dùng cùng spawn 264 và cùng vị trí xe đỗ cách điểm xuất phát
100 m. Có một ca cho mỗi tổ hợp tốc độ và trạng thái làn kề. Cả tám ca chạy
760 tick, sạch va chạm, timeout và lệch tick.

| Mục tiêu (km/h) | Khoảng cách bắt đầu đặt 12 km/h, làn trống (m) | Tốc độ bắt đầu vượt (km/h) | Tốc độ lớn nhất sau vượt (km/h) | Khoảng cách nhỏ nhất khi dừng, làn bị chiếm (m) |
|---:|---:|---:|---:|---:|
| 20 | 29,88 | 12,51 | 20,02 | 3,96 |
| 25 | 35,07 | 12,30 | 25,01 | 4,64 |
| 30 | 39,51 | 12,22 | 30,00 | 4,62 |
| 40 | 50,25 | 11,89 | 40,08 | 4,73 |

- Đường cong với mục tiêu 40 km/h đạt đỉnh **38,48 km/h**. Sai số ngang lớn
  nhất khi chạy **0,39 m**, gia tốc ngang thực đo p95 **2,16 m/s²**. Giới hạn
  theo độ cong là giới hạn lập kế hoạch. Gia tốc đo được còn phụ thuộc sai số
  bám quỹ đạo và độ nghiêng mặt đường.
- Ca xe cắt vào ở mục tiêu 30 km/h dùng một actor di chuyển theo vận tốc đặt
  trước trong mô phỏng. Ego phản ứng với xe, giữ khoảng cách dọc nhỏ nhất
  **20,41 m** sau khi xe đó đã vào làn.
- Ca mất vạch ở mục tiêu 30 km/h chèn lỗi nhận diện trong **1,15 s**. Xe dùng
  bản đồ ngắn hạn, phanh khi lỗi kéo dài và trở lại chạy khi nhận diện phục hồi.
- Các ca đo không ghi video đạt khoảng **19,5–19,9 FPS thực**. Ca 40 km/h
  ghi Full HD đạt **9,2 FPS thực**, p95 thời gian xử lý mỗi vòng khoảng
  **126 ms**. Bước mô phỏng vẫn là 0,05 s.

Kết quả tổng hợp và đường dẫn CSV từng ca nằm ở
`output/phase7/adaptive_speed/verification.json`. Video ca vượt ở mục tiêu
40 km/h nằm cùng thư mục. Phạm vi kiểm chứng là mô phỏng thời tiết quang,
với thông tin hình học và actor từ CARLA. Các giá trị lớn hơn 40 km/h được
giới hạn ở mức mục tiêu đã chọn cho đợt thử.
