# AutoPilot

PoC lái xe tự hành trong CARLA với nhận diện làn đường, giữ làn, phát hiện vật cản và chuyển làn để vượt khi làn đích trống.

## Demo theo bản đồ và thời tiết

Các GIF được ghi trực tiếp từ CARLA với mức đồ họa `Epic` cho cả hai chế độ thời tiết của PoC. Camera RGB ở bên trái, lớp phủ nhận diện ở bên phải.

| Bản đồ | Trời quang (`clear`) | Trời mưa (`rain`) |
|---|---|---|
| **Town10HD_Opt** | ![Xe vượt xe đỗ trên Town10HD_Opt khi trời quang](docs/assets/autopilot-demo.gif)<br>Vượt xe đỗ | ![Xe giảm tốc khi một xe khác cắt vào làn trên Town10HD_Opt trong mưa](docs/assets/town10hd-rain.gif)<br>Giảm tốc khi xe khác cắt vào làn |
| **Town04_Opt** | ![Xe nhường người đi bộ trên Town04_Opt khi trời quang](docs/assets/town04-clear.gif)<br>Nhường người đi bộ | ![Xe giữ làn bằng nhận diện RGB trên Town04_Opt trong mưa](docs/assets/town04-rain.gif)<br>Giữ làn bằng nhận diện RGB |
| **Town05** | ![Xe giảm tốc khi phát hiện xe phía trước trên Town05 khi trời quang](docs/assets/town05-clear.gif)<br>Giảm tốc khi phát hiện xe phía trước | ![Xe giữ làn trên Town05 trong mưa](docs/assets/town05-rain.gif)<br>Giữ làn trong mưa |

## Cài đặt CARLA

Các lệnh dưới đây tải bản CARLA 0.9.16 dùng cho PoC. [CARLA Quick Start](https://carla.readthedocs.io/en/latest/start_quickstart/) · [Bản phát hành 0.9.16](https://github.com/carla-simulator/carla/releases/tag/0.9.16)

```bash
mkdir -p ~/carla-0.9.16
cd ~/carla-0.9.16
curl -fL -o CARLA_0.9.16.tar.gz https://downloads.carlasim.com/Linux/CARLA_0.9.16.tar.gz
tar -xzf CARLA_0.9.16.tar.gz

python3 -m pip install carla

cd PythonAPI/carla/dist
python3 -m pip install carla-0.9.15-cp37m-manylinux_2_27_x86_64.whl

cd PythonAPI/examples
python3 -m pip install -r requirements.txt
```

Khởi động server trên Town10HD_Opt và giữ terminal này mở. Cờ `-RenderOffScreen` vẫn cung cấp hình ảnh cho camera khi chạy ngoài màn hình. [Chế độ render của CARLA](https://carla.readthedocs.io/en/0.9.16/adv_rendering_options/#off-screen-mode)

```bash
cd ~/carla-0.9.16
./CarlaUE4.sh -carla-rpc-port=3000
```

<a name="tái-lập-từ-clone-sạch"></a>

## Chạy AutoPilot

```bash
git clone https://github.com/khanhhdt/AutoPilot.git
cd AutoPilot
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Kiểm tra kết nối. Kết quả mong đợi là phiên bản `0.9.16` và map `Carla/Maps/Town10HD_Opt`.

```bash
python -c 'import carla; c = carla.Client("localhost", 3000); c.set_timeout(5); print(c.get_server_version(), c.get_world().get_map().name)'
```

Chạy hệ thống nhận diện làn, giữ làn, phát hiện vật cản và tránh. Lưu video RGB + overlay Full HD.

```bash
python -m scripts.run_suite --host localhost --port 3000 \
  --case avoid --video-hd 1920
```

## Cách hệ thống hoạt động

```mermaid
flowchart LR
    RGB["Camera RGB"] --> Lane["Nhận diện làn"]
    Depth["Semantic + depth"] --> Lead["Vật cản và TTC"]
    Lidar["LiDAR 360° + bản đồ làn"] --> Gap["Kiểm tra làn đích"]
    Lane --> Planner["Quyết định hành vi"]
    Lead --> Planner
    Gap --> Planner
    Planner --> Control["Pure pursuit + PID"]
    Control --> Ego["Điều khiển xe"]
```

<a name="phase-56--kiểm-chứng-và-demo"></a>

## Nghiệm thu PoC

Để CARLA ở chế độ async với world rảnh. Cần `ffmpeg` hỗ trợ `libx264` và `drawtext`, cùng `ffprobe` để kiểm tra video.

```bash
python -m scripts.verify_poc \
  --ffmpeg /usr/bin/ffmpeg --ffprobe /usr/bin/ffprobe
```

Ca `clear-road` đặt mục tiêu **50 km/h** trên đoạn thẳng trống. Xe phải giữ tốc độ đo được trong dải **50 ± 1,5 km/h** liên tục ít nhất **1 giây**, với 0 va chạm và 0 chạm vạch. Các ca vật cản và đường cong dùng mục tiêu 10 km/h.

Chạy riêng tiêu chí đường trống bằng `python -m scripts.run_suite --case clear-road`. Video nghiệm thu có hai ô Full HD và phát ở **30 FPS** từ mô phỏng **20 Hz**.

| Kiểm chứng bản `poc-v1.0` | Kết quả |
|---|---|
| Nhận diện làn thuần RGB | M1-v2 đạt 4/4 phiên, 96,3% khung đủ ground truth đạt ngưỡng |
| Tránh vật cản | 20/20 ca, 0 va chạm, 0 lần bắt đầu chuyển làn khi làn đích bị chặn |
| Trời quang và mưa | 4/4 lượt, gồm giữ làn qua cua và phản ứng với xe cắt vào |

[Kiến trúc](docs/ARCHITECTURE.md)

## Giấy phép

[MIT](LICENSE)
