"""Make an annotated H.264 demo from a completed six-case suite."""

import argparse
import json
import subprocess
from pathlib import Path


CLIPS = {
    "clear-road": ("Lane keeping", 1., 5.),
    "static": ("Stopping for a barrier", 2., None),
    "cut-in": ("Responding to a cut-in", 4., 11.),
    "pedestrian": ("Yielding to a pedestrian", 1.5, 7.),
    "avoid": ("Passing on the right", .5, None),
    "avoid-left": ("Passing on the left", .5, None),
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--ffmpeg", default="ffmpeg", help="FFmpeg executable with libx264 and drawtext")
    parser.add_argument("--hd-60", action="store_true", help="assemble a 60-second, 30-fps two-panel HD demo")
    args = parser.parse_args()
    summary = json.loads(args.summary.read_text())
    reports = {trial["name"]: Path(trial["report"]) for trial in summary["trials"] if trial.get("passed")}
    command = [args.ffmpeg, "-hide_banner", "-loglevel", "error", "-n"]
    filters = []
    intro_trim = 0.
    if args.hd_60:
        durations = {}
        for name in CLIPS:
            if name not in reports:
                parser.error(f"missing successful case: {name}")
            report = json.loads(reports[name].read_text())
            if report.get("video_layout") != "rgb-overlay" or report.get("rgb_size") != [1024, 576]:
                parser.error(f"{name} needs a video recorded with --video-hd")
            durations[name] = report["frames"] / report["video_fps_sim"]
        intro_trim = min(max(0., sum(durations.values()) - 60.), max(0., durations["clear-road"] - 2.))
    for index, (name, (title, start, end)) in enumerate(CLIPS.items()):
        if name not in reports:
            parser.error(f"missing successful case: {name}")
        video = reports[name].with_name("live.mp4")
        if not video.is_file():
            parser.error(f"missing video for {name}; run the suite with --save-video")
        command.extend(["-i", str(video)])
        if args.hd_60:
            start, end = (intro_trim if name == "clear-road" else 0.), None
        trim = f"trim=start={start}" + (f":end={end}" if end else "")
        filters.append(f"[{index}:v]{trim},setpts=PTS-STARTPTS,setsar=1,"
                       f"drawbox=x=iw-390:y=0:w=390:h=26:color=black:t=fill,"
                       f"drawtext=text='{title}':x=w-tw-10:y=4:fontsize=18:fontcolor=white[clip{index}]")
    count = len(CLIPS)
    concat = "".join(f"[clip{i}]" for i in range(count)) + f"concat=n={count}:v=1:a=0"
    if args.hd_60:
        concat += ",fps=30,tpad=stop_mode=clone:stop_duration=60,trim=end_frame=1800"
    filters.append(concat + "[demo]")
    out = args.out or args.summary.with_name("demo_hd_60s.mp4" if args.hd_60 else "demo.mp4")
    subprocess.run([*command, "-filter_complex_threads", "2", "-filter_complex", ";".join(filters),
                    "-map", "[demo]", "-c:v", "libx264", "-threads", "4",
                    "-crf", "18" if args.hd_60 else "20", "-preset", "fast", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(out)], check=True)
    print(out)


if __name__ == "__main__":
    main()
