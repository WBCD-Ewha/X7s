import os
import subprocess
from pathlib import Path
import pyrealsense2 as rs


def _find_repo_root(start: Path | None = None, anchor: str = "X7s_PLAY") -> Path:
    """
    Locate the directory named *anchor* anywhere in the ancestor *or sibling*
    hierarchy of *start*.  Raises FileNotFoundError if not found.

    Search strategy:
    1. Walk up the parents of *start*  (.) .. ../.. …
    2. For each ancestor, also check its direct children for *anchor*
       (covers sibling repositories at the same level).
    """
    start_path = (start or Path(__file__)).resolve()

    for parent in [start_path, *start_path.parents]:
        # ① 현재 경로가 바로 anchor 인가?
        if parent.name == anchor:
            return parent

        # ② 형제/자식 중에 anchor 폴더가 있는가?
        sibling = parent / anchor
        if sibling.is_dir():
            return sibling

    raise FileNotFoundError(
        f"Could not locate repository root '{anchor}' starting from {start_path}"
    )

def get_intrinsics(serial_num):
    pipe = rs.pipeline()
    cfg  = rs.config() # serial number 이용해서 카메라 고르도록 수정

    # Depth 스트림: 카메라가 지원하는 첫번째 프로필을 자동 선택
    cfg.enable_stream(rs.stream.depth)   # ← 해상도/프레임레이트 미지정

    profile = pipe.start(cfg)

    depth_profile = profile.get_stream(rs.stream.depth)
    video_profile = depth_profile.as_video_stream_profile()
    intr          = video_profile.get_intrinsics()

    print({
        "image_resolution": {
            "width":  intr.width,
            "height": intr.height
        },
        "focal_lengths_in_pixels": {
            "fx": intr.fx,
            "fy": intr.fy
        },
        "principal_point_in_pixels": {
            "cx": intr.ppx,
            "cy": intr.ppy
        }
    })
    pipe.stop()

def observe(camera_name: str, sample_id: int | str, subtask_num: int | str) -> None:
    """
    Capture a depth image, RGB image, and point cloud from an Intel RealSense camera
    and store them under the dataset directory for the given sample/sub‑task.

    Parameters
    ----------
    camera_name : str
        ROS namespace of the camera (e.g. ``camera_h``).
    sample_id : int | str
        Sequential ID of the dataset sample.
    subtask_num : int | str
        Sub‑task index within the sample.
    """
    # ------------------------------------------------------------------
    # 1. Resolve paths
    # ------------------------------------------------------------------
    x7s_play  = _find_repo_root(anchor= "X7s_PLAY")
    x7s = _find_repo_root(start= x7s_play, anchor= "X7s")
    script_dir  = x7s_play / "realsense_camera" / "src" / "ros_realsense2_camera" / "scripts"
    obs_dir     = x7s / "datasets" / f"sample_{sample_id}" / f"subtask{subtask_num}" / "observation_start"
    obs_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # 2. Common environment (libffi 7 for cv_bridge issue)
    # ------------------------------------------------------------------
    env = os.environ.copy()
    env["LD_PRELOAD"] = "/usr/lib/x86_64-linux-gnu/libffi.so.7"
    # ❷ ROS Python 모듈( rospy 등) 탐색 경로 추가
    env["PYTHONPATH"] = (
            "/opt/ros/noetic/lib/python3/dist-packages:"
            + env.get("PYTHONPATH", "")
    )

    # ❸ ROS 공유 라이브러리(.so) 경로 추가
    env["LD_LIBRARY_PATH"] = (
            "/opt/ros/noetic/lib:"
            + env.get("LD_LIBRARY_PATH", "")
    )

    # ------------------------------------------------------------------
    # 3. Command templates
    # ------------------------------------------------------------------
    base_cmd = ["python", "rs2_listener.py",
                "--camera_name", camera_name,
                "--file_directory", str(obs_dir), "--timeout", "2"]

    cmds = [
        base_cmd[:2] + ["depthStream"]   + base_cmd[2:],
        base_cmd[:2] + ["colorStream"]   + base_cmd[2:],
        base_cmd[:2] + ["pointscloud"]   + base_cmd[2:],
    ]

    # ------------------------------------------------------------------
    # 4. Execute sequentially
    # ------------------------------------------------------------------
    for cmd in cmds:
        try:
            subprocess.run(cmd, cwd=script_dir, env=env, check=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Command failed: {' '.join(cmd)}\n{e}") from e

    print(f"✓ Observation saved under: {obs_dir}")

if __name__ == "__main__":
    observe(camera_name="camera_l", sample_id=0, subtask_num=1)