cmake_minimum_required(VERSION 3.8)
project(ele_scout_kit)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/global_ht_gif_recorder.py
    scripts/ht_diagnostic_frame_recorder.py
    scripts/ht_fps_probe.py
    scripts/render_ht_gif_with_fps.py
    scripts/step_bag_player.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
