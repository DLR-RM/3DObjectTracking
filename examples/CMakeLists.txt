cmake_minimum_required(VERSION 3.10)

add_executable(evaluate_rbot_dataset evaluate_rbot_dataset.cpp evaluator.cpp)
target_link_libraries(evaluate_rbot_dataset PUBLIC rbgt)

add_executable(evaluate_ablation_study evaluate_ablation_study.cpp evaluator.cpp)
target_link_libraries(evaluate_ablation_study PUBLIC rbgt)

add_executable(run_on_recorded_sequence run_on_recorded_sequence.cpp)
    target_link_libraries(run_on_recorded_sequence PUBLIC rbgt)

if(USE_AZURE_KINECT)
    add_executable(record_camera_sequence record_camera_sequence.cpp)
    target_link_libraries(record_camera_sequence PUBLIC rbgt)

    add_executable(run_on_camera_sequence run_on_camera_sequence.cpp)
    target_link_libraries(run_on_camera_sequence PUBLIC rbgt)
endif()
