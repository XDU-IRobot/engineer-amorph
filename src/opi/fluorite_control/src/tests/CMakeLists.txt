
list(APPEND tests
    trajectory_tracker
    end_effector_pose
    keyboard
)

foreach(test ${tests})
    ament_auto_add_executable(${PROJECT_NAME}_test_${test} ${test}.cc)
    target_link_libraries(${PROJECT_NAME}_test_${test} ${PROJECT_NAME}_libs)
    install(TARGETS ${PROJECT_NAME}_test_${test}
        DESTINATION lib/${PROJECT_NAME})
endforeach()

unset(tests)
