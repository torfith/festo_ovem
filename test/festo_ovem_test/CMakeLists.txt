set(SOURCES festo_ovem_test.cpp)
add_executable(festo_ovem_test ${SOURCES})
target_link_libraries(festo_ovem_test ${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS festo_ovem_test DESTINATION bin)