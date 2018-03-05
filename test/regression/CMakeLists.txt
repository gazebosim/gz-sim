set(TEST_TYPE "REGRESSION")

set(tests
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(TYPE REGRESSION SOURCES ${tests})
