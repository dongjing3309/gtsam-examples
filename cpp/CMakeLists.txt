# install header
file(GLOB ex_headers "*.h")
install(FILES ${ex_headers} DESTINATION include/gtsamexamples)

# Add unit tests
#gtsamAddTestsGlob(tests "tests/*.cpp" "" ${PROJECT_NAME})

# Add executables
gtsamAddExamplesGlob("examples/*.cpp" "" "${PROJECT_NAME}")


### install lib, only do that if *.c or *.cpp exist
# since C srcs exist, use *.c* as pattern, not *.cpp
file(GLOB ex_srcs "*.cpp")

# export cmake setting to the source code
add_library(${PROJECT_NAME} SHARED ${ex_srcs} ${ex_headers})

# linking with dependences
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)


