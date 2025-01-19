cmake_minimum_required(VERSION 3.25)
project(test)
set(CMAKE_CXX_STANDARD 17)

enable_testing()

add_executable(test test.cpp ../ArraySequence.h
        ../ListSequence.h
        ../DynamicArray.h
        ../LinkedList.h
        ../PriorityQueue.h
        ../Graph.h
        ../Dictionary.h
        ../IDictionary.h
        ../Sequence.h)

target_link_libraries(test gtest gtest_main)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include(GoogleTest)