cmake_minimum_required(VERSION 3.27)
project(l4)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(testing)
add_subdirectory(googletest)

add_executable(l4 main.cpp Sequence.h ArraySequence.h ListSequence.h IDictionary.h Dictionary.h DynamicArray.h LinkedList.h
        PriorityQueue.h
        Graph.h)
