cmake_minimum_required(VERSION 3.10.2)
project(imomd-rtt)

set(CMAKE_CXX_STANDARD 11)

include_directories(src)
include_directories(include)

add_executable(app
        main.cpp
        src/osm_parser.cpp
        src/imomd_rrt_star.cpp
        src/eci_gen_tsp_solver.cpp
        src/tinyxml2/tinyxml2.cpp
        src/baseline/ana_star.cpp
        src/baseline/bi_a_star.cpp
        )

