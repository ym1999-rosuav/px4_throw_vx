cmake_minimum_required(VERSION 3.10)

project(throw_impulse)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(ThrowImpulse SHARED ThrowImpulse.cc)

target_link_libraries(
  ThrowImpulse
  PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
