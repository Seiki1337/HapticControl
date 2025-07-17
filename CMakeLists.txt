# CMakeList.txt : HapticControl の CMake プロジェクト。ソースを含めて、次を定義します:
# プロジェクト専用ロジックはこちらです。
#
cmake_minimum_required (VERSION 3.8)

# サポートされている場合は、MSVC コンパイラのホット リロードを有効にします。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("HapticControl")

set(EIGEN_DIR "C:/eigen-3.4.0")
set(OPEN_HAPTICS_DIR "C:/OpenHaptics")
set(HAPTICS_DRIVER_DIR "${OPEN_HAPTICS_DIR}/Developer/3_5_0/lib/x64/Release")
set(OPEN_HAPTICS_INCLUDE_DIR "${OPEN_HAPTICS_DIR}/Developer/3_5_0/include")

link_directories(${HAPTICS_DRIVER_DIR})

include_directories(
    ${EIGEN_DIR}
    ${OPEN_HAPTICS_DIR}
    ${OPEN_HAPTICS_INCLUDE_DIR}
)

# ソースをこのプロジェクトの実行可能ファイルに追加します。
add_executable (HapticControl "HapticControl.cpp" "HapticControl.h")

target_link_libraries(HapticControl hd ws2_32)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET HapticControl PROPERTY CXX_STANDARD 20)
endif()

# TODO: テストを追加し、必要な場合は、ターゲットをインストールします。
