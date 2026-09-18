#!/bin/bash
# 오프라인 악보 재생 검사. 서버 오브젝트(obj/, main.o 제외)에 링크한다 → 먼저 drumrobot_server 에서 make.
# 사용 (저장소 루트에서): drumrobot_server/test/play_score_check.sh BF DS CO ...
#   결과: drumrobot_server/log/offline_<ID>_trajectory.csv, 선택 로그는 stderr (2> 파일로 받으면 select/pair/순간이동 집계 가능)
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
S="$ROOT/drumrobot_server"
BIN="${PLAY_SCORE_CHECK_BIN:-/tmp/play_score_check}"
OBJ="${PLAY_SCORE_CHECK_OBJ:-$S/obj}"        # 서버 오브젝트 디렉터리 (make OBJDIR=... 로 다른 곳에 빌드했으면 지정)
LIBS="-lpthread -lm -ldl -lrealsense2 -lpcl_common -lpcl_filters -lpcl_segmentation -lpcl_sample_consensus -lpcl_search -lpcl_kdtree -lpcl_visualization \
      -lvtkRenderingLOD-7.1 -lvtkRenderingCore-7.1 -lvtkFiltersSources-7.1 -lvtkCommonExecutionModel-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 -lvtkCommonCore-7.1"
g++ -std=c++17 -O2 -Wall -I"$S/include" -I"$S/lib" -I"$S/lib/dynamixel_sdk/include" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/usr/include/vtk-7.1 \
    "$S/test/play_score_check.cpp" "$OBJ"/*/*.o -o "$BIN" $LIBS
mkdir -p "$S/log"
cd "$ROOT" && "$BIN" "$@"
