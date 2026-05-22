# ===== 컴파일러 설정 =====
CC = g++

# ===== 컴파일 옵션 =====
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC

# ===== 헤더 파일 경로 =====
INCLUDE = -I./include -I./lib

# ===== 링크할 라이브러리 =====
LDFLAGS = -lpthread

# ===== 경로 변수 =====
SRCDIR = ./src
OBJDIR = ./obj
BINDIR = ./bin
EXECUTABLE = main.out

# ===== 소스/오브젝트 파일 자동 탐색 =====
SOURCES := $(shell find $(SRCDIR) -name '*.cpp')
OBJFILES := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

# ===== 기본 빌드 타깃 =====
all: directories $(BINDIR)/$(EXECUTABLE)

# ===== 폴더 생성 =====
directories:
	mkdir -p $(BINDIR)
	mkdir -p $(OBJDIR)/common $(OBJDIR)/hardware $(OBJDIR)/input $(OBJDIR)/kinematics $(OBJDIR)/net $(OBJDIR)/realtime $(OBJDIR)/trajectory

# ===== 링크: .o 파일들 → 실행파일 =====
$(BINDIR)/$(EXECUTABLE): $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# ===== 컴파일: .cpp 파일 → .o 파일 (패턴 규칙) =====
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# ===== 청소 =====
clean:
	rm -rf $(OBJDIR) $(BINDIR)