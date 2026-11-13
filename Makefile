CXX      = g++
CC       = gcc
AR       = ar

# OS detection for platform-specific settings
ifeq ($(OS),Windows_NT)
	# Windows settings
	EXE_SUFFIX := .exe
	SHARED_LIB_NAME := libgameskelio_x64.dll
	SHARED_LIB_LDFLAGS := -Wl,--out-implib,libgameskelio.dll.a
	CLEAN_EXTRAS := *.dll.a
else
	# Linux/Unix-like settings
	EXE_SUFFIX :=
	SHARED_LIB_NAME := libgameskelio.so
	SHARED_LIB_LDFLAGS :=
	CLEAN_EXTRAS :=
endif

CXXFLAGS = -fPIC -O2 -Wall -std=c++17 -Isrc/libs -Isrc -I.
CFLAGS   = -fPIC -O2 -Wall -Isrc/libs -Isrc -I.
LDFLAGS  =

TARGET = gskelconv$(EXE_SUFFIX)
REBIND_TARGET = gsrebind$(EXE_SUFFIX)
LIB_TARGET = libgameskelio.a
DLL_TARGET = $(SHARED_LIB_NAME)
OBJDIR = obj

# Library source files
LIB_SRCS = iqm_loader.cpp \
           anim_cfg.cpp \
           glb_writer.cpp \
           glb_loader.cpp \
           iqm_writer.cpp \
           cgltf_impl.cpp \
           cgltf_write_impl.cpp \
           skp_loader.cpp \
           fbx_writer.cpp \
           fbx_loader.cpp \
           gameskelio.cpp \
           orientation.cpp

LIB_OBJS = $(addprefix $(OBJDIR)/, $(LIB_SRCS:.cpp=.o)) \
           $(addprefix $(OBJDIR)/, fbx.o) \
           $(addprefix $(OBJDIR)/, ufbx.o) \
           $(addprefix $(OBJDIR)/, miniz.o)

MAIN_OBJ = $(OBJDIR)/gsconvert.o
REBIND_OBJ = $(OBJDIR)/gsrebind.o

# Default target
all: $(OBJDIR) $(LIB_TARGET) $(DLL_TARGET) $(TARGET) $(REBIND_TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(LIB_TARGET): $(LIB_OBJS)
	$(AR) rcs $@ $^

$(DLL_TARGET): $(LIB_OBJS)
	$(CXX) -shared -o $@ $^ $(LDFLAGS) $(SHARED_LIB_LDFLAGS)

# main.o needs to be compiled from main.c
$(OBJDIR)/gsconvert.o: tools/gsconvert.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/gsrebind.o: tools/gsrebind.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Link with g++ because libgameskelio.a contains C++ code
$(TARGET): $(MAIN_OBJ) $(LIB_TARGET)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) -lstdc++

$(REBIND_TARGET): $(REBIND_OBJ) $(LIB_TARGET)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) -lstdc++

$(OBJDIR)/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/fbx.o: src/libs/fbx.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/ufbx.o: src/libs/ufbx.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/miniz.o: src/libs/miniz.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/cgltf_impl.o: src/libs/cgltf_impl.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/cgltf_write_impl.o: src/libs/cgltf_write_impl.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR) $(TARGET) $(REBIND_TARGET) $(LIB_TARGET) $(DLL_TARGET) $(CLEAN_EXTRAS)

.PHONY: all clean
