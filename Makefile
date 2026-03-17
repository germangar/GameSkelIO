CXX      = g++
CC       = gcc
AR       = ar
CXXFLAGS = -O2 -Wall -std=c++17 -Ilibs -I.
CFLAGS   = -O2 -Wall -Ilibs -I.
LDFLAGS  =

TARGET = gskelconv.exe
REBIND_TARGET = gsrebind.exe
LIB_TARGET = libgameskelio.a
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
           gameskelio.cpp

LIB_OBJS = $(addprefix $(OBJDIR)/, $(LIB_SRCS:.cpp=.o)) \
           $(addprefix $(OBJDIR)/, fbx.o) \
           $(addprefix $(OBJDIR)/, ufbx.o) \
           $(addprefix $(OBJDIR)/, miniz.o)

MAIN_OBJ = $(OBJDIR)/main.o
REBIND_OBJ = $(OBJDIR)/gsrebind.o

# Default target
all: $(OBJDIR) $(LIB_TARGET) $(TARGET) $(REBIND_TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(LIB_TARGET): $(LIB_OBJS)
	$(AR) rcs $@ $^

# main.o needs to be compiled from main.c
$(OBJDIR)/main.o: tools/main.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/gsrebind.o: tools/gsrebind.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Link with g++ because libgameskelio.a contains C++ code
$(TARGET): $(MAIN_OBJ) $(LIB_TARGET)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) -lstdc++

$(REBIND_TARGET): $(REBIND_OBJ) $(LIB_TARGET)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) -lstdc++

$(OBJDIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/fbx.o: libs/fbx.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/ufbx.o: libs/ufbx.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/miniz.o: libs/miniz.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/cgltf_impl.o: libs/cgltf_impl.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/cgltf_write_impl.o: libs/cgltf_write_impl.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR) $(TARGET) $(REBIND_TARGET) $(LIB_TARGET)

.PHONY: all clean
