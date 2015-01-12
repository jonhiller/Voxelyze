#voxelyze makefile


# This is the directory in which to find subdirectories to install/find headers and libs:
USER_HOME_PATH = $(HOME)

VOXELYZE_NAME = voxelyze.0.9.92
VOXELYZE_VERSION = voxelyze.0.9
VOXELYZE_LIB_NAME = lib$(VOXELYZE_NAME).a
VOXELYZE_LIB_VERSION = lib$(VOXELYZE_VERSION).a

CXX=g++
CC=g++
INCLUDE= -I./include
FLAGS = -O3 -std=c++11 -DPARDISO_5=1 -Wall $(INCLUDE)

VOXELYZE_SRC = \
	src/Voxelyze.cpp \
	src/VX_Voxel.cpp \
	src/VX_External.cpp \
	src/VX_Link.cpp \
	src/VX_Material.cpp \
	src/VX_MaterialVoxel.cpp \
	src/VX_MaterialLink.cpp \
	src/VX_Collision.cpp \
	src/VX_LinearSolver.cpp \
	src/VX_MeshRender.cpp 

VOXELYZE_OBJS = \
	src/Voxelyze.o \
	src/VX_Voxel.o \
	src/VX_External.o \
	src/VX_Link.o \
	src/VX_Material.o \
	src/VX_MaterialVoxel.o \
	src/VX_MaterialLink.o \
	src/VX_Collision.o \
	src/VX_LinearSolver.o \
	src/VX_MeshRender.o
		
	
.PHONY: clean all

#dummy target that builds everything for the library
all: $(VOXELYZE_LIB_VERSION)
	

# Auto sorts out dependencies (but leaves .d files):
%.o: %.cpp
#	@echo making $@ and dependencies for $< at the same time
	@$(CC) -c $(FLAGS) -o $@ $<
	@$(CC) -MM -MP $(FLAGS) $< -o $*.d

-include *.d


$(VOXELYZE_LIB_VERSION):	$(VOXELYZE_OBJS)
	ar rcs lib/$(VOXELYZE_LIB_VERSION) $(VOXELYZE_OBJS)


clean:
	rm -rf *.o */*.o *.d */*.d lib/$(VOXELYZE_LIB_VERSION)


##################################################


$(USER_HOME_PATH)/include:
		mkdir $(USER_HOME_PATH)/include

$(USER_HOME_PATH)/lib:
		mkdir $(USER_HOME_PATH)/lib


installusr:     $(USER_HOME_PATH)/include $(USER_HOME_PATH)/lib
		cp lib/$(VOXELYZE_LIB_VERSION) $(USER_HOME_PATH)/lib/$(VOXELYZE_LIB_VERSION)
		rm -f $(USER_HOME_PATH)/lib/$(VOXELYZE_LIB_NAME)
		ln -s lib/$(VOXELYZE_LIB_VERSION) $(USER_HOME_PATH)/lib/$(VOXELYZE_LIB_NAME)
		rm -rf $(USER_HOME_PATH)/include/$(VOXELYZE_VERSION)
		-mkdir $(USER_HOME_PATH)/include/$(VOXELYZE_VERSION)
		cp include/*.h $(USER_HOME_PATH)/include/$(VOXELYZE_VERSION)
		-mkdir $(USER_HOME_PATH)/include/$(VOXELYZE_VERSION)/rapidjson
		cp include/rapidjson/*.h $(USER_HOME_PATH)/include/$(VOXELYZE_VERSION)/rapidjson
		rm -f $(USER_HOME_PATH)/include/$(VOXELYZE_NAME)
		ln -s $(VOXELYZE_VERSION) $(USER_HOME_PATH)/include/$(VOXELYZE_NAME)



installglobal:
		cp lib/$(VOXELYZE_LIB_VERSION) $(GLOBAL_PATH)/lib/$(VOXELYZE_LIB_VERSION)
		rm -f $(GLOBAL_PATH)/lib/$(VOXELYZE_LIB_NAME)
		ln -s lib/$(VOXELYZE_LIB_VERSION) $(GLOBAL_PATH)/lib/$(VOXELYZE_LIB_NAME)
		rm -rf $(GLOBAL_PATH)/include/$(VOXELYZE_VERSION)
		-mkdir $(GLOBAL_PATH)/include/$(VOXELYZE_VERSION)
		cp include/*.h $(GLOBAL_PATH)/include/$(VOXELYZE_VERSION)
		-mkdir $(GLOBAL_PATH)/include/$(VOXELYZE_VERSION)/rapidjson
		cp include/rapidjson/*.h $(GLOBAL_PATH)/include/$(VOXELYZE_VERSION)/rapidjson
		rm -f $(GLOBAL_PATH)/include/$(VOXELYZE_NAME)
		ln -s $(VOXELYZE_VERSION) $(GLOBAL_PATH)/include/$(VOXELYZE_NAME)

