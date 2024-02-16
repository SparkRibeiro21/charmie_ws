CC = gcc
CXX = g++
C_DEFS = -DSOFT_VERSION=$(SOFT_VERSION)
UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
	C_FLAGS = -std=c++0x -Wall -fPIC -Wno-sign-compare -Wno-unused-variable -g -I$(INC_DIR) $(LIBDIRS)
	LIBS += -lm -lpthread -fPIC -shared -framework IOKit -framework CoreFoundation
else
	C_FLAGS = -std=c++0x -Wall -fPIC -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -g -s $(C_DEFS) -I$(INC_DIR) $(LIBDIRS)
	LIBS += -lm -lpthread -fPIC -shared
endif

BUILDDIR = ./build/
INC_DIR = ./include/
SRC_DIR = ./src/
EXAMPLE_DIR = ./example/
BUILD_EXAMPLE_DIR = $(BUILDDIR)example/
BUILD_LIB_DIR = $(BUILDDIR)lib/
BUILD_OBJ_DIR = $(BUILDDIR)obj/
BUILD_MAP_DIR = $(BUILDDIR)map/

SRC_SERIAL_DIR = $(SRC_DIR)serial/
SRC_SERIAL_IMPL_DIR = $(SRC_SERIAL_DIR)impl/
SRC_SERIAL_IMPL_LISTPORT_DIR = $(SRC_SERIAL_IMPL_DIR)list_ports/


SRC_XARM_DIR = $(SRC_DIR)xarm/
SRC_XARM_CORE_DIR = $(SRC_XARM_DIR)core/
SRC_XARM_WRAPPER_DIR = $(SRC_XARM_DIR)wrapper/

SRC_XARM_CORE_COMMON_DIR = $(SRC_XARM_CORE_DIR)common/
SRC_XARM_CORE_DEBUG_DIR = $(SRC_XARM_CORE_DIR)debug/
SRC_XARM_CORE_INSTRUCTION_DIR = $(SRC_XARM_CORE_DIR)instruction/
SRC_XARM_CORE_OS_DIR = $(SRC_XARM_CORE_DIR)os/
SRC_XARM_CORE_PORT_DIR = $(SRC_XARM_CORE_DIR)port/

SRC_XARM := $(wildcard $(SRC_SERIAL_DIR)*.cc \
	$(SRC_SERIAL_IMPL_DIR)*.cc \
	$(SRC_SERIAL_IMPL_LISTPORT_DIR)*.cc \
	$(SRC_XARM_WRAPPER_DIR)*.cc \
	$(SRC_XARM_CORE_COMMON_DIR)*.cc \
	$(SRC_XARM_CORE_DEBUG_DIR)*.cc \
	$(SRC_XARM_CORE_INSTRUCTION_DIR)*.cc \
	$(SRC_XARM_CORE_OS_DIR)*.cc \
	$(SRC_XARM_CORE_PORT_DIR)*.cc \
	$(SRC_XARM_CORE_DIR)*.cc)
# OBJ_XARM := $(patsubst %.cc, %.o, $(SRC_XARM))

LIB_NAME = libxarm.so

SRC_EXAMPLE := $(wildcard $(EXAMPLE_DIR)*.cc)
# OBJ_EXAMPLE := $(patsubst %.cc, %.o, $(SRC_EXAMPLE))

XARM_OBJS = $(addprefix $(BUILD_OBJ_DIR), $(addsuffix .o, $(basename $(SRC_XARM))))
EXAMPLE_OBJS = $(addprefix $(BUILD_OBJ_DIR), $(addsuffix .o, $(basename $(SRC_EXAMPLE))))

all: xarm test

test: $(EXAMPLE_OBJS)
	mkdir -p $(BUILD_EXAMPLE_DIR)
	mkdir -p $(BUILD_OBJ_DIR)
	mkdir -p $(BUILD_MAP_DIR)
	for file in $(SRC_EXAMPLE); do \
		make test-`echo $$file | awk -F'/' '{print $$NF}' | awk -F'.cc' '{print $$1}'`; \
	done

ifeq ($(UNAME), Darwin)
xarm: $(XARM_OBJS)
	mkdir -p $(BUILD_LIB_DIR)
	mkdir -p $(BUILD_OBJ_DIR)
	mkdir -p $(BUILD_MAP_DIR)
	$(CXX) -o $(C_FLAGS) $^ -o $(BUILD_LIB_DIR)/$(LIB_NAME) $(LIBS) -Wl
test-%:
	mkdir -p $(BUILD_EXAMPLE_DIR)
	mkdir -p $(BUILD_OBJ_DIR)
	mkdir -p $(BUILD_MAP_DIR)
	$(CXX) -o $(C_FLAGS) $(addprefix $(BUILD_OBJ_DIR)example/, $(subst test-, , $@)).o -o $(addprefix $(BUILD_EXAMPLE_DIR), $(subst test-, , $@)) -L$(BUILD_LIB_DIR) -lxarm -Wl
else
xarm: $(XARM_OBJS)
	mkdir -p $(BUILD_LIB_DIR)
	mkdir -p $(BUILD_OBJ_DIR)
	mkdir -p $(BUILD_MAP_DIR)
	$(CXX) -o $(C_FLAGS) -s -fopenmp $^ -o $(BUILD_LIB_DIR)/$(LIB_NAME) $(LIBS) -Wl,-Map,$(BUILD_MAP_DIR)/$@.map
test-%:
	mkdir -p $(BUILD_EXAMPLE_DIR)
	mkdir -p $(BUILD_OBJ_DIR)
	mkdir -p $(BUILD_MAP_DIR)
	$(CXX) -o $(C_FLAGS) -s -fopenmp $(addprefix $(BUILD_OBJ_DIR)example/, $(subst test-, , $@)).o -o $(addprefix $(BUILD_EXAMPLE_DIR), $(subst test-, , $@)) -L$(BUILD_LIB_DIR) -lxarm -Wl,-Map,$(BUILD_MAP_DIR)/$@.map
endif

$(BUILD_OBJ_DIR)%.o: %.c
	mkdir -p $(dir $@)
	$(CC) -c $(C_FLAGS) $< -o $@

$(BUILD_OBJ_DIR)%.o: %.cc
	mkdir -p $(dir $@)
	$(CXX) -c $(C_FLAGS) $< -o $@

install:
	cp -rf include/serial /usr/include
	cp -rf include/xarm /usr/include
	cp -f build/lib/$(LIB_NAME) /usr/lib/
uninstall:
	rm -rf /usr/include/serial
	rm -rf /usr/include/xarm
	rm -f /usr/lib/$(LIB_NAME)

clean:
	rm -rf ./build

