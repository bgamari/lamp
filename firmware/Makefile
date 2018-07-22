DEVICE          = stm32f030p6
OPENCM3_DIR     = libopencm3
OBJS            += main.o

ifeq ($(ENABLE_SEMIHOSTING),1)
LDFLAGS		      += --specs=rdimon.specs
LDLIBS		      += -lrdimon
CFLAGS	        += -DENABLE_SEMIHOSTING
endif

CFLAGS          += -Os -ggdb3
CXXFLAGS        += -fno-exceptions -fno-rtti $(ARCH_FLAGS) $(CFLAGS)
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.hex

clean:
	$(Q)$(RM) -rf binary.* *.o

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
