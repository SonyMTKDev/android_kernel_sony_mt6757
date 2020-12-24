obj-$(CONFIG_SONY_DBGCONTROL) += dbgcontrol.o
obj-$(CONFIG_RAMDUMP_TAGS) += debug_memory.o
obj-$(CONFIG_RAMDUMP_TAGS) += board-rdtags.o
CFLAGS_board-rdtags.o  := \
       -D"INFO_PRODUCT=\"$(if $(TARGET_PRODUCT),$(TARGET_PRODUCT),unknown)\"" \
       -D"INFO_VARIANT= \
       \"$(if $(TARGET_BUILD_VARIANT),$(TARGET_BUILD_VARIANT),unknown)\"" \
       -D"INFO_BUILDID= \
       \"$(if $(SEMC_SYSTEM_VERSION),$(SEMC_SYSTEM_VERSION),private)\""
