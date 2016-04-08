GCOV_PROFILE := y
#subdir-ccflags-y := -Werror
EXTRA_CFLAGS += -I../kernel-nvhost/drivers/video/tegra/host
ifeq ($(CONFIG_ARM64),y)
subdir-ccflags-y += -Iarch/arm/mach-tegra/include
ccflags += -Iarch/arm/mach-tegra/include
endif

obj-$(CONFIG_TEGRA_GRHOST) += ../../../../kernel-nvhost/drivers/video/tegra/host/
obj-$(CONFIG_TEGRA_NVMAP) += nvmap/
