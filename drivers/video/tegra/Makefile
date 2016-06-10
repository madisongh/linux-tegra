GCOV_PROFILE := y
#subdir-ccflags-y := -Werror
EXTRA_CFLAGS += -I../nvhost/drivers/video/tegra/host
ifeq ($(CONFIG_ARM64),y)
subdir-ccflags-y += -Iarch/arm/mach-tegra/include
ccflags += -Iarch/arm/mach-tegra/include
endif

obj-$(CONFIG_TEGRA_GRHOST) += ../../../../nvhost/drivers/video/tegra/host/
obj-$(CONFIG_TEGRA_NVMAP)  += ../../../../nvmap/drivers/video/tegra/nvmap/
obj-$(CONFIG_TEGRA_DC)     += ../../../../display/drivers/video/tegra/dc/
obj-$(CONFIG_FB_TEGRA)     += ../../../../display/drivers/video/tegra/fb.o
obj-$(CONFIG_TEGRA_CAMERA_PLATFORM) += camera/
