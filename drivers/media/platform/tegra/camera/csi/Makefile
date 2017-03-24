GCOV_PROFILE := y
ccflags-y += -Invidia/nvhost/drivers/video/tegra/host
ccflags-y += -Invidia/nvhost/include
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Idrivers/media/platform/tegra
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -Invidia/nvhost-t18x/drivers/video/tegra/host
endif

obj-y += csi.o csi2_fops.o csi4_fops.o
