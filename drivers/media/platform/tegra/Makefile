GCOV_PROFILE := y

subdir-ccflags-y := -Werror
#
# Makefile for the video capture/playback device drivers.
#
obj-y				+= csi/
ifndef CONFIG_ARCH_TEGRA_210_SOC
obj-y				+= camera/
endif
obj-y				+= vi/
obj-$(CONFIG_VIDEO_ISC)	+= auto/
obj-$(CONFIG_VIDEO_CAMERA)	+= regmap_util.o

