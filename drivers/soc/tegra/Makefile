obj-y += fuse/

ifeq ($(CONFIG_ARCH_TEGRA),y)
obj-y += common.o
obj-y += flowctrl.o
obj-y += pmc.o
endif

obj-$(CONFIG_ARCH_TEGRA_210_SOC)	+= pm-tegra210.o
obj-$(CONFIG_TEGRA_PROC_POWER_MODEL)	+= tegra-ppm.o
obj-$(CONFIG_TEGRA_DVFS)       += tegra-dvfs.o cvb.o
obj-$(CONFIG_TEGRA_210_DVFS)   += tegra210-dvfs.o
obj-$(CONFIG_TEGRA_CPU_EDP)		+= tegra-cpu-edp.o
obj-$(CONFIG_TEGRA_GPU_EDP)		+= tegra-gpu-edp.o
obj-$(CONFIG_TEGRA_SYS_EDP)		+= sysedp/
