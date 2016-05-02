GCOV_PROFILE := y
GCOV_PROFILE_panel-a-1080p-11-6.o := n
GCOV_PROFILE_panel-a-1080p-14-0.o := n
GCOV_PROFILE_panel-a-1200-1920-8-0.o := n
GCOV_PROFILE_panel-a-1200-800-8-0.o := n
GCOV_PROFILE_panel-a-edp-1080p-14-0.o := n
GCOV_PROFILE_panel-c-lvds-1366-14.o := n
GCOV_PROFILE_panel-i-edp-1080p-11-6.o := n
GCOV_PROFILE_panel-j-1440-810-5-8.o := n
GCOV_PROFILE_panel-j-720p-5.o := n
GCOV_PROFILE_panel-l-720p-5-loki.o := n
GCOV_PROFILE_panel-lgd-wxga-7-0.o := n
GCOV_PROFILE_panel-o-720p-6-0.o := n
GCOV_PROFILE_panel-p-wuxga-10-1.o := n
GCOV_PROFILE_panel-s-edp-uhdtv-15-6.o := n
GCOV_PROFILE_panel-s-edp-wqxgap-15-6.o := n
GCOV_PROFILE_panel-s-wqxga-10-1.o := n
GCOV_PROFILE_panel-s-wuxga-7-0.o := n
GCOV_PROFILE_panel-s-wuxga-8-0.o := n

asflags-y				+= -march=armv7-a

# Temp board files (should be gone when we go DT)

obj-y                   += panel-lgd-wxga-7-0.o
obj-y                   += panel-p-wuxga-10-1.o
obj-y                   += panel-s-wqxga-10-1.o
obj-y                   += panel-s-wuxga-7-0.o
obj-y                   += panel-s-wuxga-8-0.o
obj-y                   += panel-a-1200-800-8-0.o
obj-y                   += panel-a-1200-1920-8-0.o
obj-y                   += panel-a-edp-1080p-14-0.o
obj-y                   += panel-i-edp-1080p-11-6.o
obj-y                   += panel-c-lvds-1366-14.o
obj-y                   += panel-a-1080p-14-0.o
obj-y                   += panel-j-1440-810-5-8.o
obj-y                   += panel-l-720p-5-loki.o
obj-y                   += panel-j-720p-5.o
obj-y                   += panel-s-edp-uhdtv-15-6.o
obj-y                   += panel-o-720p-6-0.o
obj-y                   += panel-n-wqxga-6-0.o

obj-y                                   += io.o
obj-y                                   += irq.o
obj-y					+= flowctrl.o
obj-y					+= pm.o
obj-y					+= reset.o
obj-y					+= reset-handler.o
obj-y					+= sleep.o
obj-y					+= tegra.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= sleep-tegra20.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= pm-tegra20.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= cpuidle-tegra20.o
endif
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= sleep-tegra30.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= pm-tegra30.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= cpuidle-tegra30.o
endif
obj-$(CONFIG_SMP)			+= platsmp.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o

obj-$(CONFIG_ARCH_TEGRA_114_SOC)	+= sleep-tegra30.o
obj-$(CONFIG_ARCH_TEGRA_114_SOC)	+= pm-tegra30.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_ARCH_TEGRA_114_SOC)	+= cpuidle-tegra114.o
endif
obj-$(CONFIG_ARCH_TEGRA_124_SOC)	+= sleep-tegra30.o
obj-$(CONFIG_ARCH_TEGRA_124_SOC)	+= pm-tegra30.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_ARCH_TEGRA_124_SOC)	+= cpuidle-tegra114.o
endif

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-paz00.o

obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += board-t210ref.o
obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += board-t210ref-camera.o
obj-${CONFIG_ARCH_TEGRA_21x_SOC}      += tegra21_init.o

