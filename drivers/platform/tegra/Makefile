#
# Makefile for linux/drivers/platform/tegra
# Tegra Platform-Specific Drivers
#

GCOV_PROFILE := y

subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/platform/tegra/include \
		    -I$(srctree)/../t18x/drivers/platform/tegra/ \
		    -I$(srctree)/arch/arm/mach-tegra

ifeq ($(CONFIG_ARCH_TEGRA_21x_SOC),y)
obj-$(CONFIG_USB_SUPPORT)		+= tegra_usb_pmc.o
endif
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o
obj-$(CONFIG_OF_TEGRA_IOMMU_SMMU)       += iommu.o
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG)    += tegra_bootloader_debug.o
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG_INIT) += tegra_bootloader_debug_init.o

obj-y					+= tegra_vpr.o
obj-y					+= smc-calls.o

obj-$(CONFIG_TEGRA_CAMERA_MIPI_CAL)     += mipi-cal.o
obj-$(CONFIG_TEGRA_NVADSP)		+= nvadsp/
obj-$(CONFIG_TEGRA_MC) += mc/

ifeq ($(CONFIG_TEGRA_PTM),y)
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)         += tegra_ptm_t210.o
endif

obj-y += nvdumper/
ifneq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-$(CONFIG_PM_SLEEP) += pm-irq.o
endif
obj-$(CONFIG_TEGRA_PMC) += pmc.o
obj-$(CONFIG_TEGRA210_BOOTROM_PMC) += tegra-bootrom-pmc-config.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)	+= tegra-pmc-powerdetect.o
ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)	+= tegra21-pmc-iopower.o
endif
obj-$(CONFIG_PM) += pm.o

obj-$(CONFIG_TEGRA_MC_DOMAINS) += pm_domains.o

obj-$(CONFIG_TEGRA_WAKEUP_MONITOR)      += tegra_wakeup_monitor.o

obj-y += powergate/

# tempporarily support soctherm platform driver for T12x with include files
# from arch/arm/tegra area.
ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
ccflags-y += -I$(srctree)/arch/arm/mach-tegra
endif

ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
ccflags-y += -I$(srctree)/arch/arm/mach-tegra
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)        += tegra_throttle.o
endif

ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)
obj-y += dvfs.o
obj-$(CONFIG_TEGRA_CLK_FRAMEWORK) += clock.o tegra-non-dt-clock-reset.o
obj-$(CONFIG_DEBUG_FS) += clocks_stats.o
ifeq ($(CONFIG_ARCH_TEGRA_13x_SOC),y)
obj-y += tegra13_dvfs.o
endif
obj-$(CONFIG_ARCH_TEGRA_HAS_CL_DVFS) += tegra_cl_dvfs.o
endif

obj-$(CONFIG_TEGRA_EDP_LIMITS)          += cpu-edp.o # must precede cpu-tegra.o

ifeq ($(CONFIG_CPU_FREQ),y)
#obj-$(CONFIG_ARCH_TEGRA_21x_SOC)                  += cpu-tegra.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += cpu-tegra21.o
endif
endif

obj-y += flowctrl.o
obj-$(CONFIG_TEGRA3_PM) += pm-t3.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL) += sysfs-cluster.o
obj-$(CONFIG_TEGRA_HMP_CLUSTER_CONTROL) += sysfs-cluster.o

obj-y += io.o
obj-$(CONFIG_TEGRA_CORE_VOLT_CAP) += tegra_core_volt_cap.o
ifneq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-y += ahb.o
endif
obj-$(CONFIG_TEGRA_CENTRAL_ACTMON) += central_actmon/
obj-$(CONFIG_TEGRA_CPU_VOLT_CAP)	+= tegra_volt_cap.o

ifeq ($(CONFIG_PM_SLEEP),y)
obj-y += tegra-wakeups.o
endif

ifneq ($(CONFIG_ARM64),)

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra



obj-$(CONFIG_TEGRA_CLK_FRAMEWORK) += tegra_clocks_pll.o
ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)

# FIXME: Bug 1211919:
# Remove CFLAGS_tegra21_clocks.o flag when tegra21_clocks.c becomes
# stabilized.
CFLAGS_tegra21_clocks.o = -Werror -Wno-unused-function -Wno-unused-variable

obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra21_clocks.o
endif
obj-$(CONFIG_COMMON_CLK) += common_clock.o


obj-$(CONFIG_TEGRA3_PM) += pm-t3.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += wakeups-t21x.o
endif

obj-$(CONFIG_TEGRA_KFUSE) += tegra_kfuse.o
obj-y += apbio.o

ifeq ($(CONFIG_TEGRA_CLK_FRAMEWORK),y)

# FIXME: Bug 1211919:
# Remove CFLAGS_tegra21_dvfs.o flag when tegra21_dvfs.c becomes
# stabilized.
CFLAGS_tegra21_dvfs.o = -Werror -Wno-unused-function -Wno-unused-variable

obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra21_dvfs.o
endif

obj-$(CONFIG_TEGRA_MC_DOMAINS) += pm_domains.o
obj-y += bond_out.o

ifeq ($(CONFIG_TEGRA_SOCTHERM),y)
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra_soctherm.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC) += tegra_soctherm.o
endif
obj-$(CONFIG_TEGRA_USE_SIMON)    += tegra_simon.o
obj-$(CONFIG_ARCH_TEGRA_13x_SOC) += tegra13_edp.o

obj-$(CONFIG_TEGRA_HMP_CLUSTER_CONTROL)      += cluster-control.o

obj-$(CONFIG_TEGRA_USE_NCT)             += nct.o nct_sysfs.o

endif

obj-$(CONFIG_TEGRA_IVC)			+= tegra-ivc.o

obj-$(CONFIG_TEGRA_GPU_EDP)             += gpu-edp.o
obj-y					+= asim.o
obj-y					+= mselect.o

obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o

obj-y					+= tegra_prod.o

obj-y					+= reboot.o
obj-y					+= tegra-hv-mode.o

obj-$(CONFIG_DENVER_MCA)		+= denver_mca.o
obj-$(CONFIG_TEGRA_13x_SERROR)		+= tegra13_serr.o

obj-$(CONFIG_DENVER_CPU)		+= denver-knobs.o
obj-$(CONFIG_DENVER_DEBUG)		+= denver-hardwood.o

obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o

obj-$(CONFIG_TEGRA_FIRMWARES_CLASS)	+= firmwares.o
obj-$(CONFIG_TEGRA_FIRMWARES_INVENTORY)	+= firmwares-all.o
