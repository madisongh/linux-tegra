dts-dirs += altera
dts-dirs += amd
dts-dirs += apm
dts-dirs += arm
dts-dirs += broadcom
dts-dirs += cavium
dts-dirs += exynos
dts-dirs += freescale
dts-dirs += hisilicon
dts-dirs += marvell
dts-dirs += mediatek
dts-dirs += qcom
dts-dirs += rockchip
dts-dirs += sprd
dts-dirs += xilinx

subdir-y	:= $(dts-dirs)

dtstree		:= $(srctree)/$(src)

dtb-$(CONFIG_OF_ALL_DTBS) := $(patsubst $(dtstree)/%.dts,%.dtb, $(foreach d,$(dts-dirs), $(wildcard $(dtstree)/$(d)/*.dts)))

always		:= $(dtb-y)

targets += dtbs

DTC_FLAGS := -i $(srctree)/arch/arm/boot/dts
DTCCPP_FLAGS := -I$(srctree)/arch/arm/boot/dts

ifeq ($(CONFIG_ARCH_TEGRA_21x_SOC),y)
DTC_FLAGS += -i $(srctree)/arch/arm64/boot/dts/nvidia/t210
DTCCPP_FLAGS += -I$(srctree)/arch/arm64/boot/dts/nvidia/t210
endif

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
DTC_FLAGS += -i $(srctree)/../t18x/include
DTC_FLAGS += -i $(srctree)/../t18x/arch/arm64/boot/dts/nvidia/t18x
DTCCPP_FLAGS += -I$(srctree)/../t18x/include
DTCCPP_FLAGS += -I$(srctree)/../t18x/arch/arm64/boot/dts/nvidia/t18x
endif

ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
DTC_FLAGS += -i $(srctree)/../t19x/include
DTC_FLAGS += -i $(srctree)/../t18x/include
DTC_FLAGS += -i $(srctree)/../t19x/arch/arm64/boot/dts/nvidia/t19x
DTCCPP_FLAGS += -I$(srctree)/../t19x/include
DTCCPP_FLAGS += -I$(srctree)/../t18x/include
DTCCPP_FLAGS += -I$(srctree)/../t19x/arch/arm64/boot/dts/nvidia/t19x
endif

ifeq ($(shell test -f $(srctree)/arch/arm64/boot/dts/nvidia/t210/Makefile && echo y),y)
local-dtb := $(dtb-y)
dtb-y :=
include $(srctree)/arch/arm64/boot/dts/nvidia/t210/Makefile
t210dts-path := nvidia/t210
t210-dtb := $(addprefix nvidia/t210/,$(dtb-y))
t210-dtb-out := $(addprefix $(O)/arch/arm64/boot/dts/nvidia/t210/,$(dtb-y))
dtb-y := $(local-dtb)
dtb-y += $(t210-dtb)
endif

ifneq ($(wildcard $(kbuild-dir)/$(call tegra-path,t18x,arch/arm64/boot/dts/nvidia/t18x/Makefile)),)
local-dtb := $(dtb-y)
dtb-y :=
include $(kbuild-dir)/$(call tegra-path,t18x,arch/arm64/boot/dts/nvidia/t18x/Makefile)
t18x-dtb := $(addprefix $(call tegra-path,t18x,arch/arm64/boot/dts/nvidia/t18x/),$(dtb-y))
t18x-dtb-out := $(addprefix $(O)/arch/arm64/boot/dts/,$(t18x-dtb))
dtb-y := $(local-dtb)
dtb-y += $(t18x-dtb)
endif

ifneq ($(wildcard $(kbuild-dir)/$(call tegra-path,t19x,arch/arm64/boot/dts/nvidia/t19x/Makefile)),)
local-dtb := $(dtb-y)
dtb-y :=
include $(kbuild-dir)/$(call tegra-path,t19x,arch/arm64/boot/dts/nvidia/t19x/Makefile)
t19x-dtb := $(addprefix $(call tegra-path,t19x,arch/arm64/boot/dts/nvidia/t19x/),$(dtb-y))
t19x-dtb-out := $(addprefix $(O)/arch/arm64/boot/dts/,$(t19x-dtb))
dtb-y := $(local-dtb)
dtb-y += $(t19x-dtb)
endif

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))
ifneq ($(t210-dtb-out),)
	cp -u $(t210-dtb-out) arch/arm64/boot/dts/
endif
ifneq ($(t18x-dtb-out),)
	cp -u $(t18x-dtb-out) $(O)/arch/arm64/boot/dts/
endif
ifneq ($(t19x-dtb-out),)
	cp -u $(t19x-dtb-out) $(O)/arch/arm64/boot/dts/
endif

clean-files := dts/*.dtb *.dtb
ifneq ($(t210-dtb-out),)
	clean-files += $(t210-dtb-out)
endif
ifneq ($(t18x-dtb-out),)
	clean-files += $(t18x-dtb-out)
endif
ifneq ($(t19x-dtb-out),)
	clean-files += $(t19x-dtb-out)
endif
