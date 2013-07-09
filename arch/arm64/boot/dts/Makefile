dtb-$(CONFIG_ARCH_THUNDER) += thunder-88xx.dtb
dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb

targets += dtbs
dtb-$(CONFIG_MACH_EXUMA) := tegra132-exuma.dtb
targets += $(dtb-y)

DTC_FLAGS := -i $(srctree)/arch/arm/boot/dts

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := *.dtb
