#
# Makefile for the MSM specific device drivers.
#

ccflags-y += -Idrivers/misc/
ccflags-y += -Isecurity/selinux -Isecurity/selinux/include

obj-$(CONFIG_MSM_SSBI) += ssbi.o
obj-$(CONFIG_USB_BAM) += usb_bam.o
obj-$(CONFIG_IPA) += ipa/
obj-$(CONFIG_SPS) += sps/
obj-$(CONFIG_QPNP_PWM) += qpnp-pwm.o
obj-$(CONFIG_QPNP_POWER_ON) += qpnp-power-on.o
obj-$(CONFIG_QPNP_VIBRATOR) += qpnp-vibrator.o
obj-$(CONFIG_HAPTICS_2605) += drv2605.o
obj-$(CONFIG_QPNP_CLKDIV) += qpnp-clkdiv.o
obj-$(CONFIG_QPNP_COINCELL) += qpnp-coincell.o
obj-$(CONFIG_MSM_AVTIMER) += avtimer.o
obj-$(CONFIG_SSM) += ssm.o
obj-$(CONFIG_PFT) += pft.o
obj-$(CONFIG_QPNP_REVID) += qpnp-revid.o
obj-$(CONFIG_ON_SEMI_VIBRATOR) += on_semi_vibrator.o
