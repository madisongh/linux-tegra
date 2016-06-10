/* USB OTG (On The Go) defines */
/*
 *
 * These APIs may be used between USB controllers.  USB device drivers
 * (for either host or peripheral roles) don't use these calls; they
 * continue to use just usb_device and usb_gadget.
 */

#ifndef __LINUX_USB_OTG_H
#define __LINUX_USB_OTG_H

#include <linux/phy/phy.h>
#include <linux/usb/phy.h>
#include <linux/usb/otg-fsm.h>

struct usb_otg {
	u8			default_a;

	struct phy		*phy;
	/* old usb_phy interface */
	struct usb_phy		*usb_phy;
	struct usb_bus		*host;
	struct usb_gadget	*gadget;

	enum usb_otg_state	state;
	struct otg_fsm fsm;

	/* bind/unbind the host controller */
	int	(*set_host)(struct usb_otg *otg, struct usb_bus *host);

	/* bind/unbind the peripheral controller */
	int	(*set_peripheral)(struct usb_otg *otg,
					struct usb_gadget *gadget);

	/* effective for A-peripheral, ignored for B devices */
	int	(*set_vbus)(struct usb_otg *otg, bool enabled);

	/* for B devices only:  start session with A-Host */
	int	(*start_srp)(struct usb_otg *otg);

	/* start or continue HNP role switch */
	int	(*start_hnp)(struct usb_otg *otg);

};

/**
 * struct usb_otg_caps - describes the otg capabilities of the device
 * @otg_rev: The OTG revision number the device is compliant with, it's
 *		in binary-coded decimal (i.e. 2.0 is 0200H).
 * @hnp_support: Indicates if the device supports HNP.
 * @srp_support: Indicates if the device supports SRP.
 * @adp_support: Indicates if the device supports ADP.
 */
struct usb_otg_caps {
	u16 otg_rev;
	bool hnp_support;
	bool srp_support;
	bool adp_support;
};

extern const char *usb_otg_state_string(enum usb_otg_state state);

/* Context: can sleep */
static inline int
otg_start_hnp(struct usb_otg *otg)
{
	if (otg && otg->start_hnp)
		return otg->start_hnp(otg);

	return -ENOTSUPP;
}

/* Context: can sleep */
static inline int
otg_set_vbus(struct usb_otg *otg, bool enabled)
{
	if (otg && otg->set_vbus)
		return otg->set_vbus(otg, enabled);

	return -ENOTSUPP;
}

/* for HCDs */
static inline int
otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (otg && otg->set_host)
		return otg->set_host(otg, host);

	return -ENOTSUPP;
}

/* for usb peripheral controller drivers */

/* Context: can sleep */
static inline int
otg_set_peripheral(struct usb_otg *otg, struct usb_gadget *periph)
{
	if (otg && otg->set_peripheral)
		return otg->set_peripheral(otg, periph);

	return -ENOTSUPP;
}

static inline int
otg_start_srp(struct usb_otg *otg)
{
	if (otg && otg->start_srp)
		return otg->start_srp(otg);

	return -ENOTSUPP;
}

/* for OTG controller drivers (and maybe other stuff) */
extern int usb_bus_start_enum(struct usb_bus *bus, unsigned port_num);

enum usb_dr_mode {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
};

/**
 * usb_get_dr_mode - Get dual role mode for given device
 * @dev: Pointer to the given device
 *
 * The function gets phy interface string from property 'dr_mode',
 * and returns the correspondig enum usb_dr_mode
 */
extern enum usb_dr_mode usb_get_dr_mode(struct device *dev);

static inline int otg_chrg_vbus(struct usb_otg *otg, int on)
{
	if (!otg->fsm.ops->chrg_vbus)
		return -EOPNOTSUPP;
	otg->fsm.ops->chrg_vbus(otg, on);
	return 0;
}

static inline int otg_drv_vbus(struct usb_otg *otg, int on)
{
	if (!otg->fsm.ops->drv_vbus)
		return -EOPNOTSUPP;
	if (otg->fsm.drv_vbus != on) {
		otg->fsm.drv_vbus = on;
		otg->fsm.ops->drv_vbus(otg, on);
	}
	return 0;
}

static inline int otg_loc_conn(struct usb_otg *otg, int on)
{
	if (!otg->fsm.ops->loc_conn)
		return -EOPNOTSUPP;
	if (otg->fsm.loc_conn != on) {
		otg->fsm.loc_conn = on;
		otg->fsm.ops->loc_conn(otg, on);
	}
	return 0;
}

static inline int otg_loc_sof(struct usb_otg *otg, int on)
{
	if (!otg->fsm.ops->loc_sof)
		return -EOPNOTSUPP;
	if (otg->fsm.loc_sof != on) {
		otg->fsm.loc_sof = on;
		otg->fsm.ops->loc_sof(otg, on);
	}
	return 0;
}

static inline int otg_start_pulse(struct usb_otg *otg)
{
	if (!otg->fsm.ops->start_pulse)
		return -EOPNOTSUPP;
	if (!otg->fsm.data_pulse) {
		otg->fsm.data_pulse = 1;
		otg->fsm.ops->start_pulse(otg);
	}
	return 0;
}

static inline int otg_start_adp_prb(struct usb_otg *otg)
{
	if (!otg->fsm.ops->start_adp_prb)
		return -EOPNOTSUPP;
	if (!otg->fsm.adp_prb) {
		otg->fsm.adp_sns = 0;
		otg->fsm.adp_prb = 1;
		otg->fsm.ops->start_adp_prb(otg);
	}
	return 0;
}

static inline int otg_start_adp_sns(struct usb_otg *otg)
{
	if (!otg->fsm.ops->start_adp_sns)
		return -EOPNOTSUPP;
	if (!otg->fsm.adp_sns) {
		otg->fsm.adp_sns = 1;
		otg->fsm.ops->start_adp_sns(otg);
	}
	return 0;
}

static inline int otg_add_timer(struct usb_otg *otg, enum otg_fsm_timer timer)
{
	if (!otg->fsm.ops->add_timer)
		return -EOPNOTSUPP;
	otg->fsm.ops->add_timer(otg, timer);
	return 0;
}

static inline int otg_del_timer(struct usb_otg *otg, enum otg_fsm_timer timer)
{
	if (!otg->fsm.ops->del_timer)
		return -EOPNOTSUPP;
	otg->fsm.ops->del_timer(otg, timer);
	return 0;
}

static inline int otg_start_host(struct usb_otg *otg, int on)
{
	if (!otg->fsm.ops->start_host)
		return -EOPNOTSUPP;
	return otg->fsm.ops->start_host(otg, on);
}

static inline int otg_start_gadget(struct usb_otg *otg, int on)
{
	if (!otg->fsm.ops->start_gadget)
		return -EOPNOTSUPP;
	return otg->fsm.ops->start_gadget(otg, on);
}

#endif /* __LINUX_USB_OTG_H */
