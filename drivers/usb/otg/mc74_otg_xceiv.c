/*
 * Meraki MC74: device-only USB transceiver for the Kona dwc_otg core.
 *
 * The stock MC74 kernel used a Cisco "BCM_OTG_XCEIV" driver that was never
 * published, and the Samsung kernel's bcmpmu_otg_xceiv needs a Broadcom PMU
 * the MC74 doesn't have. The MC74 port is always a USB device, so this
 * just brings up the PHY, reports VBUS as present and lets the gadget
 * connect.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/notifier.h>
#include <linux/usb/bcm_hsotgctrl.h>

static struct otg_transceiver mc74_xceiv;
static struct platform_device *mc74_xceiv_pdev;
static struct delayed_work mc74_connect_work;

static int mc74_xceiv_init(struct otg_transceiver *otg)
{
	bcm_hsotgctrl_phy_init(true);	/* id = device */
	otg->state = OTG_STATE_B_IDLE;
	return 0;
}

static void mc74_xceiv_shutdown(struct otg_transceiver *otg)
{
	bcm_hsotgctrl_phy_deinit();
	otg->state = OTG_STATE_UNDEFINED;
}

static int mc74_xceiv_set_vbus(struct otg_transceiver *otg, bool on)
{
	return 0;	/* never a host, never drives VBUS */
}

static int mc74_xceiv_set_power(struct otg_transceiver *otg, unsigned ma)
{
	return 0;
}

static int mc74_xceiv_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	otg->host = host;
	return 0;
}

static int mc74_xceiv_pullup_on(struct otg_transceiver *otg, bool on)
{
	bcm_hsotgctrl_phy_set_non_driving(!on);
	return 0;
}

static void mc74_connect(struct work_struct *work)
{
	pr_info("mc74_otg_xceiv: VBUS present, connecting gadget\n");
	bcm_hsotgctrl_phy_set_id_stat(true);	/* ID floating = B device */
	bcm_hsotgctrl_phy_set_vbus_stat(true);
	bcm_hsotgctrl_phy_set_non_driving(false);
	atomic_notifier_call_chain(&mc74_xceiv.notifier, USB_EVENT_VBUS, NULL);
}

static int mc74_xceiv_set_peripheral(struct otg_transceiver *otg,
				     struct usb_gadget *gadget)
{
	otg->gadget = gadget;
	if (gadget)
		schedule_delayed_work(&mc74_connect_work, msecs_to_jiffies(500));
	return 0;
}

static int __init mc74_otg_xceiv_init(void)
{
	mc74_xceiv_pdev = platform_device_register_simple("mc74_otg_xceiv", -1, NULL, 0);
	if (IS_ERR(mc74_xceiv_pdev))
		return PTR_ERR(mc74_xceiv_pdev);

	INIT_DELAYED_WORK(&mc74_connect_work, mc74_connect);
	mc74_xceiv.dev = &mc74_xceiv_pdev->dev;
	mc74_xceiv.label = "mc74_otg_xceiv";
	mc74_xceiv.state = OTG_STATE_UNDEFINED;
	mc74_xceiv.init = mc74_xceiv_init;
	mc74_xceiv.shutdown = mc74_xceiv_shutdown;
	mc74_xceiv.set_vbus = mc74_xceiv_set_vbus;
	mc74_xceiv.set_power = mc74_xceiv_set_power;
	mc74_xceiv.set_host = mc74_xceiv_set_host;
	mc74_xceiv.set_peripheral = mc74_xceiv_set_peripheral;
	mc74_xceiv.pullup_on = mc74_xceiv_pullup_on;
	ATOMIC_INIT_NOTIFIER_HEAD(&mc74_xceiv.notifier);

	otg_set_transceiver(&mc74_xceiv);
	pr_info("mc74_otg_xceiv: registered (device-only)\n");
	return 0;
}
/* before dwc_otg (module_init) probes and looks up the transceiver */
fs_initcall(mc74_otg_xceiv_init);

MODULE_LICENSE("GPL");
