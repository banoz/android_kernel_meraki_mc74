/*
 * Meraki MC74 (CAPRI_ME1): stand-ins for Samsung-only symbols that the
 * I9082 kernel references unconditionally.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>

#ifndef CONFIG_SEC_DEBUG
struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static int __init mc74_sec_class_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class)) {
		pr_err("mc74: failed to create sec class\n");
		sec_class = NULL;
		return PTR_ERR(sec_class);
	}
	return 0;
}
core_initcall(mc74_sec_class_init);
#endif

#ifndef CONFIG_USB_SWITCH_FSA9485
void fsa9485_vbus_check(bool vbus_status)
{
}
EXPORT_SYMBOL(fsa9485_vbus_check);
#endif

#if !defined(CONFIG_USB_SWITCH_TSU6721) && !defined(CONFIG_USB_SWITCH_FSA9485)
void uas_jig_force_sleep(void)
{
}
#endif

/*
 * Bring-up safety net: "mc74_rebootafter=<seconds>" on the command line
 * restarts the board after that long, unless userspace writes 0 to
 * /sys/module/mc74_stubs/parameters/rebootafter first. A hung RAM test
 * then falls back to the saved bootcmd without a power cycle.
 */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>

static int rebootafter;
module_param(rebootafter, int, 0644);

static int __init mc74_rebootafter_setup(char *s)
{
	return kstrtoint(s, 10, &rebootafter) == 0;
}
__setup("mc74_rebootafter=", mc74_rebootafter_setup);

static int mc74_reboot_thread(void *unused)
{
	int left;

	while (rebootafter > 0) {
		left = rebootafter;
		msleep(1000);
		if (rebootafter <= 0)
			break;
		rebootafter = left - 1;
		if (rebootafter == 0) {
			pr_emerg("mc74: rebootafter expired, restarting\n");
			emergency_restart();
		}
	}
	return 0;
}

static int __init mc74_rebootafter_init(void)
{
	if (rebootafter > 0) {
		pr_info("mc74: will restart in %d s unless cancelled\n", rebootafter);
		kthread_run(mc74_reboot_thread, NULL, "mc74_rebootafter");
	}
	return 0;
}
late_initcall(mc74_rebootafter_init);

#ifdef CONFIG_MMC_KONA_SDIO_WIFI
/*
 * Power the BCM4339 (WL_REG_ON, GPIO 90) before the SDIO hosts probe, as the
 * stock kernel does ("Calling WLAN_INIT" before sdhci), so SDIO1 finds the
 * card at probe time; brcmfmac (backports module) then binds to it.
 */
#include <linux/gpio.h>
#include <linux/io.h>

static int __init mc74_wifi_power(void)
{
	int rc = gpio_request(90, "wl_reg_on");

	if (rc) {
		pr_err("mc74: WL_REG_ON gpio 90: %d\n", rc);
		return rc;
	}
	gpio_direction_output(90, 0);
	msleep(20);
	gpio_set_value(90, 1);
	msleep(200);		/* mmc-pwrseq post-power-on delay in the mainline DT */
	pr_info("mc74: BCM4339 powered (WL_REG_ON high)\n");
	return 0;
}
subsys_initcall(mc74_wifi_power);

/* The Kona SDIO host only "sees" the Wi-Fi card once insertion is emulated */
extern int wifi_sdio_card_emulate(int insert);

extern int capri_pm_sdio_pinmux_ctrl(unsigned int sdio_dev, int enable_pulldn);

static void mc74_wifi_detect_work(struct work_struct *w)
{
	int rc;

	/* pull-ups on SDIO1 CMD/DAT (the dt-blob leaves them pulled down,
	 * so the card's open-drain CMD line reads low: EILSEQ on CMD5) */
	capri_pm_sdio_pinmux_ctrl(0, 0);
	msleep(5);

	/*
	 * Bring the chip out of reset only while the SDIO1 host is powered and
	 * clocking with the lines pulled up (what mmc-pwrseq does in mainline);
	 * if REG_ON rises with the bus idle/pulled down the chip doesn't answer.
	 */
	{
		void __iomem *h = ioremap(0x3f180000, SZ_4K);

		if (h) {
			gpio_set_value(90, 0);
			msleep(20);
			writeb(0x0f, h + 0x29);		/* bus power, 3.3 V */
			writew(0x8001, h + 0x2c);	/* internal clock on, divider 0x80 */
			msleep(2);
			writew(0x8005, h + 0x2c);	/* SD clock on (~187 kHz) */
			msleep(5);
			gpio_set_value(90, 1);
			msleep(200);
			iounmap(h);
		}
	}
	rc = wifi_sdio_card_emulate(1);

	pr_info("mc74: Wi-Fi SDIO card insertion emulated: %d\n", rc);
}
static DECLARE_DELAYED_WORK(mc74_wifi_detect, mc74_wifi_detect_work);

static int __init mc74_wifi_detect_init(void)
{
	schedule_delayed_work(&mc74_wifi_detect, msecs_to_jiffies(200));
	return 0;
}
late_initcall(mc74_wifi_detect_init);
#endif
