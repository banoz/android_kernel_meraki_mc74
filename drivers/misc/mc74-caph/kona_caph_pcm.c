// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kona (BCM281xx "Capri") CAPH/AudioH ALSA driver for the Cisco Meraki MC74.
 *
 * A plain (non-ASoC) ALSA card: ASoC core init hangs on this board, and the
 * hardware has a fixed routing anyway.
 *
 * Playback (handsfree speaker), as in Broadcom's halaudio driver:
 *   DDR ring -> AADMAC -> CFIFO -> SSASW (trigger CAPH_IHF_THR_MET)
 *     -> AudioH IHF FIFO -> IHF DAC -> speaker
 * Capture (analog mic, AudioH VIN right channel):
 *   analog mic -> VRX PGA/ADC -> VIN FIFO R -> SSASW (CAPH_ADC_VOICE_FIFO_THR_MET)
 *     -> CFIFO -> AADMAC -> DDR ring
 *
 * Both streams are fixed at 48 kHz S16_LE (playback stereo, capture mono) with
 * exactly two periods: the AADMAC ring is a two-half buffer. The half-buffer
 * "ready" flags are polled from an hrtimer (the CAPH interrupt is not wired up),
 * each consumed/filled half is one ALSA period.
 *
 * The register helpers are Broadcom's GPL "chal" layer, vendored unchanged.
 * Diagnostics: /sys/kernel/debug/kona_caph/status
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include <chal/chal_caph.h>
#include <chal/chal_caph_dma.h>
#include <chal/chal_caph_cfifo.h>
#include <chal/chal_caph_switch.h>
#include <chal/chal_caph_intc.h>
#include <chal/chal_audio.h>

#define AUDIOH_PHYS		0x35020000
#define AUDIOH_IHF_FIFO		(AUDIOH_PHYS + 0x5000)
#define AUDIOH_VINR_FIFO	(AUDIOH_PHYS + 0x6800)
#define AUDIOH_EP_FIFO		(AUDIOH_PHYS + 0x5800)	/* VOUT: handset earpiece */
#define AUDIOH_HS_FIFO		(AUDIOH_PHYS + 0x4800)	/* stereo headset DAC */
#define CFIFO_PHYS		0x3502d000

#define RATE			48000
#define AADMA_TSIZE		4	/* AADMA_DEF_BURST_SIZE */
/* playback CFIFO: AADMA_48KHZ_STEREO_EGR_FIFO_SIZE at SRAM offset 0 */
#define PB_CFIFO_ADDR		0
#define PB_CFIFO_SIZE		1024
#define PB_CFIFO_THRES1		4	/* AADMA_DEF_EGR_THRES1 */
/* capture CFIFO: AADMA_48KHZ_MONO_IGR_FIFO_SIZE right after the playback one */
#define CAP_CFIFO_ADDR		1024
#define CAP_CFIFO_SIZE		32
#define CAP_CFIFO_THRES1	32	/* AADMA_DEF_IGR_THRES1 */
/* earpiece (mono) CFIFO after the capture one */
#define EP_CFIFO_ADDR		1056
#define EP_CFIFO_SIZE		512
#define EP_CFIFO_THRES1		4
/* headset (stereo) CFIFO after the earpiece one */
#define HS_CFIFO_ADDR		1568
#define HS_CFIFO_SIZE		1024
#define HS_CFIFO_THRES1		4
#define CFIFO_THRES2		0
#define AUDIOH_THRES1		4	/* AUDIOH_DEFAULT_THRES */
#define AUDIOH_THRES2		2	/* AUDIOH_DEFAULT_THRES2 */

#define POLL_US			1000

static int caph_trace;	/* snd_mc74_caph.caph_trace=1 logs every PCM op */
module_param(caph_trace, int, 0644);
#define CTRACE(fmt, ...) do { if (caph_trace) pr_info("mc74-caph: " fmt "\n", ##__VA_ARGS__); } while (0)

/* AudioH analog mic front end (AUDIORX) */
#define AUDIORX_VRX1		0x200
#define  VRX1_LDO_DIG_PWRDN	BIT(27)
#define  VRX1_GAIN_SHIFT	16
#define  VRX1_GAIN_MASK		(0x3f << 16)
#define  VRX1_SEL_MIC1B_MIC2	BIT(6)
#define  VRX1_APMCLK_PWRDN	BIT(2)
#define  VRX1_CMBUF_PWRDN	BIT(1)
#define  VRX1_PWRDN		BIT(0)
#define AUDIORX_VREF		0x208
#define  VREF_FASTSETTLE	BIT(1)
#define  VREF_PWRUP		BIT(0)
#define AUDIORX_VMIC		0x20c
#define  VMIC_VAUXMIC_CTRL	(3 << 4)
#define  VMIC_MIC_PWRDN		BIT(2)
#define  VMIC_MIC_EN		BIT(1)
#define AUDIOH_ADC_CTL		0x004
#define  ADC_CTL_AMIC_EN	BIT(4)
#define AUDIOH_ADCPATH_GLOBAL	0x060
/* stock MC74 values while halaudio runs: VRX1=0x00250040, VREF=0x2, VMIC=0x32 */
#define MIC_GAIN_DEFAULT	0x25
#define MIC_SEL_DEFAULT		0	/* Mic1: the handsfree mic (loud, clean speaker pickup); Mic2 is noisy */

/* KHUB CCU audio clock gates */
#define KHUB_AUDIOH_CLKGATE	0x4a0
#define KHUB_CAPH_CLKGATE	0x4b0
#define KHUB_WR_ACCESS		0x000
#define KHUB_PASSWORD		(0xa5a5 << 8)
/* stock (working) gate values: all AudioH clocks (APB/156M/26M/2P4M) sw-enabled + hyst, CAPH clocks */
#define AUDIOH_GATES		0xffff
#define CAPH_GATES		0x3030
#define AUDIOH_STPRSTS		0x00550000	/* APB|156M|26M|2P4M running */
#define KHUB_POLICY_CTL		0x00c
#define  POLICY_CTL_GO		BIT(0)
#define  POLICY_CTL_GO_ATL	BIT(2)
/*
 * Stock enables these clocks in all four policies; mainline's hub CCU driver
 * only sets bits for clocks it knows, which leaves AudioH/CAPH unable to run.
 */
static const u32 khub_mask1_regs[] = { 0x010, 0x014, 0x018, 0x01c };
static const u32 khub_mask2_regs[] = { 0x048, 0x04c, 0x050, 0x054 };
#define STOCK_POLICY_MASK1	0x07ffb6a7
#define STOCK_POLICY_MASK2	0x1f800052

struct caph_stream {
	struct snd_pcm_substream *substream;
	CAPH_DMA_CHANNEL_e dma_ch;
	CAPH_CFIFO_e cfifo_ch;
	CAPH_SWITCH_CHNL_e sw_ch;
	bool routed, running;
	unsigned int hw_ptr;		/* frames, multiple of period_size */
	unsigned long periods, idle_polls, both_ready;
};

struct caph {
	struct device *dev;
	void __iomem *audioh, *cfifo, *aadmac, *ssasw, *ahintc, *khub;
	ChalAudioCtrlBlk_t audio_blk;
	CHAL_HANDLE h_audio, h_dma, h_cfifo, h_sw, h_intc;
	spinlock_t lock;

	struct snd_card *card;
	struct caph_stream pb, cap, ep, hs;	/* speaker, mic, handset earpiece, headset */
	struct hrtimer poll;
	int mic_gain, mic_sel;
	bool spk_on, ep_on, hs_mic;
	/* headset jack: TS3A227E on I2C bus 2, polled */
	struct i2c_client *jack;
	struct switch_dev h2w;
	struct delayed_work jack_work;
	int jack_state;
	void __iomem *aci;		/* ACI/AUXMIC block: handset mic bias */
	struct dentry *dbg;
};

static void audioh_rmw(struct caph *c, u32 reg, u32 clr, u32 set)
{
	writel((readl(c->audioh + reg) & ~clr) | set, c->audioh + reg);
}

/*
 * Mainline's hub CCU driver does not know the AudioH/CAPH clocks, and nothing
 * enables them on the RAM-boot path; touching AudioH/CAPH registers while they
 * are gated stalls the bus. Program the gates exactly as stock leaves them.
 */
static const char * const caph_clk_names[] = {
	"audioh_apb_clk", "audioh_2p4m_clk", "audioh_156m_clk", "audioh_26m", "caph_srcmixer_clk",
};

static int caph_clocks_on_raw(struct caph *c);

static int caph_clocks_on(struct caph *c)
{
	int i, n = 0;
	u32 a;

	for (i = 0; i < ARRAY_SIZE(caph_clk_names); i++) {
		struct clk *k = clk_get(NULL, caph_clk_names[i]);

		if (IS_ERR(k) || clk_enable(k)) {
			dev_warn(c->dev, "clock %s unavailable\n", caph_clk_names[i]);
			continue;
		}
		n++;
	}
	udelay(100);
	a = readl(c->khub + KHUB_AUDIOH_CLKGATE);
	dev_info(c->dev, "%d/%zu audio clocks enabled, AUDIOH_CLKGATE=%08x CAPH_CLKGATE=%08x\n",
		 n, ARRAY_SIZE(caph_clk_names), a, readl(c->khub + KHUB_CAPH_CLKGATE));
	if ((a & AUDIOH_STPRSTS) == AUDIOH_STPRSTS)
		return 0;
	return caph_clocks_on_raw(c);
}

static int caph_clocks_on_raw(struct caph *c)
{
	u32 a = readl(c->khub + KHUB_AUDIOH_CLKGATE), p = readl(c->khub + KHUB_CAPH_CLKGATE);
	int i;

	writel(KHUB_PASSWORD | 1, c->khub + KHUB_WR_ACCESS);

	/* add the stock policy-mask bits, then restart the policy engine (GO | GO_ATL) */
	for (i = 0; i < 1000 && (readl(c->khub + KHUB_POLICY_CTL) & POLICY_CTL_GO); i++)
		udelay(1);
	for (i = 0; i < ARRAY_SIZE(khub_mask1_regs); i++) {
		writel(readl(c->khub + khub_mask1_regs[i]) | STOCK_POLICY_MASK1, c->khub + khub_mask1_regs[i]);
		writel(readl(c->khub + khub_mask2_regs[i]) | STOCK_POLICY_MASK2, c->khub + khub_mask2_regs[i]);
	}
	writel(POLICY_CTL_GO | POLICY_CTL_GO_ATL, c->khub + KHUB_POLICY_CTL);
	for (i = 0; i < 1000 && (readl(c->khub + KHUB_POLICY_CTL) & POLICY_CTL_GO); i++)
		udelay(1);

	writel((a & 0xffff0000) | AUDIOH_GATES, c->khub + KHUB_AUDIOH_CLKGATE);
	writel((p & 0xffff0000) | CAPH_GATES, c->khub + KHUB_CAPH_CLKGATE);
	writel(KHUB_PASSWORD | 0, c->khub + KHUB_WR_ACCESS);

	for (i = 0; i < 1000; i++) {
		a = readl(c->khub + KHUB_AUDIOH_CLKGATE);
		if ((a & AUDIOH_STPRSTS) == AUDIOH_STPRSTS)
			break;
		udelay(10);
	}
	p = readl(c->khub + KHUB_CAPH_CLKGATE);
	dev_info(c->dev, "audio clocks: AUDIOH_CLKGATE=%08x CAPH_CLKGATE=%08x (%s)\n", a, p,
		 (a & AUDIOH_STPRSTS) == AUDIOH_STPRSTS ? "running" : "NOT running");
	return (a & AUDIOH_STPRSTS) == AUDIOH_STPRSTS ? 0 : -EIO;
}

/* One-time block init (aadma_init / ssasw_init / audiohDmaInit in halaudio). */
static void caph_hw_init(struct caph *c)
{
	/* chal handles take the mapped base as a 32-bit integer (32-bit ARM) */
	c->audio_blk.audioh_base = (uint32_t)c->audioh;
	c->h_audio = &c->audio_blk;
	c->h_dma = chal_caph_dma_init((uint32_t)c->aadmac);
	c->h_cfifo = chal_caph_cfifo_init((uint32_t)c->cfifo);
	c->h_sw = chal_caph_switch_init((uint32_t)c->ssasw);
	c->h_intc = chal_caph_intc_init((uint32_t)c->ahintc);

	chal_caph_intc_reset(c->h_intc);
	chal_caph_intc_set_cfifo_int_detect_level(c->h_intc, 0xffff, 0x0000);

	chal_caph_switch_enable_clock_bypass(c->h_sw);
	chal_caph_switch_enable_clock(c->h_sw);

	/* IHF: stereo, unpacked, 16 bit */
	chal_audio_ihfpath_dma_enable(c->h_audio, CHAL_AUDIO_ENABLE);
	chal_audio_ihfpath_set_fifo_thres(c->h_audio, AUDIOH_THRES1, AUDIOH_THRES2);
	chal_audio_ihfpath_set_mono_stereo(c->h_audio, 0);
	chal_audio_ihfpath_set_pack(c->h_audio, 0);
	chal_audio_ihfpath_set_bits_per_sample(c->h_audio, 16);
	chal_audio_ihfpath_clr_fifo(c->h_audio);

	/* headset: stereo, unpacked, 16 bit */
	chal_audio_hspath_dma_enable(c->h_audio, CHAL_AUDIO_ENABLE);
	chal_audio_hspath_set_fifo_thres(c->h_audio, 2, 0);
	chal_audio_hspath_set_mono_stereo(c->h_audio, 0);
	chal_audio_hspath_set_pack(c->h_audio, 0);
	chal_audio_hspath_set_bits_per_sample(c->h_audio, 16);
	chal_audio_hspath_clr_fifo(c->h_audio);

	/* earpiece (VOUT): mono, unpacked, 16 bit */
	chal_audio_earpath_dma_enable(c->h_audio, CHAL_AUDIO_ENABLE);
	chal_audio_earpath_set_fifo_thres(c->h_audio, 2, 0);
	chal_audio_earpath_set_pack(c->h_audio, 0);
	chal_audio_earpath_set_bits_per_sample(c->h_audio, 16);
	chal_audio_earpath_clr_fifo(c->h_audio);

	/* VIN right: mono, unpacked, 16 bit */
	chal_audio_vinpath_dma_enable(c->h_audio, CHAL_AUDIO_ENABLE);
	chal_audio_vinpath_set_fifo_thres(c->h_audio, AUDIOH_THRES1, AUDIOH_THRES2);
	chal_audio_vinpath_set_mono_stereo(c->h_audio, 1);
	chal_audio_vinpath_set_pack(c->h_audio, 0);
	chal_audio_vinpath_set_bits_per_sample(c->h_audio, 16);
	chal_audio_vinpath_clr_fifo(c->h_audio);
}

static void mic_apply_gain_sel(struct caph *c)
{
	audioh_rmw(c, AUDIORX_VRX1, VRX1_GAIN_MASK | VRX1_SEL_MIC1B_MIC2,
		   (c->mic_gain << VRX1_GAIN_SHIFT) | (c->mic_sel ? VRX1_SEL_MIC1B_MIC2 : 0));
}

/* chal_audio_mic_pwrctrl() + select_primary_mic(analog), minus the ACI block */
static void mic_power(struct caph *c, bool on)
{
	if (on) {
		audioh_rmw(c, AUDIORX_VREF, 0, VREF_PWRUP | VREF_FASTSETTLE);
		audioh_rmw(c, AUDIORX_VRX1, VRX1_PWRDN | VRX1_CMBUF_PWRDN | VRX1_APMCLK_PWRDN |
			   VRX1_LDO_DIG_PWRDN, 0);
		mic_apply_gain_sel(c);
		audioh_rmw(c, AUDIORX_VMIC, VMIC_MIC_PWRDN | VMIC_VAUXMIC_CTRL, VMIC_MIC_EN | VMIC_VAUXMIC_CTRL);
		writel(readl(c->audioh + AUDIORX_VREF) & VREF_FASTSETTLE, c->audioh + AUDIORX_VREF);

		writel(BIT(8), c->audioh + AUDIOH_ADCPATH_GLOBAL);
		writel(0, c->audioh + AUDIOH_ADCPATH_GLOBAL);
		audioh_rmw(c, AUDIOH_ADC_CTL, 0, ADC_CTL_AMIC_EN);
		writel(1, c->audioh + AUDIOH_ADCPATH_GLOBAL);
	} else {
		audioh_rmw(c, AUDIOH_ADC_CTL, ADC_CTL_AMIC_EN, 0);
		audioh_rmw(c, AUDIORX_VMIC, VMIC_MIC_EN, VMIC_MIC_PWRDN | VMIC_VAUXMIC_CTRL);
		audioh_rmw(c, AUDIORX_VRX1, 0, VRX1_PWRDN | VRX1_CMBUF_PWRDN | VRX1_APMCLK_PWRDN |
			   VRX1_LDO_DIG_PWRDN);
		audioh_rmw(c, AUDIORX_VREF, VREF_PWRUP | VREF_FASTSETTLE, 0);
	}
}

static bool is_playback(struct caph *c, struct caph_stream *s)
{
	return s == &c->pb || s == &c->ep || s == &c->hs;
}

static bool is_hs(struct caph *c, struct caph_stream *s)
{
	return s == &c->hs;
}

static bool is_ep(struct caph *c, struct caph_stream *s)
{
	return s == &c->ep;
}

/* aadma_request_channel + ssasw_request_channel for one direction */
static int stream_route(struct caph *c, struct caph_stream *s, dma_addr_t buf, size_t bytes)
{
	bool pb = is_playback(c, s);
	CAPH_CFIFO_CHNL_DIRECTION_e dir = pb ? CAPH_CFIFO_IN : CAPH_CFIFO_OUT;
	u16 cfifo_addr;

	s->dma_ch = chal_caph_dma_alloc_given_channel(c->h_dma, CAPH_DMA_CH_VOID);
	s->cfifo_ch = chal_caph_cfifo_alloc_given_channel(c->h_cfifo, CAPH_CFIFO_VOID);
	s->sw_ch = chal_caph_switch_alloc_given_channel(c->h_sw, CAPH_SWITCH_CH_VOID);
	if (s->dma_ch == CAPH_DMA_CH_VOID || s->cfifo_ch == CAPH_CFIFO_VOID ||
	    s->sw_ch == CAPH_SWITCH_CH_VOID) {
		dev_err(c->dev, "no free CAPH channel (dma %x cfifo %x sw %x)\n",
			s->dma_ch, s->cfifo_ch, s->sw_ch);
		if (s->dma_ch != CAPH_DMA_CH_VOID)
			chal_caph_dma_free_channel(c->h_dma, s->dma_ch);
		if (s->cfifo_ch != CAPH_CFIFO_VOID)
			chal_caph_cfifo_free_channel(c->h_cfifo, s->cfifo_ch);
		if (s->sw_ch != CAPH_SWITCH_CH_VOID)
			chal_caph_switch_free_channel(c->h_sw, s->sw_ch);
		return -EBUSY;
	}

	chal_caph_dma_set_tsize(c->h_dma, s->dma_ch, AADMA_TSIZE);
	chal_caph_dma_set_cfifo(c->h_dma, s->dma_ch, s->cfifo_ch);
	chal_caph_cfifo_set_address(c->h_cfifo, s->cfifo_ch,
				    is_hs(c, s) ? HS_CFIFO_ADDR : is_ep(c, s) ? EP_CFIFO_ADDR :
				    pb ? PB_CFIFO_ADDR : CAP_CFIFO_ADDR);
	chal_caph_dma_set_direction(c->h_dma, s->dma_ch, dir);
	chal_caph_cfifo_set_direction(c->h_cfifo, s->cfifo_ch, dir);
	chal_caph_cfifo_set_size(c->h_cfifo, s->cfifo_ch,
				 is_hs(c, s) ? HS_CFIFO_SIZE : is_ep(c, s) ? EP_CFIFO_SIZE :
				 pb ? PB_CFIFO_SIZE : CAP_CFIFO_SIZE);
	chal_caph_cfifo_set_fifo_thres(c->h_cfifo, s->cfifo_ch,
				       is_hs(c, s) ? HS_CFIFO_THRES1 : is_ep(c, s) ? EP_CFIFO_THRES1 :
				       pb ? PB_CFIFO_THRES1 : CAP_CFIFO_THRES1,
				       CFIFO_THRES2);
	chal_caph_dma_clr_channel_fifo(c->h_dma, s->dma_ch);
	chal_caph_cfifo_clr_fifo(c->h_cfifo, s->cfifo_ch);

	cfifo_addr = (u16)(CFIFO_PHYS + chal_caph_cfifo_get_fifo_addr_offset(c->h_cfifo, s->cfifo_ch));
	chal_caph_switch_set_datafmt(c->h_sw, s->sw_ch, CAPH_MONO_16BIT);
	if (is_hs(c, s)) {
		chal_caph_switch_select_trigger(c->h_sw, s->sw_ch, CAPH_HS_THR_MET);
		chal_caph_switch_select_src(c->h_sw, s->sw_ch, cfifo_addr);
		chal_caph_switch_add_dst(c->h_sw, s->sw_ch, (u16)AUDIOH_HS_FIFO);
	} else if (is_ep(c, s)) {
		chal_caph_switch_select_trigger(c->h_sw, s->sw_ch, CAPH_EP_THR_MET);
		chal_caph_switch_select_src(c->h_sw, s->sw_ch, cfifo_addr);
		chal_caph_switch_add_dst(c->h_sw, s->sw_ch, (u16)AUDIOH_EP_FIFO);
	} else if (pb) {
		chal_caph_switch_select_trigger(c->h_sw, s->sw_ch, CAPH_IHF_THR_MET);
		chal_caph_switch_select_src(c->h_sw, s->sw_ch, cfifo_addr);
		chal_caph_switch_add_dst(c->h_sw, s->sw_ch, (u16)AUDIOH_IHF_FIFO);
	} else {
		chal_caph_switch_select_trigger(c->h_sw, s->sw_ch, CAPH_ADC_VOICE_FIFO_THR_MET);
		chal_caph_switch_select_src(c->h_sw, s->sw_ch, (u16)AUDIOH_VINR_FIFO);
		chal_caph_switch_add_dst(c->h_sw, s->sw_ch, cfifo_addr);
	}

	chal_caph_dma_set_buffer(c->h_dma, s->dma_ch, buf, bytes);
	chal_caph_dma_set_hibuffer(c->h_dma, s->dma_ch, buf + bytes / 2);

	s->routed = true;
	dev_dbg(c->dev, "%s routed: dma %#x cfifo %#x (%#x) sw %#x, ring %zu bytes\n",
		pb ? "playback" : "capture", s->dma_ch, s->cfifo_ch, cfifo_addr, s->sw_ch, bytes);
	return 0;
}

static void stream_unroute(struct caph *c, struct caph_stream *s)
{
	if (!s->routed)
		return;
	chal_caph_switch_clear_all_dst(c->h_sw, s->sw_ch);
	chal_caph_switch_free_channel(c->h_sw, s->sw_ch);
	chal_caph_dma_free_channel(c->h_dma, s->dma_ch);
	chal_caph_cfifo_free_channel(c->h_cfifo, s->cfifo_ch);
	s->routed = false;
}

/* called with c->lock held */
static void stream_start(struct caph *c, struct caph_stream *s)
{
	bool pb = is_playback(c, s);

	s->hw_ptr = 0;
	chal_caph_dma_clr_channel_fifo(c->h_dma, s->dma_ch);
	chal_caph_cfifo_clr_fifo(c->h_cfifo, s->cfifo_ch);
	chal_caph_dma_set_ddrfifo_status(c->h_dma, s->dma_ch, CAPH_READY_HIGHLOW);

	chal_caph_switch_enable(c->h_sw, s->sw_ch);
	chal_caph_cfifo_enable(c->h_cfifo, s->cfifo_ch);
	chal_caph_dma_enable(c->h_dma, s->dma_ch);

	if (is_hs(c, s)) {
		chal_audio_hspath_clr_fifo(c->h_audio);
		chal_audio_hspath_set_dac_pwr(c->h_audio, CHAL_AUDIO_CHANNEL_LEFT | CHAL_AUDIO_CHANNEL_RIGHT);
		chal_audio_hspath_set_gain(c->h_audio, 0);
		chal_audio_hspath_mute(c->h_audio, 0);
		chal_audio_hspath_enable(c->h_audio, CHAL_AUDIO_CHANNEL_LEFT | CHAL_AUDIO_CHANNEL_RIGHT);
	} else if (is_ep(c, s)) {
		/* csl_caph_audioh start + csl_caph_audioh_start_ep */
		chal_audio_earpath_clr_fifo(c->h_audio);
		chal_audio_earpath_mute(c->h_audio, 1);
		chal_audio_earpath_clear_isolation_ctrl(c->h_audio, CHAL_AUDIO_AUDIOTX_ISO_IN);
		chal_audio_earpath_set_dac_pwr(c->h_audio, CHAL_AUDIO_ENABLE);
		chal_audio_earpath_set_gain(c->h_audio, 0);
		chal_audio_earpath_set_drv_pwr(c->h_audio, CHAL_AUDIO_ENABLE);
		chal_audio_earpath_enable(c->h_audio, CHAL_AUDIO_ENABLE);
		chal_audio_earpath_set_slowramp_ctrl(c->h_audio, CHAL_AUDIO_AUDIOTX_SR_PUP_ED_DRV_TRIG);
		chal_audio_earpath_mute(c->h_audio, !c->ep_on);
	} else if (pb) {
		chal_audio_ihfpath_clr_fifo(c->h_audio);
		chal_audio_ihfpath_mute(c->h_audio, !c->spk_on);
		chal_audio_ihfpath_set_dac_pwr(c->h_audio, CHAL_AUDIO_CHANNEL_LEFT | CHAL_AUDIO_CHANNEL_RIGHT);
		chal_audio_ihfpath_enable(c->h_audio, CHAL_AUDIO_CHANNEL_LEFT | CHAL_AUDIO_CHANNEL_RIGHT);
	} else {
		chal_audio_vinpath_clr_fifo(c->h_audio);
	}
	s->running = true;
}

/* called with c->lock held */
static void stream_stop(struct caph *c, struct caph_stream *s)
{
	if (!s->running)
		return;
	s->running = false;
	if (is_hs(c, s)) {
		chal_audio_hspath_mute(c->h_audio, 1);
		chal_audio_hspath_enable(c->h_audio, 0);
		chal_audio_hspath_set_dac_pwr(c->h_audio, 0);
	} else if (is_ep(c, s)) {
		chal_audio_earpath_mute(c->h_audio, 1);
		chal_audio_earpath_set_drv_pwr(c->h_audio, 0);
		chal_audio_earpath_set_dac_pwr(c->h_audio, 0);
		chal_audio_earpath_enable(c->h_audio, 0);
		chal_audio_earpath_clear_slowramp_ctrl(c->h_audio, CHAL_AUDIO_AUDIOTX_SR_PUP_ED_DRV_TRIG);
		/* the EP DAC power-down also powers down the IHF DACs */
		if (c->pb.running)
			chal_audio_ihfpath_set_dac_pwr(c->h_audio, CHAL_AUDIO_CHANNEL_LEFT | CHAL_AUDIO_CHANNEL_RIGHT);
	} else if (is_playback(c, s)) {
		chal_audio_ihfpath_enable(c->h_audio, 0);
		chal_audio_ihfpath_set_dac_pwr(c->h_audio, 0);
	}
	chal_caph_dma_disable(c->h_dma, s->dma_ch);
	chal_caph_cfifo_disable(c->h_cfifo, s->cfifo_ch);
	chal_caph_switch_disable(c->h_sw, s->sw_ch);
}

/* returns true if a period boundary was crossed */
static bool stream_poll(struct caph *c, struct caph_stream *s)
{
	struct snd_pcm_runtime *rt;
	CAPH_DMA_CHNL_FIFO_STATUS_e st;
	int n;

	if (!s->running)
		return false;
	rt = s->substream->runtime;

	/*
	 * The HW_READY flags mark halves the DMA has finished with (played out, or
	 * filled by capture). Ack them and hand the halves straight back by
	 * setting SW_READY again, as halaudio does; the application keeps up via
	 * the ALSA pointer.
	 */
	st = chal_caph_dma_read_ddrfifo_status(c->h_dma, s->dma_ch) & CAPH_READY_HIGHLOW;
	n = hweight8(st);
	if (!n) {
		s->idle_polls++;
		return false;
	}
	if (n == 2)
		s->both_ready++;
	chal_caph_dma_clr_ddrfifo_status(c->h_dma, s->dma_ch, st);
	chal_caph_dma_set_ddrfifo_status(c->h_dma, s->dma_ch, CAPH_READY_HIGHLOW);
	s->periods += n;
	s->hw_ptr = (s->hw_ptr + n * rt->period_size) % rt->buffer_size;
	return true;
}

static enum hrtimer_restart caph_poll(struct hrtimer *t)
{
	struct caph *c = container_of(t, struct caph, poll);
	bool pb_el, cap_el, ep_el, hs_el, any;
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	pb_el = stream_poll(c, &c->pb);
	cap_el = stream_poll(c, &c->cap);
	ep_el = stream_poll(c, &c->ep);
	hs_el = stream_poll(c, &c->hs);
	any = c->pb.running || c->cap.running || c->ep.running || c->hs.running;
	spin_unlock_irqrestore(&c->lock, flags);

	if (pb_el)
		snd_pcm_period_elapsed(c->pb.substream);
	if (cap_el)
		snd_pcm_period_elapsed(c->cap.substream);
	if (ep_el)
		snd_pcm_period_elapsed(c->ep.substream);
	if (hs_el)
		snd_pcm_period_elapsed(c->hs.substream);
	if (!any)
		return HRTIMER_NORESTART;
	hrtimer_forward_now(t, ns_to_ktime(POLL_US * 1000));
	return HRTIMER_RESTART;
}

/* ---- ALSA PCM ---- */

static const struct snd_pcm_hardware caph_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = RATE,
	.rate_max = RATE,
	.buffer_bytes_max = 65536,	/* AADMAC wrap size limit for normal channels */
	/* the ring is polled every jiffy-ish (HZ=100): keep periods >= 20 ms */
	.period_bytes_min = 3840,	/* 40 ms mono / 20 ms stereo */
	.period_bytes_max = 32768,
	.periods_min = 2,
	.periods_max = 2,
};

static struct caph_stream *sub_stream(struct caph *c, struct snd_pcm_substream *ss)
{
	if (ss->pcm->device == 1)
		return &c->ep;
	if (ss->pcm->device == 2)
		return &c->hs;
	return ss->stream == SNDRV_PCM_STREAM_PLAYBACK ? &c->pb : &c->cap;
}

static int caph_pcm_open(struct snd_pcm_substream *ss)
{
	struct caph *c = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;
	int ret;
	CTRACE("%s %s", "caph_pcm_open", ss->stream ? "cap" : "pb");

	rt->hw = caph_pcm_hw;
	rt->hw.channels_min = rt->hw.channels_max =
		(ss->stream == SNDRV_PCM_STREAM_PLAYBACK && ss->pcm->device != 1) ? 2 : 1;
	ret = snd_pcm_hw_constraint_step(rt, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 64);
	if (ret < 0)
		return ret;
	sub_stream(c, ss)->substream = ss;
	if (ss->stream == SNDRV_PCM_STREAM_CAPTURE)
		mic_power(c, true);
	return 0;
}

static int caph_pcm_close(struct snd_pcm_substream *ss)
{
	struct caph *c = snd_pcm_substream_chip(ss);
	CTRACE("%s %s", "caph_pcm_close", ss->stream ? "cap" : "pb");

	if (ss->stream == SNDRV_PCM_STREAM_CAPTURE)
		mic_power(c, false);
	return 0;
}

static int caph_pcm_prepare(struct snd_pcm_substream *ss)
{
	struct caph *c = snd_pcm_substream_chip(ss);
	struct caph_stream *s = sub_stream(c, ss);
	struct snd_pcm_runtime *rt = ss->runtime;
	CTRACE("%s %s", "caph_pcm_prepare", ss->stream ? "cap" : "pb");

	stream_unroute(c, s);
	if (ss->stream == SNDRV_PCM_STREAM_PLAYBACK)
		memset(rt->dma_area, 0, snd_pcm_lib_buffer_bytes(ss));
	return stream_route(c, s, rt->dma_addr, snd_pcm_lib_buffer_bytes(ss));
}

static int caph_pcm_hw_params(struct snd_pcm_substream *ss, struct snd_pcm_hw_params *hw)
{
	CTRACE("%s %s %u", "caph_pcm_hw_params", ss->stream ? "cap" : "pb", params_buffer_bytes(hw));
	return snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(hw));
}

static int caph_pcm_hw_free(struct snd_pcm_substream *ss)
{
	struct caph *c = snd_pcm_substream_chip(ss);
	CTRACE("%s %s", "caph_pcm_hw_free", ss->stream ? "cap" : "pb");

	stream_unroute(c, sub_stream(c, ss));
	return snd_pcm_lib_free_pages(ss);
}

static int caph_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct caph *c = snd_pcm_substream_chip(ss);
	struct caph_stream *s = sub_stream(c, ss);
	unsigned long flags;
	bool start_timer = false;
	CTRACE("%s %s cmd %d", "caph_pcm_trigger", ss->stream ? "cap" : "pb", cmd);

	spin_lock_irqsave(&c->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		start_timer = !c->pb.running && !c->cap.running && !c->ep.running && !c->hs.running;
		stream_start(c, s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		stream_stop(c, s);
		break;
	default:
		spin_unlock_irqrestore(&c->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&c->lock, flags);
	if (start_timer)
		hrtimer_start(&c->poll, ns_to_ktime(POLL_US * 1000), HRTIMER_MODE_REL);
	return 0;
}

static snd_pcm_uframes_t caph_pcm_pointer(struct snd_pcm_substream *ss)
{
	struct caph *c = snd_pcm_substream_chip(ss);

	return ACCESS_ONCE(sub_stream(c, ss)->hw_ptr);
}

static struct snd_pcm_ops caph_pcm_ops = {
	.open = caph_pcm_open,
	.close = caph_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = caph_pcm_hw_params,
	.prepare = caph_pcm_prepare,
	.hw_free = caph_pcm_hw_free,
	.trigger = caph_pcm_trigger,
	.pointer = caph_pcm_pointer,
};

/* ---- mixer controls ---- */

static const char * const mic_sel_texts[] = { "Mic1", "Mic2" };

static int mic_sel_info(struct snd_kcontrol *k, struct snd_ctl_elem_info *ui)
{
	return snd_ctl_enum_info(ui, 1, ARRAY_SIZE(mic_sel_texts), mic_sel_texts);
}

static int mic_sel_get(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);

	uv->value.enumerated.item[0] = c->mic_sel;
	return 0;
}

static int mic_sel_put(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);
	int v = uv->value.enumerated.item[0];
	CTRACE("mic_sel_put");

	if (v >= ARRAY_SIZE(mic_sel_texts))
		return -EINVAL;
	if (v == c->mic_sel)
		return 0;
	c->mic_sel = v;
	mic_apply_gain_sel(c);
	return 1;
}

static int mic_gain_info(struct snd_kcontrol *k, struct snd_ctl_elem_info *ui)
{
	ui->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	ui->count = 1;
	ui->value.integer.min = 0;
	ui->value.integer.max = 0x3f;
	return 0;
}

static int mic_gain_get(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);

	uv->value.integer.value[0] = c->mic_gain;
	return 0;
}

static int mic_gain_put(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);
	long v = uv->value.integer.value[0];
	CTRACE("mic_gain_put");

	if (v < 0 || v > 0x3f)
		return -EINVAL;
	if (v == c->mic_gain)
		return 0;
	c->mic_gain = v;
	mic_apply_gain_sel(c);
	return 1;
}

static int spk_sw_get(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);

	uv->value.integer.value[0] = c->spk_on;
	return 0;
}

static int spk_sw_put(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);
	bool on = !!uv->value.integer.value[0];
	unsigned long flags;
	CTRACE("spk_sw_put");

	if (on == c->spk_on)
		return 0;
	spin_lock_irqsave(&c->lock, flags);
	c->spk_on = on;
	/* the IHF path keeps running (it paces the DMA); the switch only mutes the DAC */
	chal_audio_ihfpath_mute(c->h_audio, !on);
	spin_unlock_irqrestore(&c->lock, flags);
	return 1;
}

static int ep_sw_get(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);

	uv->value.integer.value[0] = c->ep_on;
	return 0;
}

static int ep_sw_put(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);
	bool on = !!uv->value.integer.value[0];
	unsigned long flags;

	if (on == c->ep_on)
		return 0;
	spin_lock_irqsave(&c->lock, flags);
	c->ep_on = on;
	if (c->ep.running)
		chal_audio_earpath_mute(c->h_audio, !on);
	spin_unlock_irqrestore(&c->lock, flags);
	return 1;
}

/*
 * Handset mic (stock "Handset/Headset Mic Switch"): GPIO 5 switches the
 * Mic1 input from the handsfree mic to the handset mic, which is biased
 * from the ACI/AUXMIC block (chal_audio_enable_aci_auxmic).
 */
#define MC74_HS_MIC_GPIO	5
#define ACI_ADC_CTRL		0xd4
#define  ACI_AUDIORX_VREF_BIAS	0xc0
#define AUXMIC_CMC		0x0c
#define AUXMIC_AUXEN		0x14
#define AUXMIC_F_PWRDWN		0x28

static void hs_mic_set(struct caph *c, bool on)
{
	void __iomem *a = c->aci;

	if (!a)
		return;
	if (on) {
		writel(readl(a + ACI_ADC_CTRL) | ACI_AUDIORX_VREF_BIAS, a + ACI_ADC_CTRL);
		writel(readl(a + AUXMIC_AUXEN) | 1, a + AUXMIC_AUXEN);
		writel(readl(a + AUXMIC_CMC) | 1, a + AUXMIC_CMC);
		writel(readl(a + AUXMIC_F_PWRDWN) & ~1, a + AUXMIC_F_PWRDWN);
	} else {
		writel(readl(a + AUXMIC_F_PWRDWN) | 1, a + AUXMIC_F_PWRDWN);
		writel(readl(a + AUXMIC_CMC) & ~1, a + AUXMIC_CMC);
		writel(readl(a + AUXMIC_AUXEN) & ~1, a + AUXMIC_AUXEN);
	}
	gpio_set_value(MC74_HS_MIC_GPIO, on);
}

static int hs_mic_get(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);

	uv->value.integer.value[0] = c->hs_mic;
	return 0;
}

static int hs_mic_put(struct snd_kcontrol *k, struct snd_ctl_elem_value *uv)
{
	struct caph *c = snd_kcontrol_chip(k);
	bool on = !!uv->value.integer.value[0];

	if (on == c->hs_mic)
		return 0;
	c->hs_mic = on;
	hs_mic_set(c, on);
	return 1;
}

static const struct snd_kcontrol_new caph_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Handset Mic Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = hs_mic_get,
		.put = hs_mic_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Handset Playback Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = ep_sw_get,
		.put = ep_sw_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Speaker Playback Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = spk_sw_get,
		.put = spk_sw_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic Capture Volume",
		.info = mic_gain_info,
		.get = mic_gain_get,
		.put = mic_gain_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic Capture Source",
		.info = mic_sel_info,
		.get = mic_sel_get,
		.put = mic_sel_put,
	},
};

/* ---- debugfs ---- */

static void show_stream(struct seq_file *m, struct caph *c, const char *name, struct caph_stream *s)
{
	seq_printf(m, "%s: routed %d running %d hw_ptr %u periods %lu both_ready %lu idle_polls %lu\n",
		   name, s->routed, s->running, s->hw_ptr, s->periods, s->both_ready, s->idle_polls);
	if (s->routed)
		seq_printf(m, "  dma ch %#x ddrfifo %x curr_mem_ptr %#x\n", s->dma_ch,
			   chal_caph_dma_read_ddrfifo_status(c->h_dma, s->dma_ch),
			   chal_caph_dma_read_currmempointer(c->h_dma, s->dma_ch));
}

static int status_show(struct seq_file *m, void *unused)
{
	struct caph *c = m->private;

	show_stream(m, c, "playback", &c->pb);
	show_stream(m, c, "capture", &c->cap);
	seq_printf(m, "ihf fifo %08x vinr fifo %08x\n",
		   chal_audio_ihfpath_read_fifo_status(c->h_audio),
		   chal_audio_vinpath_read_fifo_status(c->h_audio));
	seq_printf(m, "ADC_CTL %08x VRX1 %08x VREF %08x VMIC %08x VIN_FIFO_CTRL %08x DMA_CTL %08x\n",
		   readl(c->audioh + AUDIOH_ADC_CTL), readl(c->audioh + AUDIORX_VRX1),
		   readl(c->audioh + AUDIORX_VREF), readl(c->audioh + AUDIORX_VMIC),
		   readl(c->audioh + 0x34), readl(c->audioh + 0x08));
	seq_printf(m, "AUDIOH_CLKGATE %08x CAPH_CLKGATE %08x\n",
		   readl(c->khub + KHUB_AUDIOH_CLKGATE), readl(c->khub + KHUB_CAPH_CLKGATE));
	return 0;
}
static int status_open(struct inode *inode, struct file *file)
{
	return single_open(file, status_show, inode->i_private);
}

static const struct file_operations status_fops = {
	.open = status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- headset jack (TS3A227E) ---- */

#define TS3A227E_ADDR		0x3b
#define TS3A227E_INTERRUPT	0x01
#define TS3A227E_ACC_STATUS	0x0b
#define JACK_POLL_MS		300

/*
 * Accessory status as seen on the MC74: 0 = nothing, 0x0c = 4-pole headset
 * with mic. Anything else plugged is reported as headphones. h2w states
 * are what Android's WiredAccessoryManager expects: 1 headset, 2 headphone.
 */
static void jack_work_fn(struct work_struct *w)
{
	struct caph *c = container_of(w, struct caph, jack_work.work);
	int st = i2c_smbus_read_byte_data(c->jack, TS3A227E_ACC_STATUS);
	int state;

	if (st >= 0) {
		(void)i2c_smbus_read_byte_data(c->jack, TS3A227E_INTERRUPT);	/* clear */
		state = !st ? 0 : (st & 0x0c) == 0x0c ? 1 : 2;
		if (state != c->jack_state) {
			c->jack_state = state;
			switch_set_state(&c->h2w, state);
			dev_info(c->dev, "jack: %s (status %#x)\n",
				 state == 1 ? "headset" : state == 2 ? "headphones" : "unplugged", st);
		}
	}
	schedule_delayed_work(&c->jack_work, msecs_to_jiffies(JACK_POLL_MS));
}

static void jack_init(struct caph *c)
{
	struct i2c_adapter *adap = i2c_get_adapter(2);

	if (!adap)
		return;
	c->jack = i2c_new_dummy(adap, TS3A227E_ADDR);
	i2c_put_adapter(adap);
	if (!c->jack)
		return;
	c->h2w.name = "h2w";
	if (switch_dev_register(&c->h2w)) {
		i2c_unregister_device(c->jack);
		c->jack = NULL;
		return;
	}
	c->jack_state = -1;
	INIT_DELAYED_WORK(&c->jack_work, jack_work_fn);
	schedule_delayed_work(&c->jack_work, msecs_to_jiffies(JACK_POLL_MS));
}

/* ---- probe ---- */

static void __iomem *map(phys_addr_t pa, size_t len)
{
	return ioremap(pa, len);
}

static int caph_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct caph *c;
	int ret, i;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->dev = dev;
	spin_lock_init(&c->lock);
	c->mic_gain = MIC_GAIN_DEFAULT;
	c->mic_sel = MIC_SEL_DEFAULT;
	c->spk_on = true;
	c->ep_on = true;

	c->audioh = map(0x35020000, 0x6000);
	c->cfifo = map(0x3502d000, 0x1000);
	c->aadmac = map(0x3502e000, 0x1000);
	c->ssasw = map(0x3502f000, 0x800);
	c->ahintc = map(0x3502f800, 0x800);
	c->khub = map(0x34000000, 0x1000);
	if (!c->audioh || !c->cfifo || !c->aadmac || !c->ssasw || !c->ahintc || !c->khub)
		return -ENOMEM;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);

	/* clocks first: any AudioH/CAPH access with the clocks gated stalls the bus */
	ret = caph_clocks_on(c);
	if (ret)
		return ret;
	caph_hw_init(c);
	/*
	 * Handset mic switch + bias block. The ACI block stalls the bus when
	 * its APB clock is off, so only map it once the clock is enabled.
	 */
	if (!gpio_request(MC74_HS_MIC_GPIO, "handset-mic")) {
		struct clk *aci_clk = clk_get(NULL, "aci_apb_clk");

		gpio_direction_output(MC74_HS_MIC_GPIO, 0);
		if (!IS_ERR(aci_clk) && !clk_enable(aci_clk))
			c->aci = map(0x3500e000, 0x1000);
		else
			dev_warn(dev, "aci_apb_clk unavailable: no handset mic bias\n");
	}
	hrtimer_init(&c->poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	c->poll.function = caph_poll;

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, "MC74", THIS_MODULE, 0, &card);
	if (ret)
		return ret;
	snd_card_set_dev(card, dev);
	c->card = card;
	strlcpy(card->driver, "mc74-caph", sizeof(card->driver));
	strlcpy(card->shortname, "MC74", sizeof(card->shortname));
	strlcpy(card->longname, "Cisco Meraki MC74 (Kona CAPH/AudioH)", sizeof(card->longname));

	ret = snd_pcm_new(card, "CAPH", 0, 1, 1, &pcm);
	if (ret)
		goto err;
	pcm->private_data = c;
	strlcpy(pcm->name, "MC74 speaker/mic", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &caph_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &caph_pcm_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, dev, 65536, 65536);

	/* device 2: headset, stereo playback */
	ret = snd_pcm_new(card, "CAPH-HS", 2, 1, 0, &pcm);
	if (ret)
		goto err;
	pcm->private_data = c;
	strlcpy(pcm->name, "MC74 headset", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &caph_pcm_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, dev, 65536, 65536);

	/* device 1: handset earpiece, mono playback */
	ret = snd_pcm_new(card, "CAPH-EP", 1, 1, 0, &pcm);
	if (ret)
		goto err;
	pcm->private_data = c;
	strlcpy(pcm->name, "MC74 handset", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &caph_pcm_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, dev, 65536, 65536);

	for (i = 0; i < ARRAY_SIZE(caph_controls); i++) {
		ret = snd_ctl_add(card, snd_ctl_new1(&caph_controls[i], c));
		if (ret)
			goto err;
	}

	ret = snd_card_register(card);
	if (ret)
		goto err;

	c->dbg = debugfs_create_dir("mc74_caph", NULL);
	debugfs_create_file("status", 0444, c->dbg, c, &status_fops);
	platform_set_drvdata(pdev, c);
	jack_init(c);
	dev_info(dev, "MC74 sound card registered (speaker playback, mic capture)\n");
	return 0;
err:
	snd_card_free(card);
	return ret;
}

static struct platform_driver caph_driver = {
	.probe = caph_probe,
	.driver = {
		.name = "mc74-caph",
		.owner = THIS_MODULE,
	},
};

static int __init caph_init(void)
{
	struct platform_device *pd;
	int ret = platform_driver_register(&caph_driver);

	if (ret)
		return ret;
	pd = platform_device_register_simple("mc74-caph", -1, NULL, 0);
	return IS_ERR(pd) ? PTR_ERR(pd) : 0;
}
late_initcall(caph_init);

MODULE_DESCRIPTION("Kona CAPH/AudioH ALSA driver (Cisco Meraki MC74)");
MODULE_LICENSE("GPL");
