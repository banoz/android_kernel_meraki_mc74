/*
 * MC74 DMA coherency self-test.
 *
 * Uses a spare channel (2) of the ESUB DesignWare AHB DMA controller to do
 * memory-to-memory copies and checks, byte for byte, that the CPU and the DMA
 * agree on memory contents with the cache setup the kernel is running with
 * (L1 + PL310 L2, or l2off):
 *   1. coherent  -> coherent
 *   2. streaming (TO_DEVICE, written through the cache) -> coherent
 *   3. coherent  -> streaming (FROM_DEVICE, with dirty cache lines beforehand)
 *   4. streaming -> streaming (both directions, buffers reused, stale lines)
 * Run with: echo 1 > /sys/module/mc74_dmatest/parameters/run ; dmesg | grep dmatest
 * or at boot with mc74_dmatest.autorun=1
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/random.h>

#define DMAC_PHYS	0x38100000
#define CH		2
#define DW_CH(ch)	((ch) * 0x58)
#define DW_SAR(ch)	(DW_CH(ch) + 0x00)
#define DW_DAR(ch)	(DW_CH(ch) + 0x08)
#define DW_LLP(ch)	(DW_CH(ch) + 0x10)
#define DW_CTL_LO(ch)	(DW_CH(ch) + 0x18)
#define DW_CTL_HI(ch)	(DW_CH(ch) + 0x1c)
#define DW_CFG_LO(ch)	(DW_CH(ch) + 0x40)
#define DW_CFG_HI(ch)	(DW_CH(ch) + 0x44)
#define DW_MASK_TFR	0x310
#define DW_MASK_BLOCK	0x318
#define DW_MASK_SRC	0x320
#define DW_MASK_DST	0x328
#define DW_MASK_ERR	0x330
#define DW_DMA_CFG	0x398
#define DW_CH_EN	0x3a0

/* 64-bit transfers, burst 8, increment both, memory-to-memory, AHB master 1 */
#define CTL_LO_M2M	0x00009036
#define CHUNK		4096		/* bytes per block (512 x 64 bit) */
#define SIZE		(64 * 1024)

static void __iomem *dmac;
static struct platform_device *pdev;
static char result[160] = "not run";
module_param_string(result, result, sizeof(result), 0444);

static int dma_copy(dma_addr_t dst, dma_addr_t src, size_t len)
{
	size_t off;
	int i;

	writel(1, dmac + DW_DMA_CFG);
	for (off = 0; off < len; off += CHUNK) {
		writel(src + off, dmac + DW_SAR(CH));
		writel(dst + off, dmac + DW_DAR(CH));
		writel(0, dmac + DW_LLP(CH));
		writel(CTL_LO_M2M, dmac + DW_CTL_LO(CH));
		writel(CHUNK / 8, dmac + DW_CTL_HI(CH));
		writel(0x000000e0, dmac + DW_CFG_LO(CH));
		writel(0x00000002, dmac + DW_CFG_HI(CH));
		/* mask (don't raise) this channel's interrupts */
		writel(BIT(CH) << 8, dmac + DW_MASK_TFR);
		writel(BIT(CH) << 8, dmac + DW_MASK_BLOCK);
		writel(BIT(CH) << 8, dmac + DW_MASK_SRC);
		writel(BIT(CH) << 8, dmac + DW_MASK_DST);
		writel(BIT(CH) << 8, dmac + DW_MASK_ERR);
		wmb();
		writel(BIT(CH + 8) | BIT(CH), dmac + DW_CH_EN);
		for (i = 0; i < 100000 && (readl(dmac + DW_CH_EN) & BIT(CH)); i++)
			udelay(1);
		if (readl(dmac + DW_CH_EN) & BIT(CH)) {
			writel(BIT(CH + 8), dmac + DW_CH_EN);
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static void fill(u8 *p, size_t len, u32 seed)
{
	size_t i;

	for (i = 0; i < len; i++) {
		seed = seed * 1103515245 + 12345;
		p[i] = seed >> 16;
	}
}

static int check(const char *name, const u8 *got, const u8 *want, size_t len)
{
	size_t i, bad = 0, first = len;

	for (i = 0; i < len; i++)
		if (got[i] != want[i]) {
			if (first == len)
				first = i;
			bad++;
		}
	if (bad)
		pr_err("dmatest: %s: FAIL %zu/%zu bytes differ, first at %zu (got %02x want %02x)\n",
		       name, bad, len, first, got[first], want[first]);
	else
		pr_info("dmatest: %s: ok (%zu bytes)\n", name, len);
	return bad ? 1 : 0;
}

static int run_tests(void)
{
	struct device *dev = &pdev->dev;
	u8 *csrc, *cdst, *ssrc, *sdst, *ref;
	dma_addr_t csrc_b, cdst_b, ssrc_b, sdst_b;
	int fails = 0, rc, round;

	{
		void __iomem *l2c = ioremap(0x3FF20000, SZ_4K);

		if (l2c) {
			pr_info("dmatest: PL310 CTRL %08x AUX %08x\n",
				readl(l2c + 0x100), readl(l2c + 0x104));
			iounmap(l2c);
		}
	}
	csrc = dma_alloc_coherent(dev, SIZE, &csrc_b, GFP_KERNEL);
	cdst = dma_alloc_coherent(dev, SIZE, &cdst_b, GFP_KERNEL);
	ssrc = kmalloc(SIZE, GFP_KERNEL);
	sdst = kmalloc(SIZE, GFP_KERNEL);
	ref = kmalloc(SIZE, GFP_KERNEL);
	if (!csrc || !cdst || !ssrc || !sdst || !ref) {
		rc = -ENOMEM;
		goto out;
	}

	for (round = 0; round < 3; round++) {
		u32 seed = 0x1234 + round * 977;

		/* 1. coherent -> coherent */
		fill(ref, SIZE, seed);
		memcpy(csrc, ref, SIZE);
		memset(cdst, 0xa5, SIZE);
		wmb();
		rc = dma_copy(cdst_b, csrc_b, SIZE);
		if (rc)
			goto out;
		rmb();
		fails += check("coherent->coherent", cdst, ref, SIZE);

		/* 2. streaming TO_DEVICE -> coherent (data only in the CPU caches) */
		fill(ref, SIZE, seed + 1);
		memcpy(ssrc, ref, SIZE);
		memset(cdst, 0x5a, SIZE);
		ssrc_b = dma_map_single(dev, ssrc, SIZE, DMA_TO_DEVICE);
		rc = dma_copy(cdst_b, ssrc_b, SIZE);
		dma_unmap_single(dev, ssrc_b, SIZE, DMA_TO_DEVICE);
		if (rc)
			goto out;
		rmb();
		fails += check("streaming(TO_DEVICE)->coherent", cdst, ref, SIZE);

		/* 3. coherent -> streaming FROM_DEVICE, destination has dirty lines */
		fill(ref, SIZE, seed + 2);
		memcpy(csrc, ref, SIZE);
		memset(sdst, 0x3c, SIZE);		/* dirty in L1/L2 */
		wmb();
		sdst_b = dma_map_single(dev, sdst, SIZE, DMA_FROM_DEVICE);
		rc = dma_copy(sdst_b, csrc_b, SIZE);
		dma_unmap_single(dev, sdst_b, SIZE, DMA_FROM_DEVICE);
		if (rc)
			goto out;
		fails += check("coherent->streaming(FROM_DEVICE)", sdst, ref, SIZE);

		/* 4. streaming -> streaming, reading the destination first so stale
		 *    clean lines exist in the caches */
		fill(ref, SIZE, seed + 3);
		memcpy(ssrc, ref, SIZE);
		(void)ACCESS_ONCE(sdst[0]);
		{
			volatile u32 sum = 0;
			size_t i;

			for (i = 0; i < SIZE; i += 32)
				sum += sdst[i];
		}
		ssrc_b = dma_map_single(dev, ssrc, SIZE, DMA_TO_DEVICE);
		sdst_b = dma_map_single(dev, sdst, SIZE, DMA_FROM_DEVICE);
		rc = dma_copy(sdst_b, ssrc_b, SIZE);
		dma_unmap_single(dev, sdst_b, SIZE, DMA_FROM_DEVICE);
		dma_unmap_single(dev, ssrc_b, SIZE, DMA_TO_DEVICE);
		if (rc)
			goto out;
		fails += check("streaming->streaming", sdst, ref, SIZE);
	}
	rc = 0;
out:
	if (rc)
		pr_err("dmatest: DMA error %d\n", rc);
	snprintf(result, sizeof(result), "%s: %d failing checks of 12%s",
		 outer_cache.flush_range ? "L2 on" : "L2 off", fails,
		 rc ? " (DMA error)" : "");
	pr_info("dmatest: %s\n", result);
	if (csrc)
		dma_free_coherent(dev, SIZE, csrc, csrc_b);
	if (cdst)
		dma_free_coherent(dev, SIZE, cdst, cdst_b);
	kfree(ssrc);
	kfree(sdst);
	kfree(ref);
	return rc;
}

static bool autorun;
module_param(autorun, bool, 0444);

static int run_set(const char *val, struct kernel_param *kp)
{
	return run_tests();
}
module_param_call(run, run_set, NULL, NULL, 0200);

static int __init mc74_dmatest_init(void)
{
	dmac = ioremap(DMAC_PHYS, SZ_4K);
	if (!dmac)
		return -ENOMEM;
	pdev = platform_device_register_simple("mc74_dmatest", -1, NULL, 0);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (autorun)
		run_tests();
	return 0;
}
late_initcall(mc74_dmatest_init);
MODULE_LICENSE("GPL");
