# Cisco Meraki MC74 speaker/mic: Broadcom chal CAPH/AudioH helpers + plain ALSA card
# (port of the mainline mc74 kona-caph driver)
ccflags-y += -I$(src)/include -Wno-unused-function -Wno-unused-variable
obj-$(CONFIG_MC74_CAPH) += snd-mc74-caph.o
snd-mc74-caph-y := chal_caph_dma.o chal_caph_cfifo.o chal_caph_switch.o chal_caph_intc.o chal_ihf.o chal_vin.o chal_vout.o kona_caph_pcm.o
