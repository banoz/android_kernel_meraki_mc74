/* MC74 dt-blob (from the unit's dt-blob partition) wrapped as an ATAG_DTBLOB tag. */
	.section .rodata
	.align	4
	.globl	mc74_builtin_dtblob_tag
mc74_builtin_dtblob_tag:
	.word	(mc74_dtblob_end - mc74_builtin_dtblob_tag) / 4	/* hdr.size (words) */
	.word	0x54411122					/* ATAG_DTBLOB */
	.incbin	"mc74_dtblob.dtb"
mc74_dtblob_end:
	.align	4
