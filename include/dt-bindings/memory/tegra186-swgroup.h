/* To merge back to "dt-bindings/memory/tegra-swgroup.h" */
#define TEGRA_SWGROUP_NVCSI	45
#define TEGRA_SWGROUP_SCE	52
#define TEGRA_SWGROUP_HOST1X_CTX0	53
#define TEGRA_SWGROUP_HOST1X_CTX1	54
#define TEGRA_SWGROUP_HOST1X_CTX2	55
#define TEGRA_SWGROUP_HOST1X_CTX3	56
#define TEGRA_SWGROUP_HOST1X_CTX4	57
#define TEGRA_SWGROUP_HOST1X_CTX5	58
#define TEGRA_SWGROUP_HOST1X_CTX6	59
#define TEGRA_SWGROUP_HOST1X_CTX7	60
#define TEGRA_SWGROUP_SE2		61
#define TEGRA_SWGROUP_SE3		62

/*
 * This is the t18x specific component of the new SID dt-binding.
 */
#define TEGRA_SID_NVCSI		0x2	/* 2 */
#define TEGRA_SID_SE2		0xe	/* 14 */
#define TEGRA_SID_SE3		0xf	/* 15 */
#define TEGRA_SID_SCE		0x1f	/* 31 */

/* The GPC DMA clients. */
#define TEGRA_SID_GPCDMA_0	0x20	/* 32 */
#define TEGRA_SID_GPCDMA_1	0x21	/* 33 */
#define TEGRA_SID_GPCDMA_2	0x22	/* 34 */
#define TEGRA_SID_GPCDMA_3	0x23	/* 35 */
#define TEGRA_SID_GPCDMA_4	0x24	/* 36 */
#define TEGRA_SID_GPCDMA_5	0x25	/* 37 */
#define TEGRA_SID_GPCDMA_6	0x26	/* 38 */
#define TEGRA_SID_GPCDMA_7	0x27	/* 39 */

/* Host1x virtualization clients. */
#define TEGRA_SID_HOST1X_CTX0	0x38	/* 56 */
#define TEGRA_SID_HOST1X_CTX1	0x39	/* 57 */
#define TEGRA_SID_HOST1X_CTX2	0x3a	/* 58 */
#define TEGRA_SID_HOST1X_CTX3	0x3b	/* 59 */
#define TEGRA_SID_HOST1X_CTX4	0x3c	/* 60 */
#define TEGRA_SID_HOST1X_CTX5	0x3d	/* 61 */
#define TEGRA_SID_HOST1X_CTX6	0x3e	/* 62 */
#define TEGRA_SID_HOST1X_CTX7	0x3f	/* 63 */
