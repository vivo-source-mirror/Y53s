#ifndef __FP_ID_H
#define __FP_ID_H
/*for kernel
denominate method: ##VENDOR_##MODULE*/
#define FPC_BASE				0x00000100
#define FPC_FPC1022				(FPC_BASE + 1)
#define FPC_FPC1245				(FPC_BASE + 2)
#define FPC_FPC1229				(FPC_BASE + 3)
#define FPC_FPC1511				(FPC_BASE + 4)
#define FPC_FPC1540				(FPC_BASE + 5)

#define GOODIX_BASE				0x00000200
#define GOODIX_GF318M			(GOODIX_BASE + 1)
#define GOODIX_GF3208			(GOODIX_BASE + 2)
#define GOODIX_GF5126M			(GOODIX_BASE + 3)
#define GOODIX_GF5216C			(GOODIX_BASE + 4)
#define GOODIX_GF5269			(GOODIX_BASE + 5)
#define GOODIX_GF5288			(GOODIX_BASE + 6)
#define GOODIX_GF9518			(GOODIX_BASE + 7)
#define GOODIX_GF9518N			(GOODIX_BASE + 8)
#define GOODIX_GF3658			(GOODIX_BASE + 9)
#define GOODIX_GF3626			(GOODIX_BASE + 10)
#define GOODIX_GF9578			(GOODIX_BASE + 11)

#define EGIS_BASE				0x00000300
#define EGIS_ET713				(EGIS_BASE + 1)

#define SILEAD_BASE				0x00000400
#define SILEAD_GSL7001			(SILEAD_BASE + 1)

/*for frameworks
denominate method: ##VENDOR_FRAME_ID*/
#define FPC_FRAME_ID			1
#define GOODIX_FRAME_ID			2
#define SILEAD_FRAME_ID			4

#define     FP_AVDD_EN_1V8        (GPIO54 | 0x80000000)


int get_fp_id(void);

#endif

