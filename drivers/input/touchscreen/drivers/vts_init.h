#ifndef __VTS_INIT_H
#define __VTS_INIT_H

int vts_driver_ftm4_init(int *load);
int vts_driver_st80y_init(int *load);
int vts_driver_nt_i2c_init(int *load);
int vts_driver_nt_no_flash_init(int *load);
int vts_driver_sec_y761_init(int *load);
int vts_driver_hvt_no_flash_init(int *load);
int vts_driver_goodix_g9885_init(int *load);
int vts_driver_goodix_gt9886_init(int *load);
int vts_driver_ft8719_no_flash_init(int *load);
int vts_driver_ft8756_no_flash_init(int *load);
int vts_driver_ili_tek_init(int *load);
int vts_driver_ili_9882n_init(int *load);
int vts_driver_synaptics_S3908_init(int *load);
int vts_driver_goodix_gt9897_init(int *load);
int vts_driver_goodix_gt9897_i2c_init(int *load);
int vts_driver_chipone_nl9911c_init(int *load);
int vts_driver_ft3518u_init(int *load);
int vts_log_switch_init(void);
void vts_driver_ftm4_exit(void);
void vts_driver_st80y_exit(void);
void vts_driver_nt_i2c_exit(void);
void vts_driver_nt_no_flash_exit(void);
void vts_driver_sec_y761_exit(void);
void vts_driver_hvt_no_flash_exit(void);
void vts_driver_goodix_g9885_exit(void);
void vts_driver_goodix_gt9886_exit(void);
void vts_driver_ft8719_no_flash_exit(void);
void vts_driver_ft8756_no_flash_exit(void);
void vts_driver_ili_tek_exit(void);
void vts_driver_ili_9882n_exit(void);
void vts_driver_synaptics_S3908_exit(void);
void vts_log_switch_exit(void);
void vts_driver_goodix_gt9897_exit(void);
void vts_driver_goodix_gt9897_i2c_exit(void);
void vts_driver_chipone_nl9911c_exit(void);
void vts_driver_ft3518u_exit(void);

#endif
