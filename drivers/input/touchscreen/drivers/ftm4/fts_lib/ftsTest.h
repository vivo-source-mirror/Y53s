/*

 **************************************************************************
 **                        STMicroelectronics 		                **
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *               	FTS API for MP test				 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

*/


#include "ftsSoftware.h"

#define LIMITS_FILE							"stm_fts_production_limits.csv"


#define WAIT_FOR_FRESH_FRAMES					100				/*ms */
#define WAIT_AFTER_SENSEOFF						50				/*ms */

#define TIMEOUT_ITO_TEST_RESULT					200				/*ms */
#define TIMEOUT_INITIALIZATION_TEST_RESULT		5000			/*ms */

/*LABELS PRODUCTION TEST LIMITS FILE */
#define MS_RAW_MIN_MAX				"MS_RAW_DATA_MIN_MAX"
#define MS_RAW_GAP					"MS_RAW_DATA_GAP"
#define MS_RAW_LP_MIN_MAX			"MS_RAW_LOWPOWER_DATA_MIN_MAX"
#define MS_RAW_LP_GAP				"MS_RAW_LOWPOWER_DATA_GAP"
#define MS_RAW_LP_ADJH				"MS_RAW_LOWPOWER_DATA_ADJ_HORIZONTAL"
#define MS_RAW_LP_ADJV				"MS_RAW_LOWPOWER_DATA_ADJ_VERTICAL"
#define MS_CX1_MIN_MAX				"MS_TOUCH_ACTIVE_CX1_MIN_MAX"
#define MS_CX2_MAP_MIN				"MS_TOUCH_ACTIVE_CX2_MIN"
#define MS_CX2_MAP_MAX				"MS_TOUCH_ACTIVE_CX2_MAX"
#define MS_CX2_ADJH_MAP_MAX			"MS_TOUCH_ACTIVE_CX2_ADJ_HORIZONTAL"
#define MS_CX2_ADJV_MAP_MAX			"MS_TOUCH_ACTIVE_CX2_ADJ_VERTICAL"
#define MS_TOTAL_CX_MAP_MIN			"MS_TOUCH_LOWPOWER_TOTAL_CX_MIN"
#define MS_TOTAL_CX_MAP_MAX			"MS_TOUCH_LOWPOWER_TOTAL_CX_MAX"
#define MS_TOTAL_CX_ADJH_MAP_MAX	"MS_TOUCH_LOWPOWER_TOTAL_CX_ADJ_HORIZONTAL"
#define MS_TOTAL_CX_ADJV_MAP_MAX	"MS_TOUCH_LOWPOWER_TOTAL_CX_ADJ_VERTICAL"
#define SS_RAW_FORCE_MIN_MAX		"SS_RAW_DATA_FORCE_MIN_MAX"
#define SS_RAW_SENSE_MIN_MAX		"SS_RAW_DATA_SENSE_MIN_MAX"
#define SS_RAW_FORCE_GAP			"SS_RAW_DATA_FORCE_GAP"
#define SS_RAW_SENSE_GAP			"SS_RAW_DATA_SENSE_GAP"
#define SS_IX1_FORCE_MIN_MAX		"SS_TOUCH_ACTIVE_IX1_FORCE_MIN_MAX"
#define SS_IX1_SENSE_MIN_MAX		"SS_TOUCH_ACTIVE_IX1_SENSE_MIN_MAX"
#define SS_CX1_FORCE_MIN_MAX		"SS_TOUCH_ACTIVE_CX1_FORCE_MIN_MAX"
#define SS_CX1_SENSE_MIN_MAX		"SS_TOUCH_ACTIVE_CX1_SENSE_MIN_MAX"
#define SS_IX2_FORCE_MAP_MIN		"SS_TOUCH_ACTIVE_IX2_FORCE_MIN"
#define SS_IX2_FORCE_MAP_MAX		"SS_TOUCH_ACTIVE_IX2_FORCE_MAX"
#define SS_IX2_SENSE_MAP_MIN		"SS_TOUCH_ACTIVE_IX2_SENSE_MIN"
#define SS_IX2_SENSE_MAP_MAX		"SS_TOUCH_ACTIVE_IX2_SENSE_MAX"
#define SS_IX2_FORCE_ADJV_MAP_MAX	"SS_TOUCH_ACTIVE_IX2_ADJ_VERTICAL"
#define SS_IX2_SENSE_ADJH_MAP_MAX	"SS_TOUCH_ACTIVE_IX2_ADJ_HORIZONTAL"
#define SS_CX2_FORCE_MAP_MIN		"SS_TOUCH_ACTIVE_CX2_FORCE_MIN"
#define SS_CX2_FORCE_MAP_MAX		"SS_TOUCH_ACTIVE_CX2_FORCE_MAX"
#define SS_CX2_SENSE_MAP_MIN		"SS_TOUCH_ACTIVE_CX2_SENSE_MIN"
#define SS_CX2_SENSE_MAP_MAX		"SS_TOUCH_ACTIVE_CX2_SENSE_MAX"
#define SS_CX2_FORCE_ADJV_MAP_MAX	"SS_TOUCH_ACTIVE_CX2_ADJ_VERTICAL"
#define SS_CX2_SENSE_ADJH_MAP_MAX	"SS_TOUCH_ACTIVE_CX2_ADJ_HORIZONTAL"

/* TOTAL SS */
#define SS_TOTAL_IX_FORCE_MAP_MIN		"SS_TOUCH_ACTIVE_TOTAL_IX_FORCE_MIN"
#define SS_TOTAL_IX_FORCE_MAP_MAX		"SS_TOUCH_ACTIVE_TOTAL_IX_FORCE_MAX"
#define SS_TOTAL_IX_SENSE_MAP_MIN		"SS_TOUCH_ACTIVE_TOTAL_IX_SENSE_MIN"
#define SS_TOTAL_IX_SENSE_MAP_MAX		"SS_TOUCH_ACTIVE_TOTAL_IX_SENSE_MAX"
#define SS_TOTAL_IX_FORCE_ADJV_MAP_MAX	"SS_TOUCH_ACTIVE_TOTAL_IX_ADJ_VERTICAL"
#define SS_TOTAL_IX_SENSE_ADJH_MAP_MAX	"SS_TOUCH_ACTIVE_TOTAL_IX_ADJ_HORIZONTAL"
#define SS_TOTAL_CX_FORCE_MAP_MIN		"SS_TOUCH_ACTIVE_TOTAL_CX_FORCE_MIN"
#define SS_TOTAL_CX_FORCE_MAP_MAX		"SS_TOUCH_ACTIVE_TOTAL_CX_FORCE_MAX"
#define SS_TOTAL_CX_SENSE_MAP_MIN		"SS_TOUCH_ACTIVE_TOTAL_CX_SENSE_MIN"
#define SS_TOTAL_CX_SENSE_MAP_MAX		"SS_TOUCH_ACTIVE_TOTAL_CX_SENSE_MAX"
#define SS_TOTAL_CX_FORCE_ADJV_MAP_MAX	"SS_TOUCH_ACTIVE_TOTAL_CX_ADJ_VERTICAL"
#define SS_TOTAL_CX_SENSE_ADJH_MAP_MAX	"SS_TOUCH_ACTIVE_TOTAL_CX_ADJ_HORIZONTAL"

/*KEYS */
#define MS_KEY_RAW_MIN_MAX			"MS_KEY_RAW_DATA_MIN_MAX"
#define MS_KEY_CX1_MIN_MAX			"MS_KEY_CX1_MIN_MAX"
#define MS_KEY_CX2_MAP_MIN			"MS_KEY_CX2_MIN"
#define MS_KEY_CX2_MAP_MAX			"MS_KEY_CX2_MAX"
#define MS_KEY_TOTAL_CX_MAP_MIN		"MS_KEY_TOTAL_CX_MIN"
#define MS_KEY_TOTAL_CX_MAP_MAX		"MS_KEY_TOTAL_CX_MAX"

//CONSTANT TOTAL IX
#define SS_IX1_FORCE_W                      "IX1_FORCE_W"
#define SS_IX2_FORCE_W                      "IX2_FORCE_W"
#define SS_IX1_SENSE_W                      "IX1_SENSE_W"
#define SS_IX2_SENSE_W                      "IX2_SENSE_W"


#define SAVE_FLAG_RETRY		3


typedef struct {
	int MutualRaw;
    int MutualRawGap;
	int MutualRawLP;															///< MS Low Power Raw min/Max test
    int MutualRawGapLP;															///< MS Low Power Raw Gap(max-min) test
	int MutualRawAdjLP;															///< MS Low Power Raw Adjacent test
	int MutualCx1;
	int MutualCx2;
	int MutualCx2Adj;
	int MutualCxTotal;
	int MutualCxTotalAdj;

	int MutualKeyRaw;
	int MutualKeyCx1;
	int MutualKeyCx2;
	int MutualKeyCxTotal;

	int SelfForceRaw;
	int SelfForceRawGap;
	int SelfForceIx1;
	int SelfForceIx2;
	int SelfForceIx2Adj;
	int SelfForceIxTotal;
	int SelfForceIxTotalAdj;
	int SelfForceCx1;
	int SelfForceCx2;
	int SelfForceCx2Adj;
	int SelfForceCxTotal;
	int SelfForceCxTotalAdj;

	int SelfSenseRaw;
	int SelfSenseRawGap;
	int SelfSenseIx1;
	int SelfSenseIx2;
	int SelfSenseIx2Adj;
	int SelfSenseIxTotal;
	int SelfSenseIxTotalAdj;
	int SelfSenseCx1;
	int SelfSenseCx2;
	int SelfSenseCx2Adj;
	int SelfSenseCxTotal;
	int SelfSenseCxTotalAdj;

} TestToDo;

int ftm4_computeAdjHoriz(struct fts_ts_info *info, u8 *data, int row, int column, u8 **result);
int ftm4_computeAdjHorizTotal(struct fts_ts_info *info, u16 *data, int row, int column, u16 **result);
int ftm4_computeAdjVert(struct fts_ts_info *info, u8 *data, int row, int column, u8 **result);
int ftm4_computeAdjVertTotal(struct fts_ts_info *info, u16 *data, int row, int column, u16 **result);
int ftm4_computeTotal(struct fts_ts_info *info, u8 *data, u8 main, int row, int column, int m, int n, u16 **result);
int ftm4_checkLimitsMinMax(struct fts_ts_info *info, short *data, int row, int column, int min, int max);
int ftm4_checkLimitsMap(struct fts_ts_info *info, u8 *data, int row, int column, int *min, int *max);
int ftm4_checkLimitsMapTotal(struct fts_ts_info *info, u16 *data, int row, int column, int *min, int *max);
int ftm4_checkLimitsMapAdj(struct fts_ts_info *info, u8 *data, int row, int column, int *max);
int ftm4_checkLimitsMapAdjTotal(struct fts_ts_info *info, u16 *data, int row, int column, int *max);
int ftm4_production_test_ito(struct fts_ts_info *info);
int ftm4_production_test_initialization(struct fts_ts_info *info);
int ftm4_ms_compensation_tuning(struct fts_ts_info *info);
int ftm4_ss_compensation_tuning(struct fts_ts_info *info);
int ftm4_lp_timer_calibration(struct fts_ts_info *info);
int ftm4_save_cx_tuning(struct fts_ts_info *info);
int ftm4_production_test_splitted_initialization(struct fts_ts_info *info, int saveToFlash);
int ftm4_production_test_main(struct fts_ts_info *info, char *pathThresholds, int stop_on_fail, int saveInit, TestToDo *todo,
		u32 signature, enum vts_sensor_test_result *result);
int ftm4_production_test_ms_raw(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo);
int ftm4_production_test_ms_cx(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo);
int ftm4_production_test_ss_raw(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo);
int ftm4_production_test_ss_ix_cx(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo);
int ftm4_production_test_data(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo);
int ftm4_production_test_ms_key_cx(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo);
int ftm4_production_test_ms_key_raw(struct fts_ts_info *info,char *path_limits);
int fts_save_mp_flag(struct fts_ts_info *info, u32 signature);
int ftm4_parseProductionTestLimits(struct fts_ts_info *info, char *path, char *label, int **data, int *row, int *column);
int ftm4_readLine(char *data, char *line, int size, int *n);
