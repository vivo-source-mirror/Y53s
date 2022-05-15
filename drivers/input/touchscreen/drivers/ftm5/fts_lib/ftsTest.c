/*
  *
  **************************************************************************
  **                        STMicroelectronics				  **
  **************************************************************************
  **                        marco.cali@st.com				  **
  **************************************************************************
  *                                                                        *
  *			   FTS API for MP test				   *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsTest.c
  * \brief Contains all the functions related to the Mass Production Test
  */

#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../fts.h"


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>

/**
  * Initialize the testToDo variable with the default info->tests_ftm5 to perform during
  * the Mass Production Test
  * @return OK
  */
int initTestToDo(struct fts_ts_info *info)
{
	/*** Initialize Limit File ***/
	info->limit_file.size = 0;
	info->limit_file.data = NULL;
	strlcpy(info->limit_file.name, " ", MAX_LIMIT_FILE_NAME);
	return OK;
}

/**
  * Compute the Horizontal adjacent matrix doing the abs of the difference
  * between the column i with the i-1 one. \n
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHoriz_ftm5(i8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		logError_ftm5(1, "%s computeAdjHoriz_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjHoriz_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix of short values doing the abs of
  * the difference between the column i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  *  array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which
  * will contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizTotal_ftm5(short *data, int row, int column, u16 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		logError_ftm5(1, "%s computeAdjHorizTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjHorizTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix doing the abs of the difference between
  * the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVert_ftm5(i8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		logError_ftm5(1, "%s computeAdjVert_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjVert_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix of short values doing the abs of
  * the difference between the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertTotal_ftm5(short *data, int row, int column, u16 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		logError_ftm5(1, "%s computeAdjVertTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjVertTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix doing the abs of the difference
  * between the column i with the i-1 one. \n
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  *  contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizFromU(u8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		logError_ftm5(1, "%s computeAdjHoriz_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjHoriz_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix of u16 values doing the abs of
  * the difference between the column i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  *  array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizTotalFromU(u16 *data, int row, int column, u16 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		logError_ftm5(1, "%s computeAdjHorizTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjHorizTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix doing the abs of the difference between
  *  the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  *  contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertFromU(u8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		logError_ftm5(1, "%s computeAdjVert_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjVert_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix of u16 values doing the abs of
  * the difference between the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertTotalFromU(u16 *data, int row, int column, u16 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		logError_ftm5(1, "%s computeAdjVertTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		logError_ftm5(1, "%s computeAdjVertTotal_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}



/**
  * Check that each value of a matrix of short doesn't exceed a min and a Max
  * value (these values are included in the interval). \n
  * The matrix is stored as 1 dimension array one row after the other. \n
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min minimum value allowed
  * @param max Maximum value allowed
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMinMax_ftm5(short *data, int row, int column, int min, int max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min || data[i * column + j] >
			    max) {
				logError_ftm5(1,
					 "%s checkLimitsMinMax_ftm5: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					 tag_ftm5, i, j, data[i * column + j], min,
					 max);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that the difference between the max and min of a matrix of short is
  * less or equal to a threshold.\n
  * The matrix is stored as 1 dimension array one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param threshold threshold value allowed
  * @return OK if the difference is <= to threshold otherwise
  * ERROR_TEST_CHECK_FAIL
  */
int checkLimitsGap_ftm5(short *data, int row, int column, int threshold)
{
	int i, j;
	int min_node;
	int max_node;

	if (row == 0 || column == 0) {
		logError_ftm5(1,
			 "%s checkLimitsGap_ftm5: invalid number of rows = %d or columns = %d  ERROR %08X\n",
			 tag_ftm5, row, column, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	min_node = data[0];
	max_node = data[0];

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min_node)
				min_node = data[i * column + j];
			else if (data[i * column + j] > max_node)
				max_node = data[i * column + j];
		}
	}

	if (max_node - min_node > threshold) {
		logError_ftm5(1, "%s checkLimitsGap_ftm5: GAP = %d exceed limit  %d\n",
			 tag_ftm5, max_node - min_node, threshold);
		return ERROR_TEST_CHECK_FAIL;
	} else
		return OK;
}

/**
  * Check that the difference between the max and min of a matrix of short is
  * less or equal to a threshold.\n
  * The matrix is stored as 1 dimension array one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param threshold threshold value allowed
  * @param row_start index of the starting column which should be considered
  * @param column_start index of the starting column which should be considered
  * @param row_end number of index to subtract to row to identify last
  *		valid row to check
  * @param column_end number of index to subtract to column to identify last
  *		valid column to check
  * @return OK if the difference is <= to threshold otherwise
  * ERROR_TEST_CHECK_FAIL
  */
int checkLimitsGapOffsets(short *data, int row, int column, int threshold,
	int row_start, int column_start, int row_end, int column_end)
{
	int i, j;
	int min_node;
	int max_node;

	if (row == 0 || column == 0) {
		logError_ftm5(1,
			 "%s checkLimitsGap_ftm5: invalid number of rows = %d or columns = %d  ERROR %08X\n",
			 tag_ftm5, row, column, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	min_node = data[row_start*column+column_start];
	max_node = data[row_start*column+column_start];

	for (i = row_start; i < row-row_end; i++) {
		for (j = column_start; j < column-column_end; j++) {
			if (data[i * column + j] < min_node)
				min_node = data[i * column + j];
			else if (data[i * column + j] > max_node)
				max_node = data[i * column + j];
		}
	}

	if (max_node - min_node > threshold) {
		logError_ftm5(1, "%s checkLimitsGap_ftm5: GAP = %d exceed limit  %d\n",
			 tag_ftm5, max_node - min_node, threshold);
		return ERROR_TEST_CHECK_FAIL;
	} else
		return OK;
}

/**
  * Check that each value of a matrix of i8 doesn't exceed a specific min and
  *Max value  set for each node (these values are included in the interval). \n
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  *one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  *each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  *each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMap_ftm5(i8 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				logError_ftm5(1,
					 "%s checkLimitsMap_ftm5: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					 tag_ftm5, i, j, data[i * column + j],
					 min[i *
					     column
					     + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of short doesn't exceed a specific min and
  *  Max value  set for each node (these values are included in the interval).
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapTotal_ftm5(short *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				logError_ftm5(1,
					 "%s checkLimitsMapTotal_ftm5: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					 tag_ftm5, i, j, data[i * column + j],
					 min[i *
					     column
					     + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u8 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval). \n
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapFromU(u8 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				logError_ftm5(1,
					 "%s checkLimitsMap_ftm5: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					 tag_ftm5, i, j, data[i * column + j],
					 min[i *
					     column
					     + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u16 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval).
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapTotalFromU(u16 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				logError_ftm5(1,
					 "%s checkLimitsMapTotal_ftm5: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					 tag_ftm5, i, j, data[i * column + j],
					 min[i *
					     column
					     + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u8 doesn't exceed a specific Max value
  * set for each node (max value is included in the interval).
  * The matrixes of data and max values are stored as 1 dimension arrays one row
  * after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param max pointer to a matrix which specify the Maximum value allowed for
  *each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapAdj_ftm5(u8 *data, int row, int column, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] > max[i * column + j]) {
				logError_ftm5(1,
					 "%s checkLimitsMapAdj_ftm5: Node[%d,%d] = %d exceed limit > %d\n",
					 tag_ftm5, i, j, data[i * column + j],
					 max[i *
					     column
					     + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u16 doesn't exceed a specific Max value
  * set for each node (max value is included in the interval).
  * The matrixes of data and max values are stored as 1 dimension arrays one row
  * after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapAdjTotal_ftm5(u16 *data, int row, int column, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] > max[i * column + j]) {
				logError_ftm5(1,
					 "%s checkLimitsMapAdjTotal_ftm5: Node[%d,%d] = %d exceed limit > %d\n",
					 tag_ftm5, i, j, data[i * column + j],
					 max[i *
					     column
					     + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Perform an ITO test setting all the possible options
  * (see @link ito_opt ITO Options @endlink) and checking MS Raw ADJ if enabled
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ito_ftm5(struct fts_ts_info *info, char *path_limits, TestToDo *todo)
{
	int res = OK;
	u8 sett[2] = { 0x00, 0x00 };
	MutualSenseFrame msRawFrame;
	int *thresholds = NULL;
	u16 *adj = NULL;
	int trows, tcolumns;

	logError_ftm5(0, "%s ITO Production test is starting...\n", tag_ftm5);

	res = fts_system_reset_ftm5(info);
	if (res < 0) {
		logError_ftm5(1, "%s %s: ERROR %08X\n", tag_ftm5, __func__,
			 ERROR_PROD_TEST_ITO);
		return res | ERROR_PROD_TEST_ITO;
	}

	sett[0] = SPECIAL_TUNING_IOFF;
	logError_ftm5(0, "%s Trimming Ioff...\n", tag_ftm5);
	res = writeSysCmd(info, SYS_CMD_SPECIAL_TUNING, sett, 2);
	if (res < OK) {
		logError_ftm5(1, "%s production_test_ito_ftm5: Trimm Ioff ERROR %08X\n",
			 tag_ftm5, (res | ERROR_PROD_TEST_ITO));
		return res | ERROR_PROD_TEST_ITO;
	}

	sett[0] = 0xFF;
	sett[1] = 0xFF;
	logError_ftm5(0, "%s ITO Check command sent...\n", tag_ftm5);
	res = writeSysCmd(info, SYS_CMD_ITO, sett, 2);
	if (res < OK) {
		logError_ftm5(1, "%s production_test_ito_ftm5: ERROR %08X\n", tag_ftm5,
			 (res | ERROR_PROD_TEST_ITO));
		return res | ERROR_PROD_TEST_ITO;
	}

	logError_ftm5(0, "%s ITO Command = OK!\n", tag_ftm5);

	logError_ftm5(0, "%s MS RAW ITO ADJ TEST:\n", tag_ftm5);
	if (todo->MutualRawAdjITO == 1) {
		logError_ftm5(0, "%s Collecting MS Raw data...\n", tag_ftm5);
		res |= getMSFrame3_ftm5(info, MS_RAW, &msRawFrame);
		if (res < OK) {
			logError_ftm5(1, "%s %s: getMSFrame failed... ERROR %08X\n",
				 tag_ftm5, __func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}
		logError_ftm5(0, "%s MS RAW ITO ADJ HORIZONTAL TEST:\n", tag_ftm5);
		res = computeAdjHorizTotal_ftm5(msRawFrame.node_data,
					   msRawFrame.header.force_node,
					   msRawFrame.header.sense_node,
					   &adj);
		if (res < OK) {
			logError_ftm5(1,
				 "%s %s: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
				 tag_ftm5,
				 __func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}

		res = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_RAW_ITO_ADJH, &thresholds,
						&trows, &tcolumns);
		if (res < OK || (trows != msRawFrame.header.force_node ||
				 tcolumns != msRawFrame.header.sense_node -
				 1)) {
			logError_ftm5(1,
				 "%s %s: parseProductionTestLimits_ftm5 MS_RAW_ITO_ADJH failed... ERROR %08X\n",
				 tag_ftm5, __func__, ERROR_PROD_TEST_DATA);
			goto ERROR;
		}


		res = checkLimitsMapAdjTotal_ftm5(adj, msRawFrame.header.force_node,
					     msRawFrame.header.sense_node - 1,
					     thresholds);
		if (res != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ITO ADJH failed... ERROR COUNT = %d\n",
				 tag_ftm5, res);
			logError_ftm5(0,
				 "%s MS RAW ITO ADJ HORIZONTAL TEST:.................FAIL\n\n",
				 tag_ftm5);
			print_frame_short_ftm5("MS Raw ITO frame =",
					  array1dTo2d_short_ftm5(
						  msRawFrame.node_data,
						  msRawFrame.
						  node_data_size,
						  msRawFrame.header.
						  sense_node),
					  msRawFrame.header.force_node,
					  msRawFrame.header.sense_node);
			res = ERROR_PROD_TEST_ITO;
			goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS RAW ITO ADJ HORIZONTAL TEST:.................OK\n",
				 tag_ftm5);

		kfree(thresholds);
		thresholds = NULL;

		kfree(adj);
		adj = NULL;

		logError_ftm5(0, "%s MS RAW ITO ADJ VERTICAL TEST:\n", tag_ftm5);
		res = computeAdjVertTotal_ftm5(msRawFrame.node_data,
					  msRawFrame.header.force_node,
					  msRawFrame.header.sense_node,
					  &adj);
		if (res < OK) {
			logError_ftm5(1,
				 "%s %s: computeAdjVert_ftm5 failed... ERROR %08X\n",
				 tag_ftm5,
				 __func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}

		res = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_RAW_ITO_ADJV, &thresholds,
						&trows, &tcolumns);
		if (res < OK || (trows != msRawFrame.header.force_node - 1 ||
				 tcolumns != msRawFrame.header.sense_node)) {
			logError_ftm5(1,
				 "%s %s: parseProductionTestLimits_ftm5 MS_RAW_ITO_ADJV failed... ERROR %08X\n",
				 tag_ftm5, __func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}


		res = checkLimitsMapAdjTotal_ftm5(adj, msRawFrame.header.force_node -
					     1, msRawFrame.header.sense_node,
					     thresholds);
		if (res != OK) {
			logError_ftm5(1,
				 "%s %s: checkLimitsAdj MS RAW ITO ADJV failed... ERROR COUNT = %d\n",
				 tag_ftm5, __func__, res);
			logError_ftm5(0,
				 "%s MS RAW ITO ADJ VERTICAL TEST:.................FAIL\n\n",
				 tag_ftm5);
			print_frame_short_ftm5("MS Raw ITO frame =",
					  array1dTo2d_short_ftm5(
						  msRawFrame.node_data,
						  msRawFrame.
						  node_data_size,
						  msRawFrame.header.
						  sense_node),
					  msRawFrame.header.force_node,
					  msRawFrame.header.sense_node);
			res = ERROR_PROD_TEST_ITO;
			goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS RAW ITO ADJ VERTICAL TEST:.................OK\n",
				 tag_ftm5);

		kfree(thresholds);
		thresholds = NULL;

		kfree(adj);
		adj = NULL;
	} else
		logError_ftm5(0,
			 "%s MS RAW ITO ADJ TEST:.................SKIPPED\n",
			 tag_ftm5);

ERROR:
	if (thresholds != NULL)
		kfree(thresholds);
	if (adj != NULL)
		kfree(adj);
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	freeLimitsFile(&info->limit_file);
	res |= fts_system_reset_ftm5(info);
	if (res < OK) {
		logError_ftm5(1, "%s production_test_ito_ftm5: ERROR %08X\n", tag_ftm5,
			 ERROR_PROD_TEST_ITO);
		res = (res | ERROR_PROD_TEST_ITO);
	}
	return res;
}

/**
  * Perform the Initialization of the IC
  * @param type type of initialization to do
  * (see @link sys_special_opt Initialization Options (Full or Panel) @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int production_test_initialization_ftm5(struct fts_ts_info *info, u8 type)
{
	int res;

	logError_ftm5(0, "%s INITIALIZATION Production test is starting...\n", tag_ftm5);
	if (type != SPECIAL_PANEL_INIT && type != SPECIAL_FULL_PANEL_INIT) {
		logError_ftm5(1,
			 "%s production_test_initialization_ftm5: Type incompatible! Type = %02X ERROR %08X\n",
			 tag_ftm5, type, ERROR_OP_NOT_ALLOW |
			 ERROR_PROD_TEST_INITIALIZATION);
		return ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_INITIALIZATION;
	}

	res = fts_system_reset_ftm5(info);
	if (res < 0) {
		logError_ftm5(1, "%s production_test_initialization_ftm5: ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_INITIALIZATION);
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}

	logError_ftm5(0, "%s INITIALIZATION command sent... %02X\n", tag_ftm5, type);
	res = writeSysCmd(info, SYS_CMD_SPECIAL, &type, 1);
	if (res < OK) {
		logError_ftm5(1, "%s production_test_initialization_ftm5: ERROR %08X\n",
			 tag_ftm5, (res | ERROR_PROD_TEST_INITIALIZATION));
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}


	logError_ftm5(0, "%s Refresh Sys Info...\n", tag_ftm5);
	res |= readSysInfo(info, 1);	/* need to update the chipInfo in order
				  * to refresh several versions */

	if (res < 0) {
		logError_ftm5(1,
			 "%s production_test_initialization_ftm5: read sys info ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_INITIALIZATION);
		res = (res | ERROR_PROD_TEST_INITIALIZATION);
	}

	return res;
}


/**
  * Perform a FULL (ITO + INIT + DATA CHECK) Mass Production Test of the IC
  * @param pathThresholds name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param saveInit if >0 (possible values: NO_INIT, SPECIAL_PANEL_INIT or
  * SPECIAL_FULL_PANEL_INIT),
  * the Initialization of the IC is executed otherwise it is skipped
  * @param todo pointer to a TestToDo variable which select the test to do
  * @param mpflag MP flag value to write in case of successful test
  * @return OK if success or an error code which specify the type of error
  */
int production_test_main_ftm5(struct fts_ts_info *info, char *pathThresholds, int stop_on_fail, int saveInit,
			 TestToDo *todo, u8 mpflag, enum vts_sensor_test_result *result)
{
	int res, ret;

	logError_ftm5(0, "%s MAIN Production test is starting...\n", tag_ftm5);

	logError_ftm5(0, "%s\n", tag_ftm5);

	logError_ftm5(0, "%s ITO TEST:\n", tag_ftm5);
	res = production_test_ito_ftm5(info, pathThresholds, todo);
	if (res < 0) {
		logError_ftm5(0, "%s Error during ITO TEST! ERROR %08X\n", tag_ftm5, res);
		*result |= VTS_SENSOR_TEST_ITO_FAILED;
		goto END;/* in case of ITO TEST failure is no sense keep going
			 * */
	} else
		logError_ftm5(0, "%s ITO TEST OK!\n", tag_ftm5);


	logError_ftm5(0, "%s\n", tag_ftm5);

	logError_ftm5(0, "%s INITIALIZATION TEST :\n", tag_ftm5);
	if (saveInit != NO_INIT) {
		res = production_test_initialization_ftm5(info, (u8)saveInit);
		if (res < 0) {
			logError_ftm5(0,
				 "%s Error during  INITIALIZATION TEST! ERROR %08X\n",
				 tag_ftm5, res);
			*result |= VTS_SENSOR_TEST_INIT_FAILED;
			if (stop_on_fail)
				goto END;
		} else
			logError_ftm5(0, "%s INITIALIZATION TEST OK!\n", tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s INITIALIZATION TEST :................. SKIPPED\n",
			 tag_ftm5);


	logError_ftm5(0, "%s\n", tag_ftm5);

	if (saveInit != NO_INIT) {
		logError_ftm5(0, "%s Cleaning up...\n", tag_ftm5);
		ret = fts_system_reset_ftm5(info);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_main_ftm5: system reset ERROR %08X\n",
				 tag_ftm5, ret);
			res |= ret;
			if (stop_on_fail)
				goto END;
		}
		logError_ftm5(0, "%s\n", tag_ftm5);
	}

	logError_ftm5(0, "%s PRODUCTION DATA TEST:\n", tag_ftm5);
	ret = production_test_data_ftm5(info, pathThresholds, stop_on_fail, todo);
	if (ret < OK) {
		logError_ftm5(0,
			 "%s Error during PRODUCTION DATA TEST! ERROR %08X\n",
			 tag_ftm5, ret);
		*result |= VTS_SENSOR_TEST_DATA_FAILED;
	} else {
		logError_ftm5(0, "%s PRODUCTION DATA TEST OK!\n", tag_ftm5);

		if (saveInit != NO_INIT) {
			/* save the mp flag to desired value
			 * because data check OK */
			ret = saveMpFlag(info, mpflag);
			if (ret < OK)
				logError_ftm5(0,
					 "%s Error while saving MP FLAG! ERROR %08X\n",
					 tag_ftm5, ret);
			else
				logError_ftm5(0, "%s MP FLAG saving OK!\n", tag_ftm5);
		}
	}

	res |= ret;
	/* the OR is important because if the data test is OK but
	  * the init test fail, the main production test result should = FAIL */


END:
	if (res < OK) {
		logError_ftm5(0,
			 "%s MAIN Production test finished.................FAILED\n",
			 tag_ftm5);
		return res;
	} else {
		logError_ftm5(0,
			 "%s MAIN Production test finished.................OK\n",
			 tag_ftm5);
		return OK;
	}
}

/**
  * Perform all the test selected in a TestTodo variable related to MS raw data
  * (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_raw_ftm5(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret, count_fail = 0;
	MutualSenseFrame msRawFrame;


	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	u16 *adj = NULL;

	int maxAdjH = 0, maxAdjV = 0;
	int i, z;

	msRawFrame.node_data = NULL;

	/************** Mutual Sense Test *************/
	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s MS RAW DATA TEST is starting...\n", tag_ftm5);
	if (todo->MutualRaw == 1 || todo->MutualRawGap == 1 ||
	    todo->MutualRawAdj == 1 || todo->MutualRawMap == 1 ||
	    todo->MutualRawAdjGap == 1 || todo->MutualRawAdjPeak == 1) {
		ret = setScanMode(info, SCAN_MODE_LOCKED, LOCKED_ACTIVE);
		msleep(WAIT_FOR_FRESH_FRAMES);
		ret |= setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
		msleep(WAIT_AFTER_SENSEOFF);
		ret |= getMSFrame3_ftm5(info, MS_RAW, &msRawFrame);
		if (ret < OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: getMSFrame failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			return ret | ERROR_PROD_TEST_DATA;
		}

		logError_ftm5(0, "%s MS RAW MIN MAX TEST:\n", tag_ftm5);
		if (todo->MutualRaw == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 2)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_MIN_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMinMax_ftm5(msRawFrame.node_data,
						msRawFrame.header.force_node,
						msRawFrame.header.sense_node,
						thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 MS RAW failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS RAW MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS RAW MIN MAX TEST:.................OK\n",
					 tag_ftm5);
			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s MS RAW MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);
		logError_ftm5(0, "%s MS RAW MAP MIN MAX TEST:\n", tag_ftm5);
		if (todo->MutualRawMap == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, MS_RAW_EACH_NODE_MIN,
				&thresholds_min, &trows, &tcolumns);
			if (ret < OK || (trows !=
					 msRawFrame.header.force_node ||
					 tcolumns != msRawFrame.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_EACH_NODE_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, MS_RAW_EACH_NODE_MAX,
				&thresholds_max, &trows, &tcolumns);
			if (ret < OK || (trows !=
					 msRawFrame.header.force_node ||
					 tcolumns != msRawFrame.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_EACH_NODE_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = checkLimitsMapTotal_ftm5(msRawFrame.node_data,
				msRawFrame.header.force_node,
				msRawFrame.header.sense_node, thresholds_min,
				thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMaxEachNodeData failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS RAW MAP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else{
				logError_ftm5(0, "%s MS RAW MAP MIN MAX TEST:.................OK\n",
					tag_ftm5);
			}
			if (thresholds_min != NULL) {
				kfree(thresholds_min);
				thresholds_min = NULL;
			}
			if (thresholds_max != NULL) {
				kfree(thresholds_max);
				thresholds_max = NULL;
			}
		} else {
			logError_ftm5(0, "%s MS RAW MAP MIN MAX TEST:.................SKIPPED\n", tag_ftm5);
		}

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s MS RAW GAP TEST:\n", tag_ftm5);
		if (todo->MutualRawGap == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file, MS_RAW_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_GAP failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap_ftm5(msRawFrame.node_data,
					     msRawFrame.header.force_node,
					     msRawFrame.header.sense_node,
					     thresholds[0]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsGap_ftm5 MS RAW failed... ERROR = %08X\n",
					 tag_ftm5, ret);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS RAW GAP TEST:.................OK\n\n",
					 tag_ftm5);
			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s MS RAW GAP TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s MS RAW ADJ TEST:\n", tag_ftm5);
		if ((todo->MutualRawAdj == 1) || (todo->MutualRawAdjGap == 1) ||
			(todo->MutualRawAdjPeak == 1)) {
			logError_ftm5(0, "%s MS RAW ADJ HORIZONTAL TESTs:\n", tag_ftm5);
			ret = computeAdjHorizTotal_ftm5(msRawFrame.node_data,
						   msRawFrame.header.force_node,
						   msRawFrame.header.sense_node,
						   &adj);
			if (ret < OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			if (todo->MutualRawAdj) {
				logError_ftm5(0, "%s MS RAW ADJ HORIZONTAL min/Max:\n",
					tag_ftm5);

				ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_ADJH,
							&thresholds, &trows,
							&tcolumns);
				if (ret < OK || (trows !=
					 msRawFrame.header.force_node ||
					 tcolumns !=
					 msRawFrame.header.sense_node - 1)) {
					logError_ftm5(1,
						"%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_ADJH failed... ERROR %08X\n",
						tag_ftm5, ERROR_PROD_TEST_DATA);
					ret |= ERROR_PROD_TEST_DATA;
					goto ERROR_LIMITS;
				}


				ret = checkLimitsMapAdjTotal_ftm5(adj,
						     msRawFrame.header.
						     force_node,
						     msRawFrame.header.
						     sense_node - 1,
						     thresholds);
				if (ret != OK) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ADJH failed... ERROR COUNT = %d\n",
						 tag_ftm5, ret);
					logError_ftm5(0,
						 "%s MS RAW ADJ HORIZONTAL min/Max:.................FAIL\n\n",
						 tag_ftm5);
					count_fail += 1;
					if (stop_on_fail == 1)
						goto ERROR;
				} else
					logError_ftm5(0,
						 "%s MS RAW ADJ HORIZONTAL min/Max:.................OK\n",
						 tag_ftm5);

				if (thresholds != NULL) {
					kfree(thresholds);
					thresholds = NULL;
				}
			}


			if (todo->MutualRawAdjGap) {
				logError_ftm5(0, "%s MS RAW ADJ HORIZONTAL GAP:\n",
					tag_ftm5);

				ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_ADJH_GAP,
							&thresholds, &trows,
							&tcolumns);
				if (ret < OK || (trows != 1 ||
					 tcolumns != 1)) {
					logError_ftm5(1,
						"%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_ADJH failed... ERROR %08X\n",
						tag_ftm5, ERROR_PROD_TEST_DATA);
					ret |= ERROR_PROD_TEST_DATA;
					goto ERROR_LIMITS;
				}


				ret = checkLimitsGapOffsets(adj,
						     msRawFrame.header.
						     force_node,
						     msRawFrame.header.
						     sense_node - 1,
						     thresholds[0],
						     1,1,1,1);
				if (ret != OK) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ADJH GAP failed...\n",
						 tag_ftm5);
					logError_ftm5(0,
						 "%s MS RAW ADJ HORIZONTAL GAP:.................FAIL\n\n",
						 tag_ftm5);
					count_fail += 1;
					if (stop_on_fail == 1)
						goto ERROR;
				} else
					logError_ftm5(0,
						 "%s MS RAW ADJ HORIZONTAL GAP:.................OK\n",
						 tag_ftm5);

				if (thresholds != NULL) {
					kfree(thresholds);
					thresholds = NULL;
				}
			}


			if (todo->MutualRawAdjPeak) {
				logError_ftm5(0, "%s MS RAW ADJ Peak: Getting max ADJH\n",
					tag_ftm5);
				maxAdjH = abs(adj[msRawFrame.header.force_node
						+ 1]);
				/* skip nodes on the edges */
				for (i = 1; i < msRawFrame.header
					.force_node - 1; i++) {
					for (z = 1; z <
						(msRawFrame.header
						.sense_node - 2); z++){
						if (maxAdjH < abs(adj[(i *
							msRawFrame.header
							.force_node) + z]))
						maxAdjH = abs(adj[(i *
							msRawFrame.header
							.force_node) + z]);
					}

				}
			}

			kfree(adj);
			adj = NULL;

			logError_ftm5(0, "%s MS RAW ADJ VERTICAL TESTs:\n", tag_ftm5);
			ret = computeAdjVertTotal_ftm5(msRawFrame.node_data,
						  msRawFrame.header.force_node,
						  msRawFrame.header.sense_node,
						  &adj);
			if (ret < OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			if (todo->MutualRawAdj) {
				logError_ftm5(0, "%s MS RAW ADJ VERTICAL min/Max:\n",
					tag_ftm5);
				ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_ADJV,
							&thresholds, &trows,
							&tcolumns);
				if (ret < OK || (trows !=
					msRawFrame.header.force_node - 1 ||
					tcolumns != msRawFrame.header.sense_node)) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_ADJV failed... ERROR %08X\n",
						 tag_ftm5, ERROR_PROD_TEST_DATA);
					ret |= ERROR_PROD_TEST_DATA;
					goto ERROR_LIMITS;
				}


				ret = checkLimitsMapAdjTotal_ftm5(adj,
							     msRawFrame.header.
							     force_node - 1,
							     msRawFrame.header.
							     sense_node,
							     thresholds);
				if (ret != OK) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ADJV failed... ERROR COUNT = %d\n",
						 tag_ftm5, ret);
					logError_ftm5(0,
						 "%s MS RAW ADJ VERTICAL min/Max:.................FAIL\n\n",
						 tag_ftm5);
					count_fail += 1;
					if (stop_on_fail == 1)
						goto ERROR;
				} else
					logError_ftm5(0,
						 "%s MS RAW ADJ VERTICAL min/Max:.................OK\n",
						 tag_ftm5);

				if (thresholds != NULL) {
					kfree(thresholds);
					thresholds = NULL;
				}
			}

			if (todo->MutualRawAdjGap) {
				logError_ftm5(0, "%s MS RAW ADJ VERTICAL GAP:\n",
					tag_ftm5);
				ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_ADJV_GAP,
							&thresholds, &trows,
							&tcolumns);
				if (ret < OK || (trows != 1 ||
					tcolumns != 1)) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_ADJV_GAP failed... ERROR %08X\n",
						 tag_ftm5, ERROR_PROD_TEST_DATA);
					ret |= ERROR_PROD_TEST_DATA;
					goto ERROR_LIMITS;
				}


				ret = checkLimitsGapOffsets(adj,
							    msRawFrame.header.
							    force_node - 1,
							    msRawFrame.header.
							    sense_node,
							    thresholds[0],
							    1,1,1,1);
				if (ret != OK) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ADJV GAP failed... ERROR COUNT = %d\n",
						 tag_ftm5, ret);
					logError_ftm5(0,
						 "%s MS RAW ADJ VERTICAL GAP:.................FAIL\n\n",
						 tag_ftm5);
					count_fail += 1;
					if (stop_on_fail == 1)
						goto ERROR;
				} else
					logError_ftm5(0,
						 "%s MS RAW ADJ VERTICAL GAP:.................OK\n",
						 tag_ftm5);

				if (thresholds != NULL) {
					kfree(thresholds);
					thresholds = NULL;
				}
			}


			if (todo->MutualRawAdjPeak) {
				logError_ftm5(0, "%s MS RAW ADJ Peak: Getting max ADJV\n",
					tag_ftm5);
				maxAdjV = abs(adj[(msRawFrame.header.force_node)
						+ 1]);

				/* skip nodes on the edges */
				for (i = 1; i < (msRawFrame.header
						.force_node - 2); i++) {
					for (z = 1; z < msRawFrame.header
						.sense_node - 1; z++) {
						if (maxAdjH < abs(adj[(i *
							msRawFrame.header
							.force_node) + z]))
							maxAdjH = abs(adj[(i *
							     msRawFrame.header
							     .force_node) + z]);

					}
				}



				ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_ADJ_PEAK,
							&thresholds, &trows,
							&tcolumns);
				if (ret < OK || (trows != 1 ||
					tcolumns != 1)) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_ADJV_PEAK failed... ERROR %08X\n",
						 tag_ftm5, ERROR_PROD_TEST_DATA);
					ret |= ERROR_PROD_TEST_DATA;
					goto ERROR_LIMITS;
				}

				logError_ftm5(1, "%s maxAdjH = %d  maxAdjV = %d  threshold = %d\n",
					tag_ftm5, maxAdjH, maxAdjV, thresholds[0]);

				ret = OK;
				if (maxAdjH > maxAdjV) {
					if (maxAdjH > thresholds[0])
						ret = ERROR_PROD_TEST_DATA;
				} else {
					if (maxAdjV > thresholds[0])
						ret = ERROR_PROD_TEST_DATA;
				}

				if (ret != OK) {
					logError_ftm5(1,
						 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ADJV GAP failed... ERROR COUNT = %d\n",
						 tag_ftm5, ret);
					logError_ftm5(0,
						 "%s MS RAW ADJ PEAK:.................FAIL\n\n",
						 tag_ftm5);
					count_fail += 1;
					if (stop_on_fail == 1)
						goto ERROR;
				} else
					logError_ftm5(0,
						 "%s MS RAW ADJ PEAK:.................OK\n",
						 tag_ftm5);

				if (thresholds != NULL) {
					kfree(thresholds);
					thresholds = NULL;
				}

			}


			kfree(adj);
			adj = NULL;
		} else
			logError_ftm5(0,
				 "%s MS RAW ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0, "%s MS RAW DATA TEST:.................SKIPPED\n",
			 tag_ftm5);

	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s MS KEY RAW TEST:\n", tag_ftm5);
	if (todo->MutualKeyRaw == 1) {
		ret = production_test_ms_key_raw_ftm5(info, path_limits);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: production_test_ms_key_raw_ftm5 failed... ERROR = %08X\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (count_fail == 1) {
				logError_ftm5(0,
					 "%s MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",
					 tag_ftm5, count_fail);
				goto ERROR_LIMITS;
			}
		}
	} else
		logError_ftm5(0, "%s MS KEY RAW TEST:.................SKIPPED\n",
			 tag_ftm5);

	ret = production_test_ms_raw_lp(info, path_limits, stop_on_fail, todo);
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: production_test_ms_raw_lp failed... ERROR = %08X\n",
			 tag_ftm5, ret);
		count_fail += 1;
		if (count_fail == 1) {
			logError_ftm5(0,
				 "%s MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",
				 tag_ftm5, count_fail);
			goto ERROR_LIMITS;
		}
	}

ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		if (msRawFrame.node_data != NULL) {
			kfree(msRawFrame.node_data);
			msRawFrame.node_data = NULL;
		}
		logError_ftm5(0,
			 "%s MS RAW DATA TEST finished!.................OK\n",
			 tag_ftm5);
		return OK;
	} else {
		print_frame_short_ftm5("MS Raw frame =", array1dTo2d_short_ftm5(
					  msRawFrame.node_data,
					  msRawFrame.node_data_size,
					  msRawFrame.header.sense_node),
				  msRawFrame.header.force_node,
				  msRawFrame.header.sense_node);

		if (msRawFrame.node_data != NULL)
			kfree(msRawFrame.node_data);
		if (thresholds != NULL)
			kfree(thresholds);
		if (adj != NULL)
			kfree(adj);
		if (thresholds_min != NULL) {
			kfree(thresholds_min);
			thresholds_min = NULL;
		}
		if (thresholds_max != NULL) {
			kfree(thresholds_max);
			thresholds_max = NULL;
		}
		logError_ftm5(0,
			 "%s MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
	}


ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}


/**
  * Perform all the test selected in a TestTodo variable related to MS low power
  * raw data
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_raw_lp(struct fts_ts_info *info, char *path_limits, int stop_on_fail,
			      TestToDo *todo)
{
	int ret, count_fail = 0;
	MutualSenseFrame msRawFrame;


	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	u16 *adj = NULL;
	msRawFrame.node_data = NULL;

	/************** Mutual Sense Test **************/
	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s MS RAW LP DATA TEST:\n", tag_ftm5);
	if (todo->MutualRawLP == 1 || todo->MutualRawGapLP == 1 ||
	    todo->MutualRawAdjLP == 1) {
		ret = setScanMode(info, SCAN_MODE_LOCKED, LOCKED_LP_ACTIVE);
		msleep(WAIT_FOR_FRESH_FRAMES);
		ret |= setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
		msleep(WAIT_AFTER_SENSEOFF);
		ret |= getMSFrame3_ftm5(info, MS_RAW, &msRawFrame);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: getMSFrame failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			return ret | ERROR_PROD_TEST_DATA;
		}

		logError_ftm5(0, "%s MS RAW LP MIN MAX TEST:\n", tag_ftm5);
		if (todo->MutualRawLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_LP_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_LP_MIN_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMinMax_ftm5(msRawFrame.node_data,
						msRawFrame.header.force_node,
						msRawFrame.header.sense_node,
						thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 MS RAW LP failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS RAW LP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS RAW LP MIN MAX TEST:.................OK\n",
					 tag_ftm5);
			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s MS RAW LP MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s MS RAW LP MAP MIN MAX TEST:\n", tag_ftm5);
		if (todo->MutualRawMapLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, MS_RAW_LP_EACH_NODE_MIN,
				&thresholds_min, &trows, &tcolumns);
			if (ret < OK || (trows !=
					 msRawFrame.header.force_node ||
					 tcolumns != msRawFrame.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_LP_EACH_NODE_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, MS_RAW_LP_EACH_NODE_MAX,
				&thresholds_max, &trows, &tcolumns);
			if (ret < OK || (trows !=
					 msRawFrame.header.force_node ||
					 tcolumns != msRawFrame.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_LP_EACH_NODE_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = checkLimitsMapTotal_ftm5(msRawFrame.node_data,
				msRawFrame.header.force_node,
				msRawFrame.header.sense_node, thresholds_min,
				thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMaxEachNodeData failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS RAW LP MAP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else{
				logError_ftm5(0, "%s MS RAW LP MAP MIN MAX TEST:.................OK\n",
					tag_ftm5);
			}
			if (thresholds_min != NULL) {
				kfree(thresholds_min);
				thresholds_min = NULL;
			}
			if (thresholds_max != NULL) {
				kfree(thresholds_max);
				thresholds_max = NULL;
			}
		} else {
			logError_ftm5(0, "%s MS RAW LP MAP MIN MAX TEST:.................SKIPPED\n", tag_ftm5);
		}

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s MS RAW LP GAP TEST:\n", tag_ftm5);
		if (todo->MutualRawGapLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_LP_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_LP_GAP failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap_ftm5(msRawFrame.node_data,
					     msRawFrame.header.force_node,
					     msRawFrame.header.sense_node,
					     thresholds[0]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsGap_ftm5 MS RAW LP failed... ERROR = %08X\n",
					 tag_ftm5, ret);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS RAW LP GAP TEST:.................OK\n\n",
					 tag_ftm5);
			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s MS RAW LP GAP TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s MS RAW LP ADJ TEST:\n", tag_ftm5);
		if (todo->MutualRawAdjLP == 1) {
			logError_ftm5(0, "%s MS RAW LP ADJ HORIZONTAL TEST:\n", tag_ftm5);
			ret = computeAdjHorizTotal_ftm5(msRawFrame.node_data,
						   msRawFrame.header.force_node,
						   msRawFrame.header.sense_node,
						   &adj);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_LP_ADJH,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != msRawFrame.header.force_node ||
					tcolumns !=
					msRawFrame.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_LP_ADJH failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMapAdjTotal_ftm5(adj,
						     msRawFrame.header.
						     force_node,
						     msRawFrame.header.
						     sense_node - 1,
						     thresholds);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsAdj MS RAW LP ADJH failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS RAW LP ADJ HORIZONTAL TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS RAW LP ADJ HORIZONTAL TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;

			kfree(adj);
			adj = NULL;

			logError_ftm5(0, "%s MS RAW LP ADJ VERTICAL TEST:\n", tag_ftm5);
			ret = computeAdjVertTotal_ftm5(msRawFrame.node_data,
						  msRawFrame.header.force_node,
						  msRawFrame.header.sense_node,
						  &adj);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_RAW_LP_ADJV,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != msRawFrame.header.force_node -
					1 || tcolumns !=
					msRawFrame.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_RAW_ADJV failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMapAdjTotal_ftm5(adj,
						     msRawFrame.header.
						     force_node - 1,
						     msRawFrame.header.
						     sense_node, thresholds);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsAdj MS RAW ADJV failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS RAW LP ADJ VERTICAL TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS RAW LP ADJ VERTICAL TEST:.................OK\n",
					 tag_ftm5);
			kfree(thresholds);
			thresholds = NULL;

			kfree(adj);
			adj = NULL;
		} else
			logError_ftm5(0,
				 "%s MS RAW LP ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s MS RAW LP DATA TEST:.................SKIPPED\n",
			 tag_ftm5);

ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		if (msRawFrame.node_data != NULL) {
			kfree(msRawFrame.node_data);
			msRawFrame.node_data = NULL;
		}
		logError_ftm5(0,
			 "%s MS RAW DATA TEST finished!.................OK\n",
			 tag_ftm5);
		return OK;
	} else {
		if (msRawFrame.node_data != NULL) {
			print_frame_short_ftm5("MS Raw LP frame =",
					  array1dTo2d_short_ftm5(
						  msRawFrame.node_data,
						  msRawFrame.
						  node_data_size,
						  msRawFrame.header.
						  sense_node),
					  msRawFrame.header.force_node,
					  msRawFrame.header.sense_node);
			kfree(msRawFrame.node_data);
		}
		if (thresholds != NULL)
			kfree(thresholds);
		if (adj != NULL)
			kfree(adj);
		if (thresholds_min != NULL) {
			kfree(thresholds_min);
			thresholds_min = NULL;
		}
		if (thresholds_max != NULL) {
			kfree(thresholds_max);
			thresholds_max = NULL;
		}
		logError_ftm5(0,
			 "%s MS RAW LP DATA TEST:.................FAIL fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
	}


ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}

/**
  * Perform MS raw test for keys data
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_key_raw_ftm5(struct fts_ts_info *info, char *path_limits)
{
	int ret;
	MutualSenseFrame msRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;

	/************** Mutual Sense Test **************/
	logError_ftm5(0, "%s MS KEY RAW DATA TEST is starting...\n", tag_ftm5);
	ret = setScanMode(info, SCAN_MODE_ACTIVE, 0xFF);
	msleep(WAIT_FOR_FRESH_FRAMES);
	ret |= setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
	msleep(WAIT_AFTER_SENSEOFF);
	ret |= getMSFrame3_ftm5(info, MS_KEY_RAW, &msRawFrame);
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: getMSKeyFrame failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
					MS_KEY_RAW_MIN_MAX, &thresholds, &trows,
					&tcolumns);
	if (ret < 0 || (trows != 1 || tcolumns != 2)) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_KEY_RAW_MIN_MAX failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
	}

	ret = checkLimitsMinMax_ftm5(msRawFrame.node_data,
				msRawFrame.header.force_node,
				msRawFrame.header.sense_node,
				thresholds[0], thresholds[1]);
	if (ret != OK) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 MS KEY RAW failed... ERROR COUNT = %d\n",
			 tag_ftm5, ret);
		goto ERROR;
	} else
		logError_ftm5(0, "%s MS KEY RAW TEST:.................OK\n\n", tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;

	kfree(msRawFrame.node_data);
	msRawFrame.node_data = NULL;
	return OK;

ERROR:
	print_frame_short_ftm5("MS Key Raw frame =", array1dTo2d_short_ftm5(
				  msRawFrame.node_data,
				  msRawFrame.node_data_size,
				  msRawFrame.header.sense_node),
			  msRawFrame.header.force_node,
			  msRawFrame.header.sense_node);
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	logError_ftm5(0, "%s MS KEY RAW TEST:.................FAIL\n\n", tag_ftm5);
	return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;

ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}

/**
  * Perform all the info->tests_ftm5 selected in a TestTodo variable related to MS Init
  * data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_cx_ftm5(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;
	TotMutualSenseData totCompData;

	u8 *adjhor = NULL;

	u8 *adjvert = NULL;

	u16 container;
	/* u16 *total_cx = NULL; */
	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;


	/* MS CX TEST */
	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s MS CX Testes are starting...\n", tag_ftm5);

	ret = readMutualSenseCompensationData_ftm5(info, LOAD_CX_MS_TOUCH, &msCompData);
	/* read MS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readMutualSenseCompensationData_ftm5 failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = readTotMutualSenseCompensationData(info, LOAD_PANEL_CX_TOT_MS_TOUCH,
						 &totCompData);
	/* read  TOT MS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readTotMutualSenseCompensationData failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return ret | ERROR_PROD_TEST_DATA;
	}

	logError_ftm5(0, "%s MS CX1 TEST:\n", tag_ftm5);
	if (todo->MutualCx1 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX1_MIN_MAX, &thresholds,
						&trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX1_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (u16)msCompData.cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 MS CX1 failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0, "%s MS CX1 TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0, "%s MS CX1 TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0, "%s MS CX1 TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;

	logError_ftm5(0, "%s MS CX2 MIN MAX TEST:\n", tag_ftm5);
	if (todo->MutualCx2 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_MAP_MIN, &thresholds_min,
						&trows, &tcolumns);
						/* load min thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_MAP_MAX, &thresholds_max,
						&trows, &tcolumns);
						/* load max thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     thresholds_min, thresholds_max);
					 /* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 MS CX2 MIN MAX failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS CX2 MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS CX2 MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s MS CX2 MIN MAX TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s MS CX2 ADJ TEST:\n", tag_ftm5);
	if (todo->MutualCx2Adj == 1) {
		/* MS CX2 ADJ HORIZ */
		logError_ftm5(0, "%s MS CX2 ADJ HORIZ TEST:\n", tag_ftm5);

		ret = computeAdjHoriz_ftm5(msCompData.node_data,
				      msCompData.header.force_node,
				      msCompData.header.sense_node,
				      &adjhor);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s MS CX2 ADJ HORIZ computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node - 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_ADJH_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjhor, msCompData.header.force_node,
					msCompData.header.sense_node - 1,
					thresholds_max);
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 CX2 ADJH failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS CX2 ADJ HORIZ TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS CX2 ADJ HORIZ TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;

		/* MS CX2 ADJ VERT */
		logError_ftm5(0, "%s MS CX2 ADJ VERT TEST:\n", tag_ftm5);

		ret = computeAdjVert_ftm5(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     &adjvert);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjVert_ftm5 failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s MS CX2 ADJ VERT computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node - 1 ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_ADJV_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjvert, msCompData.header.force_node -
					1, msCompData.header.sense_node - 1,
					thresholds_max);
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 CX2 ADJV failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS CX2 ADJ HORIZ TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS CX2 ADJ VERT TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		logError_ftm5(0, "%s MS CX2 ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* START OF TOTAL CHECK */
	logError_ftm5(0, "%s MS TOTAL CX TEST:\n", tag_ftm5);

	if (todo->MutualCxTotal == 1 || todo->MutualCxTotalAdj == 1) {
		logError_ftm5(0, "%s MS TOTAL CX MIN MAX TEST:\n", tag_ftm5);
		if (todo->MutualCxTotal == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(totCompData.node_data,
						  totCompData.header.force_node,
						  totCompData.header.sense_node,
						  thresholds_min,
						  thresholds_max);
			/* check the limits */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5  MS TOTAL CX TEST failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS TOTAL CX MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS TOTAL CX MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s MS TOTAL CX MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);


		logError_ftm5(0, "%s MS TOTAL CX ADJ TEST:\n", tag_ftm5);
		if (todo->MutualCxTotalAdj == 1) {
			/* MS TOTAL CX ADJ HORIZ */
			logError_ftm5(0, "%s MS TOTAL CX ADJ HORIZ TEST:\n", tag_ftm5);

			ret = computeAdjHorizTotal_ftm5(totCompData.node_data,
						   totCompData.header.force_node,
						   totCompData.header.sense_node,
						   &total_adjhor);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s MS TOTAL CX ADJ HORIZ computed!\n",
				 tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_ADJH_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_ADJH_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjhor,
						     totCompData.header.
						     force_node,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 MS TOTAL CX ADJH failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ HORIZ TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ HORIZ TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;

			/* MS TOTAL CX ADJ VERT */
			logError_ftm5(0, "%s MS TOTAL CX ADJ VERT TEST:\n", tag_ftm5);

			ret = computeAdjVertTotal_ftm5(totCompData.node_data,
						  totCompData.header.force_node,
						  totCompData.header.sense_node,
						  &total_adjvert);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s MS TOTAL CX ADJ VERT computed!\n", tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_ADJV_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_ADJV_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjvert,
						     totCompData.header.
						     force_node - 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 MS TOTAL CX ADJV failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ HORIZ TEST:.................FAIL\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ VERT TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			logError_ftm5(0,
				 "%s MS TOTAL CX ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);

		kfree(totCompData.node_data);
		totCompData.node_data = NULL;
	} else
		logError_ftm5(0, "%s MS TOTAL CX TEST:.................SKIPPED\n",
			 tag_ftm5);



	if ((todo->MutualCx1LP | todo->MutualCx2LP | todo->MutualCx2AdjLP |
	     todo->MutualCxTotalLP | todo->MutualCxTotalAdjLP) == 1) {
		ret = production_test_ms_cx_lp(info, path_limits, stop_on_fail, todo);
		if (ret < OK) {
			count_fail += 1;
			logError_ftm5(1,
				 "%s production_test_data_ftm5: production_test_cx_lp failed... ERROR = %08X\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS CX testes finished!.................FAILED  fails_count = %d\n\n",
				 tag_ftm5, count_fail);
			return ret;
		}
	} else
		logError_ftm5(0, "%s MS CX LP TEST:.................SKIPPED\n",
			 tag_ftm5);


	if ((todo->MutualKeyCx1 | todo->MutualKeyCx2 |
	     todo->MutualKeyCxTotal) == 1) {
		ret = production_test_ms_key_cx_ftm5(info, path_limits, stop_on_fail,
						todo);
		if (ret < 0) {
			count_fail += 1;
			logError_ftm5(1,
				 "%s production_test_data_ftm5: production_test_ms_key_cx_ftm5 failed... ERROR = %08X\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS CX testes finished!.................FAILED  fails_count = %d\n\n",
				 tag_ftm5, count_fail);
			return ret;
		}
	} else
		logError_ftm5(0, "%s MS KEY CX TEST:.................SKIPPED\n",
			 tag_ftm5);



ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		logError_ftm5(0, "%s MS CX testes finished!.................OK\n",
			 tag_ftm5);
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return OK;
	} else {
		print_frame_i8("MS Init Data (Cx2) =", array1dTo2d_i8(
				       msCompData.node_data,
				       msCompData.node_data_size,
				       msCompData.header.sense_node),
			       msCompData.header.force_node,
			       msCompData.header.sense_node);
		print_frame_short_ftm5(" TOT MS Init Data (Cx) =", array1dTo2d_short_ftm5(
					  totCompData.node_data,
					  totCompData.node_data_size,
					  totCompData.header.sense_node),
				  totCompData.header.force_node,
				  totCompData.header.sense_node);
		logError_ftm5(0,
			 "%s MS CX testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (totCompData.node_data != NULL)
			kfree(totCompData.node_data);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (msCompData.node_data != NULL)
			kfree(msCompData.node_data);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (totCompData.node_data != NULL)
		kfree(totCompData.node_data);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	return ret;
}

/**
  * Perform all the info->tests_ftm5 selected in a TestTodo variable related to MS Init
  * data of the keys
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  *  otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_key_cx_ftm5(struct fts_ts_info *info, char *path_limits, int stop_on_fail,
			      TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int num_keys = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;
	TotMutualSenseData totCompData;


	short container;


	/* MS CX TEST */
	logError_ftm5(0, "%s MS KEY CX Testes are starting...\n", tag_ftm5);

	ret = readMutualSenseCompensationData_ftm5(info, LOAD_CX_MS_KEY, &msCompData);
	/* read MS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readMutualSenseCompensationData_ftm5 failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	if (msCompData.header.force_node > msCompData.header.sense_node)
		/* the meaningful data are only in the first row,
		  * the other rows are only a copy of the first one */
		num_keys = msCompData.header.force_node;
	else
		num_keys = msCompData.header.sense_node;

	logError_ftm5(0, "%s MS KEY CX1 TEST:\n", tag_ftm5);
	if (todo->MutualKeyCx1 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_KEY_CX1_MIN_MAX, &thresholds,
						&trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_KEY_CX1_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)msCompData.cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);	/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 MS CX1 failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS KEY CX1 TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS KEY CX1 TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0, "%s MS KEY CX1 TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;

	logError_ftm5(0, "%s MS KEY CX2 TEST:\n", tag_ftm5);
	if (todo->MutualKeyCx2 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_KEY_CX2_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load min thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node  ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_KEY_CX2_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_KEY_CX2_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load max thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node  ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_KEY_CX2_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     thresholds_min, thresholds_max);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 MS KEY CX2 failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS KEY CX2 TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS KEY CX2 TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0, "%s MS CX2 TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* START OF TOTAL CHECK */
	logError_ftm5(0, "%s MS KEY TOTAL CX TEST:\n", tag_ftm5);

	if (todo->MutualKeyCxTotal == 1) {
		ret = readTotMutualSenseCompensationData(info, 
			LOAD_PANEL_CX_TOT_MS_KEY, &totCompData);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeTotalCx failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_KEY_TOTAL_CX_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load min thresholds */
		if (ret < 0 || (trows != totCompData.header.force_node ||
				tcolumns != totCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_KEY_TOTAL_CX_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_KEY_TOTAL_CX_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load max thresholds */
		if (ret < 0 || (trows != totCompData.header.force_node  ||
				tcolumns != totCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_KEY_TOTAL_CX_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapTotal_ftm5(totCompData.node_data,
					  totCompData.header.force_node,
					  totCompData.header.sense_node,
					  thresholds_min, thresholds_max);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5  MS TOTAL KEY CX TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS KEY TOTAL CX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS KEY TOTAL CX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;

		kfree(totCompData.node_data);
		totCompData.node_data = NULL;
	} else
		logError_ftm5(0,
			 "%s MS KEY TOTAL CX TEST:.................SKIPPED\n",
			 tag_ftm5);


ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		logError_ftm5(0,
			 "%s MS KEY CX testes finished!.................OK\n",
			 tag_ftm5);
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return OK;
	} else {
		print_frame_i8("MS Key Init Data (Cx2) =", array1dTo2d_i8(
				       msCompData.node_data,
				       msCompData.node_data_size,
				       msCompData.header.sense_node),
			       msCompData.header.force_node,
			       msCompData.header.sense_node);
		logError_ftm5(0,
			 "%s MS Key CX testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (msCompData.node_data != NULL)
			kfree(msCompData.node_data);
		if (totCompData.node_data != NULL)
			kfree(totCompData.node_data);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	if (totCompData.node_data != NULL)
		kfree(totCompData.node_data);
	return ret;
}

/**
  * Perform all the info->tests_ftm5 selected in a TestTodo variable related to MS LowPower
  * Init data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_cx_lp(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;
	TotMutualSenseData totCompData;

	u8 *adjhor = NULL;

	u8 *adjvert = NULL;

	u16 container;
	/* u16 *total_cx = NULL; */
	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;


	/* MS CX TEST */
	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s MS LP CX Testes are starting...\n", tag_ftm5);

	ret = readMutualSenseCompensationData_ftm5(info, LOAD_CX_MS_LOW_POWER, &msCompData);
	/* read MS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readMutualSenseCompensationData_ftm5 failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = readTotMutualSenseCompensationData(info, LOAD_PANEL_CX_TOT_MS_LOW_POWER,
						 &totCompData);
	/* read  TOT MS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readTotMutualSenseCompensationData failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return ret | ERROR_PROD_TEST_DATA;
	}

	logError_ftm5(0, "%s MS LP CX1 TEST:\n", tag_ftm5);
	if (todo->MutualCx1LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX1_LP_MIN_MAX, &thresholds,
						&trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX1_LP_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (u16)msCompData.cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 MS LP CX1 failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0, "%s MS LP CX1 TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0, "%s MS LP CX1 TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0, "%s MS LP CX1 TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;

	logError_ftm5(0, "%s MS LP CX2 MIN MAX TEST:\n", tag_ftm5);
	if (todo->MutualCx2LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_LP_MAP_MIN, &thresholds_min,
						&trows, &tcolumns);
						/* load min thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_LP_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_LP_MAP_MAX, &thresholds_max,
						&trows, &tcolumns);
						/* load max thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_LP_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     thresholds_min, thresholds_max);
					 /* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 MS LP CX2 MIN MAX failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS LP CX2 MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS LP CX2 MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s MS LP CX2 MIN MAX TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s MS LP CX2 ADJ TEST:\n", tag_ftm5);
	if (todo->MutualCx2AdjLP == 1) {
		/* MS CX2 ADJ HORIZ */
		logError_ftm5(0, "%s MS LP CX2 ADJ HORIZ TEST:\n", tag_ftm5);

		ret = computeAdjHoriz_ftm5(msCompData.node_data,
				      msCompData.header.force_node,
				      msCompData.header.sense_node,
				      &adjhor);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s MS LP CX2 ADJ HORIZ computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_ADJH_LP_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node - 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_ADJH_LP_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjhor, msCompData.header.force_node,
					msCompData.header.sense_node - 1,
					thresholds_max);
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 CX2 ADJH LP failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS LP CX2 ADJ HORIZ TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS LP CX2 ADJ HORIZ TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;

		/* MS CX2 ADJ VERT */
		logError_ftm5(0, "%s MS LP CX2 ADJ VERT TEST:\n", tag_ftm5);

		ret = computeAdjVert_ftm5(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     &adjvert);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjVert_ftm5 failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s MS LP CX2 ADJ VERT computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						MS_CX2_ADJV_LP_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node - 1 ||
				tcolumns != msCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_CX2_ADJV_LP_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjvert, msCompData.header.force_node -
					1, msCompData.header.sense_node - 1,
					thresholds_max);
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 CX2 ADJV LP failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s MS LP CX2 ADJ HORIZ TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s MS LP CX2 ADJ VERT TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		logError_ftm5(0, "%s MS LP CX2 ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* START OF TOTAL CHECK */
	logError_ftm5(0, "%s MS TOTAL LP CX TEST:\n", tag_ftm5);

	if (todo->MutualCxTotalLP == 1 || todo->MutualCxTotalAdjLP == 1) {
		logError_ftm5(0, "%s MS TOTAL LP CX MIN MAX TEST:\n", tag_ftm5);
		if (todo->MutualCxTotalLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_LP_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_LP_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_LP_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_LP_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(totCompData.node_data,
						  totCompData.header.force_node,
						  totCompData.header.sense_node,
						  thresholds_min,
						  thresholds_max);
			/* check the limits */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5  MS TOTAL CX LP TEST failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS TOTAL CX LP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS TOTAL CX LP MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s MS TOTAL CX LP MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);


		logError_ftm5(0, "%s MS TOTAL CX ADJ LP TEST:\n", tag_ftm5);
		if (todo->MutualCxTotalAdjLP == 1) {
			/* MS TOTAL CX ADJ HORIZ */
			logError_ftm5(0, "%s MS TOTAL CX ADJ HORIZ LP TEST:\n", tag_ftm5);

			ret = computeAdjHorizTotal_ftm5(totCompData.node_data,
						   totCompData.header.force_node,
						   totCompData.header.sense_node,
						   &total_adjhor);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s MS TOTAL CX ADJ HORIZ LP computed!\n",
				 tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_ADJH_LP_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_ADJH_LP_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjhor,
						     totCompData.header.
						     force_node,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 MS TOTAL CX ADJH LP failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ HORIZ LP TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ HORIZ LP TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;

			/* MS TOTAL CX ADJ VERT */
			logError_ftm5(0, "%s MS TOTAL CX ADJ VERT LP TEST:\n", tag_ftm5);

			ret = computeAdjVertTotal_ftm5(totCompData.node_data,
						  totCompData.header.force_node,
						  totCompData.header.sense_node,
						  &total_adjvert);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s MS TOTAL CX ADJ VERT LP computed!\n", tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							MS_TOTAL_CX_ADJV_LP_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 MS_TOTAL_CX_ADJV_LP_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjvert,
						     totCompData.header.
						     force_node - 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 MS TOTAL CX ADJV failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ HORIZ LP TEST:.................FAIL\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s MS TOTAL CX ADJ VERT LP TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			logError_ftm5(0,
				 "%s MS TOTAL CX ADJ LP TEST:.................SKIPPED\n",
				 tag_ftm5);

		kfree(totCompData.node_data);
		totCompData.node_data = NULL;
	} else
		logError_ftm5(0, "%s MS TOTAL CX LP TEST:.................SKIPPED\n",
			 tag_ftm5);



ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		logError_ftm5(0, "%s MS LP CX testes finished!.................OK\n",
			 tag_ftm5);
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return OK;
	} else {
		print_frame_i8("MS LP Init Data (Cx2) =", array1dTo2d_i8(
				       msCompData.node_data,
				       msCompData.node_data_size,
				       msCompData.header.sense_node),
			       msCompData.header.force_node,
			       msCompData.header.sense_node);
		print_frame_short_ftm5(" TOT MS LP Init Data (Cx) =", array1dTo2d_short_ftm5(
					  totCompData.node_data,
					  totCompData.node_data_size,
					  totCompData.header.sense_node),
				  totCompData.header.force_node,
				  totCompData.header.sense_node);
		logError_ftm5(0,
			 "%s MS LP CX testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (totCompData.node_data != NULL)
			kfree(totCompData.node_data);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (msCompData.node_data != NULL)
			kfree(msCompData.node_data);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (totCompData.node_data != NULL)
		kfree(totCompData.node_data);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	return ret;
}

/**
  * Perform all the test selected in a TestTodo variable related to SS raw data
  * (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_raw_ftm5(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int rows, columns;

	SelfSenseFrame ssRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	/* SS TEST */
	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s SS RAW Testes are starting...\n", tag_ftm5);

	/************** Self Sense Test **************/

	logError_ftm5(0, "%s Getting SS Frame...\n", tag_ftm5);
	ret = setScanMode(info, SCAN_MODE_LOCKED, LOCKED_ACTIVE);
	msleep(WAIT_FOR_FRESH_FRAMES);
	ret |= setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
	msleep(WAIT_AFTER_SENSEOFF);
	ret |= getSSFrame3(info, SS_RAW, &ssRawFrame);
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: getSSFrame failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	/* SS RAW (PROXIMITY) FORCE TEST */
	logError_ftm5(0, "%s SS RAW FORCE TEST:\n", tag_ftm5);



	if (todo->SelfForceRaw == 1 || todo->SelfForceRawGap == 1
		|| todo->SelfForceRawMap == 1) {
		columns = 1;	/* there are no data for the sense channels
				  * because is a force frame */
		rows = ssRawFrame.header.force_node;

		logError_ftm5(0, "%s SS RAW FORCE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceRaw == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_FORCE_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_FORCE_MIN_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax_ftm5(ssRawFrame.force_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW (PROXIMITY) FORCE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw force frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.force_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW FORCE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW FORCE MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		logError_ftm5(0, "%s SS RAW FORCE MAP MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceRawMap == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_FORCE_EACH_NODE_MIN,
				&thresholds_min, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_FORCE_EACH_NODE_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_FORCE_EACH_NODE_MAX,
				&thresholds_max, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_FORCE_EACH_NODE_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(ssRawFrame.force_data, rows,
						columns, thresholds_min,
						thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW FORCE MAP failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW FORCE MAP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw force frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.force_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW FORCE MAP MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			if (thresholds_min != NULL) {
				kfree(thresholds_min);
				thresholds_min = NULL;
			}
			if (thresholds_max != NULL) {
				kfree(thresholds_max);
				thresholds_max = NULL;
			}
		} else
			logError_ftm5(0,
				 "%s SS RAW FORCE MAP MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s SS RAW FORCE GAP TEST:\n", tag_ftm5);
		if (todo->SelfForceRawGap == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_FORCE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_FORCE_GAP failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap_ftm5(ssRawFrame.force_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsGap_ftm5 SS RAW FORCE GAP failed... ERROR = %08X\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW FORCE GAP TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw force frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.force_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW FORCE GAP TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW FORCE GAP TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		kfree(ssRawFrame.force_data);
		ssRawFrame.force_data = NULL;
	} else
		logError_ftm5(0,
			 "%s SS RAW FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s\n", tag_ftm5);
	/* SS RAW (PROXIMITY) SENSE TEST */
	logError_ftm5(0, "%s SS RAW SENSE TEST:\n", tag_ftm5);

	if (todo->SelfSenseRaw == 1 || todo->SelfSenseRawGap == 1 ||
		todo->SelfSenseRawMap == 1) {
		columns = ssRawFrame.header.sense_node;
		rows = 1; /* there are no data for the force channels
			  *  because is a sense frame */

		logError_ftm5(0, "%s SS RAW SENSE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseRaw == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_SENSE_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_SENSE_MIN_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax_ftm5(ssRawFrame.sense_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW SENSE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW SENSE MIN MAX TEST:.................FAIL\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw sense frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.sense_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW SENSE MIN MAX TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW SENSE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s SS RAW SENSE MAP MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseRawMap == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_SENSE_EACH_NODE_MIN,
				&thresholds_min, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_SENSE_EACH_NODE_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_SENSE_EACH_NODE_MAX,
				&thresholds_max, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_SENSE_EACH_NODE_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(ssRawFrame.sense_data, rows,
						columns, thresholds_min,
						thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW SENSE MAP failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW SENSE MAP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw sense frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.sense_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW SENSE MAP MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			if (thresholds_min != NULL) {
				kfree(thresholds_min);
				thresholds_min = NULL;
			}
			if (thresholds_max != NULL) {
				kfree(thresholds_max);
				thresholds_max = NULL;
			}
		} else
			logError_ftm5(0,
				 "%s SS RAW SENSE MAP MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s SS RAW SENSE GAP TEST:\n", tag_ftm5);
		if (todo->SelfSenseRawGap == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_SENSE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_SENSE_GAP failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap_ftm5(ssRawFrame.sense_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsGap_ftm5 SS RAW SENSE GAP failed... ERROR = %08X\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW SENSE GAP TEST:.................FAIL\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw sense frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.sense_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW SENSE GAP TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW SENSE GAP TEST:.................SKIPPED\n",
				 tag_ftm5);

		kfree(ssRawFrame.sense_data);
		ssRawFrame.sense_data = NULL;
	} else
		logError_ftm5(0,
			 "%s SS RAW SENSE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	ret = production_test_ss_raw_lp(info, path_limits, stop_on_fail, todo);
	if (ret < OK) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: production_test_ss_raw_lp failed... ERROR = %08X\n",
			 tag_ftm5, ret);
		count_fail += 1;
	}

	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		logError_ftm5(0, "%s SS RAW testes finished!.................OK\n\n",
			 tag_ftm5);
		return OK;
	} else {
		logError_ftm5(0,
			 "%s SS RAW testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (ssRawFrame.force_data != NULL)
		kfree(ssRawFrame.force_data);
	if (ssRawFrame.sense_data != NULL)
		kfree(ssRawFrame.sense_data);
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);

	return ret;
}


/**
  * Perform all the test selected in a TestTodo variable related to SS raw data
  * low power
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  *  otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_raw_lp(struct fts_ts_info *info, char *path_limits, int stop_on_fail,
			      TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int rows, columns;

	SelfSenseFrame ssRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	/* SS TEST */
	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s SS RAW LP Testes are starting...\n", tag_ftm5);

	/************** Self Sense Test **************/

	logError_ftm5(0, "%s Getting SS LP Frame...\n", tag_ftm5);
	ret = setScanMode(info, SCAN_MODE_LOCKED, LOCKED_LP_DETECT);
	msleep(WAIT_FOR_FRESH_FRAMES);
	ret |= setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
	msleep(WAIT_AFTER_SENSEOFF);
	ret |= getSSFrame3(info, SS_DETECT_RAW, &ssRawFrame);
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: getSSFrame failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	/* SS RAW (PROXIMITY) FORCE TEST */
	logError_ftm5(0, "%s SS RAW LP FORCE TEST:\n", tag_ftm5);



	if ((todo->SelfForceRawLP == 1 || todo->SelfForceRawGapLP == 1 ||
		todo->SelfForceRawMapLP == 1) &&
		ssRawFrame.header.force_node != 0) {
		columns = 1;	/* there are no data for the sense channels
				  *  because is a force frame */
		rows = ssRawFrame.header.force_node;

		logError_ftm5(0, "%s SS RAW LP FORCE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceRawLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_LP_FORCE_MIN_MAX,
							&thresholds,
							&trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_FORCE_MIN_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax_ftm5(ssRawFrame.force_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW LP FORCE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw LP force frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.force_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW LP FORCE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW LP FORCE MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		logError_ftm5(0, "%s SS RAW LP FORCE MAP MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceRawMapLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_LP_FORCE_EACH_NODE_MIN,
				&thresholds_min, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_LP_FORCE_EACH_NODE_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_LP_FORCE_EACH_NODE_MAX,
				&thresholds_max, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_LP_FORCE_EACH_NODE_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(ssRawFrame.force_data, rows,
						columns, thresholds_min,
						thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW LP FORCE MAP failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW LP FORCE MAP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw LP force frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.force_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW FORCE LP MAP MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			if (thresholds_min != NULL) {
				kfree(thresholds_min);
				thresholds_min = NULL;
			}
			if (thresholds_max != NULL) {
				kfree(thresholds_max);
				thresholds_max = NULL;
			}
		} else
			logError_ftm5(0,
				 "%s SS RAW FORCE LP MAP MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s SS RAW LP FORCE GAP TEST:\n", tag_ftm5);
		if (todo->SelfForceRawGapLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_LP_FORCE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_FORCE_GAP failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap_ftm5(ssRawFrame.force_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsGap_ftm5 SS RAW FORCE GAP failed... ERROR = %08X\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW LP FORCE GAP TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw LP force frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.force_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW LP FORCE GAP TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW LP FORCE GAP TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		kfree(ssRawFrame.force_data);
		ssRawFrame.force_data = NULL;
	} else
		logError_ftm5(0,
			 "%s SS RAW LP FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s\n", tag_ftm5);
	/* SS RAW (PROXIMITY) SENSE TEST */
	logError_ftm5(0, "%s SS RAW LP SENSE TEST:\n", tag_ftm5);

	if ((todo->SelfSenseRawLP == 1 || todo->SelfSenseRawGapLP == 1 ||
		todo->SelfSenseRawMapLP == 1) &&
		ssRawFrame.header.sense_node != 0) {
		columns = ssRawFrame.header.sense_node;
		rows = 1; /* there are no data for the force channels
			  * because is a sense frame */

		logError_ftm5(0, "%s SS RAW LP SENSE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseRawLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_LP_SENSE_MIN_MAX,
							&thresholds,
							&trows, &tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 2)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_LP_SENSE_MIN_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax_ftm5(ssRawFrame.sense_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW LP SENSE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW LP SENSE MIN MAX TEST:.................FAIL\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw LP sense frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.sense_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW SENSE MIN MAX TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW LP SENSE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s SS RAW LP SENSE MAP MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseRawMapLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_LP_SENSE_EACH_NODE_MIN,
				&thresholds_min, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_LP_SENSE_EACH_NODE_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			ret = parseProductionTestLimits_ftm5(info, path_limits,
				&info->limit_file, SS_RAW_LP_SENSE_EACH_NODE_MAX,
				&thresholds_max, &trows, &tcolumns);
			if (ret < OK || (trows != rows ||
					 tcolumns != columns)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_LP_SENSE_EACH_NODE_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(ssRawFrame.sense_data, rows,
						columns, thresholds_min,
						thresholds_max);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS RAW LP SENSE MAP failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW LP SENSE MAP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw LP sense frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.sense_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW LP SENSE MAP MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			if (thresholds_min != NULL) {
				kfree(thresholds_min);
				thresholds_min = NULL;
			}
			if (thresholds_max != NULL) {
				kfree(thresholds_max);
				thresholds_max = NULL;
			}
		} else
			logError_ftm5(0,
				 "%s SS RAW LP SENSE MAP MIN MAX TEST:.................SKIPPED\n\n",
				 tag_ftm5);

		logError_ftm5(0, "%s\n", tag_ftm5);
		logError_ftm5(0, "%s SS RAW LP SENSE GAP TEST:\n", tag_ftm5);
		if (todo->SelfSenseRawGapLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_RAW_LP_SENSE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_RAW_LP_SENSE_GAP failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap_ftm5(ssRawFrame.sense_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsGap_ftm5 SS RAW LP SENSE GAP failed... ERROR = %08X\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS RAW LP SENSE GAP TEST:.................FAIL\n",
					 tag_ftm5);
				count_fail += 1;
				print_frame_short_ftm5("SS Raw LP sense frame =",
						  array1dTo2d_short_ftm5(
							  ssRawFrame.sense_data,
							  rows *
							  columns,
							  columns), rows,
						  columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				logError_ftm5(0,
					 "%s SS RAW LP SENSE GAP TEST:.................OK\n",
					 tag_ftm5);

			kfree(thresholds);
			thresholds = NULL;
		} else
			logError_ftm5(0,
				 "%s SS RAW LP SENSE GAP TEST:.................SKIPPED\n",
				 tag_ftm5);

		kfree(ssRawFrame.sense_data);
		ssRawFrame.sense_data = NULL;
	} else
		logError_ftm5(0,
			 "%s SS RAW LP SENSE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		logError_ftm5(0,
			 "%s SS RAW LP testes finished!.................OK\n\n",
			 tag_ftm5);
		return OK;
	} else {
		logError_ftm5(0,
			 "%s SS RAW LP testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (ssRawFrame.force_data != NULL)
		kfree(ssRawFrame.force_data);
	if (ssRawFrame.sense_data != NULL)
		kfree(ssRawFrame.sense_data);
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	return ret;
}

/**
  * Perform all the info->tests_ftm5 selected in a TestTodo variable related to SS Init
  * data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_ix_cx_ftm5(struct fts_ts_info *info, char *path_limits, int stop_on_fail,
			     TestToDo *todo)
{
	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	SelfSenseData ssCompData;
	TotSelfSenseData totCompData;

	u8 *adjhor = NULL;
	u8 *adjvert = NULL;

	short container;

	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;

	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s SS IX CX testes are starting...\n", tag_ftm5);
	ret = readSelfSenseCompensationData_ftm5(info, LOAD_CX_SS_TOUCH, &ssCompData);
	/* read the SS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readSelfSenseCompensationData_ftm5 failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = readTotSelfSenseCompensationData(info, LOAD_PANEL_CX_TOT_SS_TOUCH,
					       &totCompData);
	/* read the TOT SS compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readTotSelfSenseCompensationData failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		kfree(ssCompData.ix2_fm);
		kfree(ssCompData.ix2_sn);
		kfree(ssCompData.cx2_fm);
		kfree(ssCompData.cx2_sn);
		return ret | ERROR_PROD_TEST_DATA;
	}

	/************* SS FORCE IX **************/
	/* SS IX1 FORCE TEST */
	logError_ftm5(0, "%s SS IX1 FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceIx1 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX1_FORCE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX1_FORCE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		container = (short)ssCompData.f_ix1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS IX1 FORCE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX1 FORCE TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS IX1 FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;
	/* SS IX2 FORCE TEST */
	logError_ftm5(0, "%s SS IX2 FORCE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfForceIx2 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_FORCE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_FORCE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_FORCE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_FORCE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapFromU(ssCompData.ix2_fm,
					  ssCompData.header.force_node, 1,
					  thresholds_min,
					  thresholds_max);	/* check the
								 * values with
								 * thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 FORCE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 FORCE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 FORCE MIN MAX TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS IX2 FORCE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfForceIx2Adj == 1) {
		/* SS IX2 FORCE ADJV TEST */
		logError_ftm5(0, "%s SS IX2 FORCE ADJVERT TEST:\n", tag_ftm5);
		ret = computeAdjVertFromU(ssCompData.ix2_fm,
					  ssCompData.header.force_node, 1,
					  &adjvert);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS IX2 FORCE ADJV failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS IX2 FORCE ADJV computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);	/* load the max
								 * thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjvert, ssCompData.header.force_node -
					1, 1, thresholds_max);	/* check the
								 * values with
								 * thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 FORCE ADJV TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 FORCE ADJV TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 FORCE ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* SS TOTAL FORCE IX */
	logError_ftm5(0, "%s SS TOTAL IX FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceIxTotal == 1 || todo->SelfForceIxTotalAdj == 1) {
		logError_ftm5(0, "%s SS TOTAL IX FORCE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceIxTotal == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_FORCE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
						/* load the min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_FORCE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_FORCE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
						/* load the max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_FORCE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotalFromU(totCompData.ix_fm,
						       totCompData.header.
						       force_node, 1,
						       thresholds_min,
						       thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5  SS TOTAL IX FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX FORCE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX FORCE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX FORCE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s SS TOTAL IX FORCE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfForceIxTotalAdj == 1) {
			/* SS TOTAL IX FORCE ADJV TEST */
			logError_ftm5(0, "%s SS TOTAL IX FORCE ADJVERT TEST:\n",
				 tag_ftm5);
			ret = computeAdjVertTotalFromU(totCompData.ix_fm,
						       totCompData.header.
						       force_node, 1,
						       &total_adjvert);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS TOTAL IX FORCE ADJV failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s SS TOTAL IX FORCE ADJV computed!\n",
				 tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_FORCE_ADJV_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_FORCE_ADJV_MAP_MAX... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjvert,
						     totCompData.header.
						     force_node - 1, 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL IX FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX FORCE ADJV TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX FORCE ADJV TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX FORCE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL IX FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);


	/************** SS SENSE IX **************/
	/* SS IX1 SENSE TEST */
	logError_ftm5(0, "%s SS IX1 SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseIx1 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX1_SENSE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX1_SENSE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.s_ix1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS IX1 SENSE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX1 SENSE TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS IX1 SENSE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;
	/* SS IX2 SENSE TEST */
	logError_ftm5(0, "%s SS IX2 SENSE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfSenseIx2 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_SENSE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_SENSE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_SENSE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapFromU(ssCompData.ix2_sn, 1,
					  ssCompData.header.sense_node,
					  thresholds_min, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 SENSE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 SENSE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 SENSE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 SENSE MIN MAX TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS IX2 SENSE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfSenseIx2Adj == 1) {
		/* SS IX2 SENSE ADJH TEST */
		logError_ftm5(0, "%s SS IX2 SENSE ADJHORIZ TEST:\n", tag_ftm5);
		ret = computeAdjHorizFromU(ssCompData.ix2_sn, 1,
					   ssCompData.header.sense_node,
					   &adjhor);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS IX2 SENSE ADJH failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS IX2 SENSE ADJ HORIZ computed!\n", tag_ftm5);


		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node - 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjhor, 1,
					ssCompData.header.sense_node - 1,
					thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS IX2 SENSE ADJH failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 SENSE ADJH TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 SENSE ADJH TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 SENSE ADJ TEST:.................SKIPPED\n",
			 tag_ftm5);

	/* SS TOTAL IX SENSE */
	logError_ftm5(0, "%s SS TOTAL IX SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseIxTotal == 1 || todo->SelfSenseIxTotalAdj == 1) {
		logError_ftm5(0, "%s SS TOTAL IX SENSE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseIxTotal == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_SENSE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_SENSE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_SENSE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_SENSE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotalFromU(totCompData.ix_sn, 1,
						       totCompData.header.
						       sense_node,
						       thresholds_min,
						       thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL IX SENSE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX SENSE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX SENSE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX SENSE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);


		logError_ftm5(0, "%s SS TOTAL IX SENSE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfSenseIxTotalAdj == 1) {
			/* SS TOTAL IX SENSE ADJH TEST */
			logError_ftm5(0, "%s SS TOTAL IX SENSE ADJHORIZ TEST:\n",
				 tag_ftm5);
			ret = computeAdjHorizTotalFromU(totCompData.ix_sn, 1,
							totCompData.header.
							sense_node,
							&total_adjhor);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS TOTAL IX SENSE ADJH failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0,
				 "%s SS TOTAL IX SENSE ADJ HORIZ computed!\n",
				 tag_ftm5);


			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_SENSE_ADJH_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjhor, 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS TOTAL IX SENSE ADJH failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX SENSE ADJH TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX SENSE ADJH TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX SENSE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL IX SENSE TEST:.................SKIPPED\n",
			 tag_ftm5);

	/************* SS SENSE CX **************/
	/* SS CX1 FORCE TEST */
	logError_ftm5(0, "%s SS CX1 FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceCx1 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX1_FORCE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX1_FORCE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.f_cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS CX1 FORCE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX1 FORCE TEST:.................OK\n\n",
				 tag_ftm5);
		kfree(thresholds);
		thresholds = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX1 FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);



	/* SS CX2 FORCE TEST */
	logError_ftm5(0, "%s SS CX2 FORCE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfForceCx2 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_FORCE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_FORCE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_FORCE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_FORCE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(ssCompData.cx2_fm,
				     ssCompData.header.force_node, 1,
				     thresholds_min,
				     thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS CX2 FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 FORCE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 FORCE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 FORCE MIN MAX TEST:.................SKIPPED\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS CX2 FORCE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfForceCx2Adj == 1) {
		/* SS CX2 FORCE ADJV TEST */
		logError_ftm5(0, "%s SS CX2 FORCE ADJVERT TEST:\n", tag_ftm5);
		ret = computeAdjVert_ftm5(ssCompData.cx2_fm,
				     ssCompData.header.force_node, 1, &adjvert);
		/* compute the ADJV for CX2  FORCE */
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS CX2 FORCE ADJV failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS CX2 FORCE ADJV computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjvert, ssCompData.header.force_node -
					1, 1, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 FORCE ADJV TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 FORCE ADJV TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 FORCE ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* SS TOTAL CX FORCE */
	logError_ftm5(0, "%s SS TOTAL CX FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceCxTotal == 1 || todo->SelfForceCxTotalAdj == 1) {
		logError_ftm5(0, "%s SS TOTAL CX FORCE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceCxTotal == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_FORCE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_FORCE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_FORCE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_FORCE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(totCompData.cx_fm,
						  totCompData.header.force_node,
						  1, thresholds_min,
						  thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL FORCE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL FORCE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX FORCE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		/* SS TOTAL CX FORCE ADJV TEST */
		logError_ftm5(0, "%s SS TOTAL CX FORCE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfForceCxTotalAdj == 1) {
			logError_ftm5(0, "%s SS TOTAL CX FORCE ADJVERT TEST:\n",
				 tag_ftm5);
			ret = computeAdjVertTotal_ftm5(totCompData.cx_fm,
						  totCompData.header.force_node,
						  1, &total_adjvert);
			/* compute the ADJV for CX2  FORCE */
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS TOTAL CX FORCE ADJV failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s SS TOTAL CX FORCE ADJV computed!\n",
				 tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_FORCE_ADJV_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjvert,
						     totCompData.header.
						     force_node - 1, 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL CX FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL CX FORCE ADJV TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL CX FORCE ADJV TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX FORCE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL CX FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);



	/************* SS SENSE CX *************/
	/* SS CX1 SENSE TEST */
	logError_ftm5(0, "%s SS CX1 SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseCx1 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX1_SENSE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX1_SENSE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.s_cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS CX1 SENSE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX1 SENSE TEST:.................OK\n\n",
				 tag_ftm5);
		kfree(thresholds);
		thresholds = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX1 SENSE TEST:.................SKIPPED\n\n",
			 tag_ftm5);


	/* SS CX2 SENSE TEST */
	logError_ftm5(0, "%s SS CX2 SENSE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfSenseCx2 == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_SENSE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_SENSE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_SENSE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(ssCompData.cx2_sn, 1,
				     ssCompData.header.sense_node,
				     thresholds_min, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS CX2 SENSE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 SENSE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 SENSE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 SENSE MIN MAX TEST:.................SKIPPED\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS CX2 SENSE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfSenseCx2Adj == 1) {
		/* SS CX2 SENSE ADJH TEST */
		logError_ftm5(0, "%s SS CX2 SENSE ADJHORIZ TEST:\n", tag_ftm5);
		ret = computeAdjHoriz_ftm5(ssCompData.cx2_sn, 1,
				      ssCompData.header.sense_node, &adjhor);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS CX2 SENSE ADJH failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS CX2 SENSE ADJH computed!\n", tag_ftm5);


		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node - 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjhor, 1,
					ssCompData.header.sense_node - 1,
					thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS CX2 SENSE ADJH failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 SENSE ADJH TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 SENSE ADJH TEST:.................OK\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 SENSE ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* SS TOTAL CX SENSE */
	logError_ftm5(0, "%s SS TOTAL CX SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseCxTotal == 1 || todo->SelfSenseCxTotalAdj == 1) {
		logError_ftm5(0, "%s SS TOTAL CX SENSE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseCxTotal == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_SENSE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_SENSE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_SENSE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_SENSE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(totCompData.cx_sn, 1,
						  totCompData.header.sense_node,
						  thresholds_min,
						  thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL CX SENSE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL CX SENSE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL CX SENSE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX SENSE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);


		/* SS TOTAL IX SENSE ADJH TEST */
		logError_ftm5(0, "%s SS TOTAL CX SENSE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfSenseCxTotalAdj == 1) {
			logError_ftm5(0, "%s SS TOTAL CX SENSE ADJHORIZ TEST:\n",
				 tag_ftm5);
			ret = computeAdjHorizTotal_ftm5(totCompData.cx_sn, 1,
						   totCompData.header.sense_node,
						   &total_adjhor);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS TOTAL CX SENSE ADJH failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0,
				 "%s SS TOTAL CX SENSE ADJ HORIZ computed!\n",
				 tag_ftm5);


			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_SENSE_ADJH_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjhor, 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS TOTAL CX SENSE ADJH failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL CX SENSE ADJH TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL CX SENSE ADJH TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX SENSE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL CX SENSE TEST:.................SKIPPED\n",
			 tag_ftm5);



	if ((todo->SelfSenseCx1LP | todo->SelfSenseCx2LP |
		todo->SelfSenseCx2AdjLP | todo->SelfSenseCxTotalLP |
		todo->SelfSenseCxTotalAdjLP | todo->SelfSenseIx1LP |
		todo->SelfSenseIx2LP | todo->SelfSenseIx2AdjLP |
		todo->SelfSenseIxTotalLP | todo->SelfSenseIxTotalAdjLP) == 1) {
		ret = production_test_ss_ix_cx_lp(info, path_limits, stop_on_fail,
			todo);
		if (ret < OK) {
			count_fail += 1;
			logError_ftm5(1,
				 "%s production_test_data_ftm5: production_test_ss_ix_cx_lp failed... ERROR = %08X\n",
				 tag_ftm5, ret);
			goto ERROR;
		}
	} else
		logError_ftm5(0, "%s SS IX CX LP TEST:.................SKIPPED\n",
			 tag_ftm5);

ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		kfree(ssCompData.ix2_fm);
		ssCompData.ix2_fm = NULL;
		kfree(ssCompData.ix2_sn);
		ssCompData.ix2_sn = NULL;
		kfree(ssCompData.cx2_fm);
		ssCompData.cx2_fm = NULL;
		kfree(ssCompData.cx2_sn);
		ssCompData.cx2_sn = NULL;
		kfree(totCompData.ix_fm);
		totCompData.ix_fm = NULL;
		kfree(totCompData.ix_sn);
		totCompData.ix_sn = NULL;
		kfree(totCompData.cx_fm);
		totCompData.cx_fm = NULL;
		kfree(totCompData.cx_sn);
		totCompData.cx_sn = NULL;
		logError_ftm5(0,
			 "%s SS IX CX testes finished!.................OK\n\n",
			 tag_ftm5);
		return OK;
	} else {
	/* print all kind of data in just one row for readability reason */
		print_frame_u8_ftm5("SS Init Data Ix2_fm = ", array1dTo2d_u8_ftm5(
				       ssCompData.ix2_fm,
				       ssCompData.header.force_node, 1),
			       ssCompData.header.force_node, 1);
		print_frame_i8("SS Init Data Cx2_fm = ", array1dTo2d_i8(
				       ssCompData.cx2_fm,
				       ssCompData.header.force_node, 1),
			       ssCompData.header.force_node, 1);
		print_frame_u8_ftm5("SS Init Data Ix2_sn = ", array1dTo2d_u8_ftm5(
				       ssCompData.ix2_sn,
				       ssCompData.header.sense_node,
				       ssCompData.header.sense_node), 1,
			       ssCompData.header.sense_node);
		print_frame_i8("SS Init Data Cx2_sn = ", array1dTo2d_i8(
				       ssCompData.cx2_sn,
				       ssCompData.header.sense_node,
				       ssCompData.header.sense_node), 1,
			       ssCompData.header.sense_node);
		print_frame_u16("TOT SS Init Data Ix_fm = ", array1dTo2d_u16(
					totCompData.ix_fm,
					totCompData.header.force_node, 1),
				totCompData.header.force_node, 1);
		print_frame_short_ftm5("TOT SS Init Data Cx_fm = ",
				  array1dTo2d_short_ftm5(totCompData.cx_fm,
						    totCompData.header.
						    force_node, 1),
				  totCompData.header.force_node, 1);
		print_frame_u16("TOT SS Init Data Ix_sn = ", array1dTo2d_u16(
					totCompData.ix_sn,
					totCompData.header.sense_node,
					totCompData.header.sense_node), 1,
				totCompData.header.sense_node);
		print_frame_short_ftm5("TOT SS Init Data Cx_sn = ",
				  array1dTo2d_short_ftm5(totCompData.cx_sn,
						    totCompData.header.
						    sense_node,
						    totCompData.header.
						    sense_node),
				  1, totCompData.header.sense_node);
		logError_ftm5(0,
			 "%s SS IX CX testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (ssCompData.ix2_fm != NULL)
			kfree(ssCompData.ix2_fm);
		if (ssCompData.ix2_sn != NULL)
			kfree(ssCompData.ix2_sn);
		if (ssCompData.cx2_fm != NULL)
			kfree(ssCompData.cx2_fm);
		if (ssCompData.cx2_sn != NULL)
			kfree(ssCompData.cx2_sn);
		if (totCompData.ix_fm != NULL)
			kfree(totCompData.ix_fm);
		if (totCompData.ix_sn != NULL)
			kfree(totCompData.ix_sn);
		if (totCompData.cx_fm != NULL)
			kfree(totCompData.cx_fm);
		if (totCompData.cx_sn != NULL)
			kfree(totCompData.cx_sn);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (ssCompData.ix2_fm != NULL)
		kfree(ssCompData.ix2_fm);
	if (ssCompData.ix2_sn != NULL)
		kfree(ssCompData.ix2_sn);
	if (ssCompData.cx2_fm != NULL)
		kfree(ssCompData.cx2_fm);
	if (ssCompData.cx2_sn != NULL)
		kfree(ssCompData.cx2_sn);
	if (totCompData.ix_fm != NULL)
		kfree(totCompData.ix_fm);
	if (totCompData.ix_sn != NULL)
		kfree(totCompData.ix_sn);
	if (totCompData.cx_fm != NULL)
		kfree(totCompData.cx_fm);
	if (totCompData.cx_sn != NULL)
		kfree(totCompData.cx_sn);
	return ret;
}

/**
  * Perform all the info->tests_ftm5 selected in a TestTodo variable related to SS Init
  * data for LP mode (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_ix_cx_lp(struct fts_ts_info *info, char *path_limits, int stop_on_fail,
			     TestToDo *todo)
{
	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	SelfSenseData ssCompData;
	TotSelfSenseData totCompData;

	u8 *adjhor = NULL;
	u8 *adjvert = NULL;

	short container;

	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;

	logError_ftm5(0, "%s\n", tag_ftm5);
	logError_ftm5(0, "%s SS LP IX CX testes are starting...\n", tag_ftm5);
	ret = readSelfSenseCompensationData_ftm5(info, LOAD_CX_SS_TOUCH_IDLE, &ssCompData);
	/* read the SS LP compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readSelfSenseCompensationData_ftm5 failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = readTotSelfSenseCompensationData(info, LOAD_PANEL_CX_TOT_SS_TOUCH_IDLE,
					       &totCompData);
	/* read the TOT SS LP compensation data */
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: readTotSelfSenseCompensationData failed... ERROR %08X\n",
			 tag_ftm5, ERROR_PROD_TEST_DATA);
		kfree(ssCompData.ix2_fm);
		kfree(ssCompData.ix2_sn);
		kfree(ssCompData.cx2_fm);
		kfree(ssCompData.cx2_sn);
		return ret | ERROR_PROD_TEST_DATA;
	}

	/************* SS FORCE IX LP **************/
	/* SS IX1 LP FORCE TEST */
	logError_ftm5(0, "%s SS IX1 LP FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceIx1LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX1_LP_FORCE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX1_LP_FORCE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		container = (short)ssCompData.f_ix1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS IX1 LP FORCE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX1 LP FORCE TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS IX1 LP FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;
	/* SS IX2 LP FORCE TEST */
	logError_ftm5(0, "%s SS IX2 LP FORCE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfForceIx2LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_LP_FORCE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_LP_FORCE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_LP_FORCE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_LP_FORCE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapFromU(ssCompData.ix2_fm,
					  ssCompData.header.force_node, 1,
					  thresholds_min,
					  thresholds_max);	/* check the
								 * values with
								 * thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 LP FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 LP FORCE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 LP FORCE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 LP FORCE MIN MAX TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS IX2 LP FORCE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfForceIx2AdjLP == 1) {
		/* SS IX2 FORCE ADJV TEST */
		logError_ftm5(0, "%s SS IX2 LP FORCE ADJVERT TEST:\n", tag_ftm5);
		ret = computeAdjVertFromU(ssCompData.ix2_fm,
					  ssCompData.header.force_node, 1,
					  &adjvert);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS IX2 LP FORCE ADJV failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS IX2 LP FORCE ADJV computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_LP_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);	/* load the max
								 * thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_LP_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjvert, ssCompData.header.force_node -
					1, 1, thresholds_max);	/* check the
								 * values with
								 * thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 LP FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 LP FORCE ADJV TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 LP FORCE ADJV TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 LP FORCE ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* SS TOTAL FORCE IX */
	logError_ftm5(0, "%s SS TOTAL IX LP FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceIxTotalLP == 1 || todo->SelfForceIxTotalAdjLP == 1) {
		logError_ftm5(0, "%s SS TOTAL IX LP FORCE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceIxTotalLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_LP_FORCE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
						/* load the min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_LP_FORCE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_LP_FORCE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
						/* load the max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_LP_FORCE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotalFromU(totCompData.ix_fm,
						       totCompData.header.
						       force_node, 1,
						       thresholds_min,
						       thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5  SS TOTAL IX LP FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX LP FORCE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX LP FORCE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX LP FORCE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		logError_ftm5(0, "%s SS TOTAL IX LP FORCE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfForceIxTotalAdjLP == 1) {
			/* SS TOTAL IX FORCE ADJV TEST */
			logError_ftm5(0, "%s SS TOTAL IX LP FORCE ADJVERT TEST:\n",
				 tag_ftm5);
			ret = computeAdjVertTotalFromU(totCompData.ix_fm,
						       totCompData.header.
						       force_node, 1,
						       &total_adjvert);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS TOTAL IX LP FORCE ADJV failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s SS TOTAL IX LP FORCE ADJV computed!\n",
				 tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_LP_FORCE_ADJV_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_LP_FORCE_ADJV_MAP_MAX... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjvert,
						     totCompData.header.
						     force_node - 1, 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL IX LP FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX LP FORCE ADJV TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX LP FORCE ADJV TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX LP FORCE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL IX LP FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);


	/************** SS SENSE IX LP **************/
	/* SS IX1 LP SENSE TEST */
	logError_ftm5(0, "%s SS IX1 LP SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseIx1LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX1_LP_SENSE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX1_LP_SENSE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.s_ix1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS IX1 SENSE LP TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX1 LP SENSE TEST:.................OK\n\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS IX1 LP SENSE TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	kfree(thresholds);
	thresholds = NULL;
	/* SS IX2 SENSE TEST */
	logError_ftm5(0, "%s SS IX2 LP SENSE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfSenseIx2LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_LP_SENSE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_LP_SENSE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_LP_SENSE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_LP_SENSE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapFromU(ssCompData.ix2_sn, 1,
					  ssCompData.header.sense_node,
					  thresholds_min, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 LP SENSE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 LP SENSE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 LP SENSE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 LP SENSE MIN MAX TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS IX2 LP SENSE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfSenseIx2AdjLP == 1) {
		/* SS IX2 SENSE ADJH TEST */
		logError_ftm5(0, "%s SS IX2 SENSE ADJHORIZ TEST:\n", tag_ftm5);
		ret = computeAdjHorizFromU(ssCompData.ix2_sn, 1,
					   ssCompData.header.sense_node,
					   &adjhor);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS IX2 SENSE ADJH failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS IX2 SENSE ADJ HORIZ computed!\n", tag_ftm5);


		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_IX2_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node - 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_LP_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjhor, 1,
					ssCompData.header.sense_node - 1,
					thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS IX2 LP SENSE ADJH failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS IX2 LP SENSE ADJH TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS IX2 LP SENSE ADJH TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;
	} else
		logError_ftm5(0,
			 "%s SS IX2 LP SENSE ADJ TEST:.................SKIPPED\n",
			 tag_ftm5);

	/* SS TOTAL IX SENSE */
	logError_ftm5(0, "%s SS TOTAL IX LP SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseIxTotalLP == 1 || todo->SelfSenseIxTotalAdjLP == 1) {
		logError_ftm5(0, "%s SS TOTAL IX LP SENSE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseIxTotalLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_LP_SENSE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_LP_SENSE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_LP_SENSE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_LP_SENSE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotalFromU(totCompData.ix_sn, 1,
						       totCompData.header.
						       sense_node,
						       thresholds_min,
						       thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL IX LP SENSE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX LP SENSE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX LP SENSE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX LP SENSE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);


		logError_ftm5(0, "%s SS TOTAL IX LP SENSE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfSenseIxTotalAdjLP == 1) {
			/* SS TOTAL IX SENSE ADJH TEST */
			logError_ftm5(0, "%s SS TOTAL IX LP SENSE ADJHORIZ TEST:\n",
				 tag_ftm5);
			ret = computeAdjHorizTotalFromU(totCompData.ix_sn, 1,
							totCompData.header.
							sense_node,
							&total_adjhor);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS TOTAL IX LP SENSE ADJH failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0,
				 "%s SS TOTAL IX LP SENSE ADJ HORIZ computed!\n",
				 tag_ftm5);


			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_IX_LP_SENSE_ADJH_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_IX_LP_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjhor, 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS TOTAL IX LP SENSE ADJH failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL IX LP SENSE ADJH TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL IX LP SENSE ADJH TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL IX LP SENSE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL IX LP SENSE TEST:.................SKIPPED\n",
			 tag_ftm5);

	/************* SS SENSE CX LP **************/
	/* SS CX1 LP FORCE TEST */
	logError_ftm5(0, "%s SS CX1 LP FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceCx1LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX1_LP_FORCE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX1_LP_FORCE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.f_cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS CX1 LP FORCE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX1 LP FORCE TEST:.................OK\n\n",
				 tag_ftm5);
		kfree(thresholds);
		thresholds = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX1 LP FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);



	/* SS CX2 LP FORCE TEST */
	logError_ftm5(0, "%s SS CX2 LP FORCE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfForceCx2LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_LP_FORCE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_LP_FORCE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_LP_FORCE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_LP_FORCE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(ssCompData.cx2_fm,
				     ssCompData.header.force_node, 1,
				     thresholds_min,
				     thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS CX2 LP FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 LP FORCE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 LP FORCE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 LP FORCE MIN MAX TEST:.................SKIPPED\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS CX2 LP FORCE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfForceCx2AdjLP == 1) {
		/* SS CX2 FORCE ADJV TEST */
		logError_ftm5(0, "%s SS CX2 LP FORCE ADJVERT TEST:\n", tag_ftm5);
		ret = computeAdjVert_ftm5(ssCompData.cx2_fm,
				     ssCompData.header.force_node, 1, &adjvert);
		/* compute the ADJV for CX2  FORCE */
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS CX2 LP FORCE ADJV failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS CX2 LP FORCE ADJV computed!\n", tag_ftm5);

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_LP_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 ||
				tcolumns != 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_LP_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjvert, ssCompData.header.force_node -
					1, 1, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS IX2 LP FORCE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 LP FORCE ADJV TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 LP FORCE ADJV TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 LP FORCE ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* SS TOTAL CX LP FORCE */
	logError_ftm5(0, "%s SS TOTAL CX LP FORCE TEST:\n", tag_ftm5);
	if (todo->SelfForceCxTotalLP == 1 || todo->SelfForceCxTotalAdjLP == 1) {
		logError_ftm5(0, "%s SS TOTAL CX LP FORCE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfForceCxTotalLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_LP_FORCE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_LP_FORCE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_LP_FORCE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_LP_FORCE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(totCompData.cx_fm,
						  totCompData.header.force_node,
						  1, thresholds_min,
						  thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL LP FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL FORCE LP MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL FORCE LP MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX LP FORCE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);

		/* SS TOTAL CX LP FORCE ADJV TEST */
		logError_ftm5(0, "%s SS TOTAL CX LP FORCE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfForceCxTotalAdjLP == 1) {
			logError_ftm5(0, "%s SS TOTAL CX LP FORCE ADJVERT TEST:\n",
				 tag_ftm5);
			ret = computeAdjVertTotal_ftm5(totCompData.cx_fm,
						  totCompData.header.force_node,
						  1, &total_adjvert);
			/* compute the ADJV for CX2  FORCE */
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjVert_ftm5 SS TOTAL CX LP FORCE ADJV failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0, "%s SS TOTAL CX LP FORCE ADJV computed!\n",
				 tag_ftm5);

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_LP_FORCE_ADJV_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns != 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_LP_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjvert,
						     totCompData.header.
						     force_node - 1, 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL CX LP FORCE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL CX LP FORCE ADJV TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL CX LP FORCE ADJV TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX LP FORCE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL CX LP FORCE TEST:.................SKIPPED\n\n",
			 tag_ftm5);



	/************* SS SENSE CX *************/
	/* SS CX1 SENSE TEST */
	logError_ftm5(0, "%s SS CX1 LP SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseCx1LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX1_LP_SENSE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX1_LP_SENSE_MIN_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.s_cx1;
		ret = checkLimitsMinMax_ftm5(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMinMax_ftm5 SS CX1 LP SENSE TEST failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX1 LP SENSE TEST:.................OK\n\n",
				 tag_ftm5);
		kfree(thresholds);
		thresholds = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX1 LP SENSE TEST:.................SKIPPED\n\n",
			 tag_ftm5);


	/* SS CX2 LP SENSE TEST */
	logError_ftm5(0, "%s SS CX2 LP SENSE MIN MAX TEST:\n", tag_ftm5);
	if (todo->SelfSenseCx2LP == 1) {
		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_LP_SENSE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_LP_SENSE_MAP_MIN failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_LP_SENSE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_CX2_LP_SENSE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap_ftm5(ssCompData.cx2_sn, 1,
				     ssCompData.header.sense_node,
				     thresholds_min, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS CX2 LP SENSE failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 LP SENSE MIN MAX TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 LP SENSE MIN MAX TEST:.................OK\n\n",
				 tag_ftm5);

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 LP SENSE MIN MAX TEST:.................SKIPPED\n",
			 tag_ftm5);

	logError_ftm5(0, "%s SS CX2 LP SENSE ADJ TEST:\n", tag_ftm5);
	if (todo->SelfSenseCx2AdjLP == 1) {
		/* SS CX2 SENSE ADJH TEST */
		logError_ftm5(0, "%s SS CX2 LP SENSE ADJHORIZ TEST:\n", tag_ftm5);
		ret = computeAdjHoriz_ftm5(ssCompData.cx2_sn, 1,
				      ssCompData.header.sense_node, &adjhor);
		if (ret < 0) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS CX2 LP SENSE ADJH failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		logError_ftm5(0, "%s SS CX2 LP SENSE ADJH computed!\n", tag_ftm5);


		ret = parseProductionTestLimits_ftm5(info, path_limits, &info->limit_file,
						SS_CX2_LP_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node - 1)) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				 tag_ftm5, ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj_ftm5(adjhor, 1,
					ssCompData.header.sense_node - 1,
					thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			logError_ftm5(1,
				 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS CX2 LP SENSE ADJH failed... ERROR COUNT = %d\n",
				 tag_ftm5, ret);
			logError_ftm5(0,
				 "%s SS CX2 LP SENSE ADJH TEST:.................FAIL\n\n",
				 tag_ftm5);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			logError_ftm5(0,
				 "%s SS CX2 LP SENSE ADJH TEST:.................OK\n",
				 tag_ftm5);

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;
	} else
		logError_ftm5(0,
			 "%s SS CX2 LP SENSE ADJ TEST:.................SKIPPED\n\n",
			 tag_ftm5);

	/* SS TOTAL CX SENSE */
	logError_ftm5(0, "%s SS TOTAL CX LP SENSE TEST:\n", tag_ftm5);
	if (todo->SelfSenseCxTotalLP == 1 || todo->SelfSenseCxTotalAdjLP == 1) {
		logError_ftm5(0, "%s SS TOTAL CX LP SENSE MIN MAX TEST:\n", tag_ftm5);
		if (todo->SelfSenseCxTotalLP == 1) {
			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_LP_SENSE_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_LP_SENSE_MAP_MIN failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_LP_SENSE_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_LP_SENSE_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal_ftm5(totCompData.cx_sn, 1,
						  totCompData.header.sense_node,
						  thresholds_min,
						  thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMap_ftm5 SS TOTAL CX LP SENSE failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL CX LP SENSE MIN MAX TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL CX LP SENSE MIN MAX TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX LP SENSE MIN MAX TEST:.................SKIPPED\n",
				 tag_ftm5);


		/* SS TOTAL IX SENSE ADJH TEST */
		logError_ftm5(0, "%s SS TOTAL CX LP SENSE ADJ TEST:\n", tag_ftm5);
		if (todo->SelfSenseCxTotalAdjLP == 1) {
			logError_ftm5(0, "%s SS TOTAL CX LP SENSE ADJHORIZ TEST:\n",
				 tag_ftm5);
			ret = computeAdjHorizTotal_ftm5(totCompData.cx_sn, 1,
						   totCompData.header.sense_node,
						   &total_adjhor);
			if (ret < 0) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: computeAdjHoriz_ftm5 SS TOTAL CX LP SENSE ADJH failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			logError_ftm5(0,
				 "%s SS TOTAL CX LP SENSE ADJ HORIZ computed!\n",
				 tag_ftm5);


			ret = parseProductionTestLimits_ftm5(info, path_limits,
							&info->limit_file,
							SS_TOTAL_CX_LP_SENSE_ADJH_MAP_MAX,
							&thresholds_max, &trows,
							&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node - 1)) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: parseProductionTestLimits_ftm5 SS_TOTAL_CX_LP_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
					 tag_ftm5, ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal_ftm5(total_adjhor, 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				logError_ftm5(1,
					 "%s production_test_data_ftm5: checkLimitsMapAdj_ftm5 SS TOTAL CX LP SENSE ADJH failed... ERROR COUNT = %d\n",
					 tag_ftm5, ret);
				logError_ftm5(0,
					 "%s SS TOTAL CX LP SENSE ADJH TEST:.................FAIL\n\n",
					 tag_ftm5);
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				logError_ftm5(0,
					 "%s SS TOTAL CX LP SENSE ADJH TEST:.................OK\n\n",
					 tag_ftm5);

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;
		} else
			logError_ftm5(0,
				 "%s SS TOTAL CX LP SENSE ADJ TEST:.................SKIPPED\n",
				 tag_ftm5);
	} else
		logError_ftm5(0,
			 "%s SS TOTAL CX LP SENSE TEST:.................SKIPPED\n",
			 tag_ftm5);



ERROR:
	logError_ftm5(0, "%s\n", tag_ftm5);
	if (count_fail == 0) {
		kfree(ssCompData.ix2_fm);
		ssCompData.ix2_fm = NULL;
		kfree(ssCompData.ix2_sn);
		ssCompData.ix2_sn = NULL;
		kfree(ssCompData.cx2_fm);
		ssCompData.cx2_fm = NULL;
		kfree(ssCompData.cx2_sn);
		ssCompData.cx2_sn = NULL;
		kfree(totCompData.ix_fm);
		totCompData.ix_fm = NULL;
		kfree(totCompData.ix_sn);
		totCompData.ix_sn = NULL;
		kfree(totCompData.cx_fm);
		totCompData.cx_fm = NULL;
		kfree(totCompData.cx_sn);
		totCompData.cx_sn = NULL;
		logError_ftm5(0,
			 "%s SS LP IX CX  testes finished!.................OK\n\n",
			 tag_ftm5);
		return OK;
	} else {
	/* print all kind of data in just one row for readability reason */
		print_frame_u8_ftm5("SS LP Init Data Ix2_fm = ", array1dTo2d_u8_ftm5(
				       ssCompData.ix2_fm,
				       ssCompData.header.force_node, 1),
			       ssCompData.header.force_node, 1);
		print_frame_i8("SS LP Init Data Cx2_fm = ", array1dTo2d_i8(
				       ssCompData.cx2_fm,
				       ssCompData.header.force_node, 1),
			       ssCompData.header.force_node, 1);
		print_frame_u8_ftm5("SS LP Init Data Ix2_sn = ", array1dTo2d_u8_ftm5(
				       ssCompData.ix2_sn,
				       ssCompData.header.sense_node,
				       ssCompData.header.sense_node), 1,
			       ssCompData.header.sense_node);
		print_frame_i8("SS LP Init Data Cx2_sn = ", array1dTo2d_i8(
				       ssCompData.cx2_sn,
				       ssCompData.header.sense_node,
				       ssCompData.header.sense_node), 1,
			       ssCompData.header.sense_node);
		print_frame_u16("TOT SS LP Init Data Ix_fm = ", array1dTo2d_u16(
					totCompData.ix_fm,
					totCompData.header.force_node, 1),
				totCompData.header.force_node, 1);
		print_frame_short_ftm5("TOT SS LP Init Data Cx_fm = ",
				  array1dTo2d_short_ftm5(totCompData.cx_fm,
						    totCompData.header.
						    force_node, 1),
				  totCompData.header.force_node, 1);
		print_frame_u16("TOT SS LP Init Data Ix_sn = ", array1dTo2d_u16(
					totCompData.ix_sn,
					totCompData.header.sense_node,
					totCompData.header.sense_node), 1,
				totCompData.header.sense_node);
		print_frame_short_ftm5("TOT SS LP Init Data Cx_sn = ",
				  array1dTo2d_short_ftm5(totCompData.cx_sn,
						    totCompData.header.
						    sense_node,
						    totCompData.header.
						    sense_node),
				  1, totCompData.header.sense_node);
		logError_ftm5(0,
			 "%s SS LP IX CX testes finished!.................FAILED  fails_count = %d\n\n",
			 tag_ftm5, count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (ssCompData.ix2_fm != NULL)
			kfree(ssCompData.ix2_fm);
		if (ssCompData.ix2_sn != NULL)
			kfree(ssCompData.ix2_sn);
		if (ssCompData.cx2_fm != NULL)
			kfree(ssCompData.cx2_fm);
		if (ssCompData.cx2_sn != NULL)
			kfree(ssCompData.cx2_sn);
		if (totCompData.ix_fm != NULL)
			kfree(totCompData.ix_fm);
		if (totCompData.ix_sn != NULL)
			kfree(totCompData.ix_sn);
		if (totCompData.cx_fm != NULL)
			kfree(totCompData.cx_fm);
		if (totCompData.cx_sn != NULL)
			kfree(totCompData.cx_sn);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (ssCompData.ix2_fm != NULL)
		kfree(ssCompData.ix2_fm);
	if (ssCompData.ix2_sn != NULL)
		kfree(ssCompData.ix2_sn);
	if (ssCompData.cx2_fm != NULL)
		kfree(ssCompData.cx2_fm);
	if (ssCompData.cx2_sn != NULL)
		kfree(ssCompData.cx2_sn);
	if (totCompData.ix_fm != NULL)
		kfree(totCompData.ix_fm);
	if (totCompData.ix_sn != NULL)
		kfree(totCompData.ix_sn);
	if (totCompData.cx_fm != NULL)
		kfree(totCompData.cx_fm);
	if (totCompData.cx_sn != NULL)
		kfree(totCompData.cx_sn);
	return ret;
}

/**
  * Perform a complete Data Test check of the IC
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_data_ftm5(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int res = OK, ret;

	if (todo == NULL) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: No TestToDo specified!! ERROR = %08X\n",
			 tag_ftm5, (ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA));
		return ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA;
	}


	logError_ftm5(0, "%s DATA Production test is starting...\n", tag_ftm5);


	ret = production_test_ms_raw_ftm5(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: production_test_ms_raw_ftm5 failed... ERROR = %08X\n",
			 tag_ftm5, ret);
		if (stop_on_fail == 1)
			goto END;
	}



	ret = production_test_ms_cx_ftm5(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: production_test_ms_cx_ftm5 failed... ERROR = %08X\n",
			 tag_ftm5, ret);
		if (stop_on_fail == 1)
			goto END;
	}


	ret = production_test_ss_raw_ftm5(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: production_test_ss_raw_ftm5 failed... ERROR = %08X\n",
			 tag_ftm5, ret);
		if (stop_on_fail == 1)
			goto END;
	}

	ret = production_test_ss_ix_cx_ftm5(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		logError_ftm5(1,
			 "%s production_test_data_ftm5: production_test_ss_ix_cx_ftm5 failed... ERROR = %08X\n",
			 tag_ftm5, ret);
		if (stop_on_fail == 1)
			goto END;
	}

END:
	freeLimitsFile(&info->limit_file);	/* /< release the limit file loaded
					 * during the test */
	if (res < OK)
		logError_ftm5(0, "%s DATA Production test failed!\n", tag_ftm5);
	else
		logError_ftm5(0, "%s DATA Production test finished!\n", tag_ftm5);
	return res;
}


/**
  * Retrieve the actual Test Limit data from the system (bin file or header
  *file)
  * @param path name of Production Test Limit file to load or "NULL" if the
  *limits data should be loaded by a .h file
  * @param file pointer to the LimitFile struct which will contains the limits
  *data
  * @return OK if success or an error code which specify the type of error
  *encountered
  */
int getLimitsFile(struct fts_ts_info *info, char *path, LimitFile *file)
{
	const struct firmware *fw = NULL;
	struct device *dev = NULL;
	int fd = -1;

	logError_ftm5(0, "%s Get Limits File starting... %s\n", tag_ftm5, path);

	if (file->data != NULL) {/* to avoid memory leak on consecutive call of
				 * the function with the same pointer */
		logError_ftm5(0,
			 "%s Pointer to Limits Data already contains something... freeing its content!\n",
			 tag_ftm5);
		kfree(file->data);
		file->data = NULL;
		file->size = 0;
	}

	strlcpy(file->name, path, MAX_LIMIT_FILE_NAME);
	if (strncmp(path, "NULL", 4) == 0) {
		logError_ftm5(1, "%s limit file path NULL... ERROR %08X\n", tag_ftm5,
			 ERROR_FILE_NOT_FOUND);
		return ERROR_FILE_NOT_FOUND;
	} else {
		dev = &info->client->dev;
		if (dev != NULL) {
			logError_ftm5(0, "%s Loading Limits File from .csv!\n", tag_ftm5);
			fd = request_firmware(&fw, path, dev);
			if (fd == 0) {
				logError_ftm5(0, "%s Start to copy %s...\n", tag_ftm5,
					 path);
				file->size = fw->size;
				file->data = (char *)kmalloc((file->size) *
							     sizeof(char),
							     GFP_KERNEL);
				if (file->data != NULL) {
					memcpy(file->data, (char *)fw->data,
					       file->size);
					logError_ftm5(0,
						 "%s Limit file Size = %d\n",
						 tag_ftm5,
						 file->size);
					release_firmware(fw);
					return OK;
				} else {
					logError_ftm5(1,
						 "%s Error while allocating data... ERROR %08X\n",
						 tag_ftm5, ERROR_ALLOC);
					release_firmware(fw);
					return ERROR_ALLOC;
				}
			} else {
				logError_ftm5(1,
					 "%s Request the file %s failed... ERROR %08X\n",
					 tag_ftm5, path, ERROR_FILE_NOT_FOUND);
				return ERROR_FILE_NOT_FOUND;
			}
		} else {
			logError_ftm5(1,
				 "%s Error while getting the device ERROR %08X\n",
				 tag_ftm5,
				 ERROR_FILE_READ);
			return ERROR_FILE_READ;
		}
	}
}

/**
  * Reset and release the memory which store a Production Limit File previously
  *loaded
  * @param file pointer to the LimitFile struct to free
  * @return OK if success or an error code which specify the type of error
  *encountered
  */

int freeLimitsFile(LimitFile *file)
{
	logError_ftm5(0, "%s Freeing Limit File ...\n", tag_ftm5);
	if (file != NULL) {
		if (file->data != NULL) {
			kfree(file->data);
			file->data = NULL;
		} else
			logError_ftm5(0, "%s Limit File was already freed!\n", tag_ftm5);
		file->size = 0;
		strlcpy(file->name, " ", MAX_LIMIT_FILE_NAME);
		return OK;
	} else {
		logError_ftm5(1, "%s Passed a NULL argument! ERROR %08X\n", tag_ftm5,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}

/**
  * Reset and release the memory which store the current Limit File previously
  *loaded
  * @return OK if success or an error code which specify the type of error
  *encountered
  */

int freeCurrentLimitsFile(struct fts_ts_info *info)
{
	return freeLimitsFile(&info->limit_file);
}


/**
  * Parse the raw data read from a Production test limit file in order to find
  *the specified information
  * If no limits file data are passed, the function loads and stores the limit
  *file from the system
  * @param path name of Production Test Limit file to load or "NULL" if the
  *limits data should be loaded by a .h file
  * @param file pointer to LimitFile struct that should be parsed or NULL if the
  *limit file in the system should be loaded and then parsed
  * @param label string which identify a particular set of data in the file that
  *want to be loaded
  * @param data pointer to the pointer which will contains the specified limits
  *data as 1 dimension matrix with data arranged row after row
  * @param row pointer to a int variable which will contain the number of row of
  *data
  * @param column pointer to a int variable which will contain the number of
  *column of data
  * @return OK if success or an error code which specify the type of error
  */
int parseProductionTestLimits_ftm5(struct fts_ts_info *info, char *path, LimitFile *file, char *label,
			      int **data, int *row, int *column)
{
	int find = 0;
	char *token = NULL;
	int i = 0;
	int j = 0;
	int z = 0;


	char *line2 = NULL;
	char line[800];
	char *buf = NULL;
	int n, size, pointer = 0, ret = OK;
	char *data_file = NULL;


	if (file == NULL || strcmp(path, file->name) != 0 || file->size == 0) {
		logError_ftm5(0,
			 "%s No limit File data passed... try to get them from the system!\n",
			 tag_ftm5);
		ret = getLimitsFile(info, LIMITS_FILE, &info->limit_file);
		if (ret < OK) {
			logError_ftm5(1,
				 "%s parseProductionTestLimits_ftm5: ERROR %08X\n",
				 tag_ftm5,
				 ERROR_FILE_NOT_FOUND);
			return ERROR_FILE_NOT_FOUND;
		}
		size = info->limit_file.size;
		data_file = info->limit_file.data;
	} else {
		logError_ftm5(0, "%s Limit File data passed as arguments!\n", tag_ftm5);
		size = file->size;
		data_file = file->data;
	}



	logError_ftm5(0, "%s The size of the limits file is %d bytes...\n", tag_ftm5,
		 size);



	while (find == 0) {
		/* start to look for the wanted label */
		if (readLine_ftm5(&data_file[pointer], line, size - pointer, &n) <
		    0) {
			find = -1;
			break;
		}
		pointer += n;
		if (line[0] == '*') {	/* each header row start with *  ex.
					 * *label,n_row,n_colum */
			line2 = kstrdup(line, GFP_KERNEL);
			if (line2 == NULL) {
				logError_ftm5(1,
					 "%s parseProductionTestLimits_ftm5: kstrdup ERROR %08X\n",
					 tag_ftm5, ERROR_ALLOC);
				ret = ERROR_ALLOC;
				goto END;
			}
			buf = line2;
			line2 += 1;
			token = strsep(&line2, ",");
			if (strcmp(token, label) == 0) {/* if the row is the
							 * wanted one i retrieve
							 * rows and columns info
							 * */
				find = 1;
				token = strsep(&line2, ",");
				if (token != NULL) {
					if (sscanf(token, "%d", row) == 1)
						logError_ftm5(0, "%s Row = %d\n",
							tag_ftm5, *row);
					else {
						logError_ftm5(0, "%s ERROR while reading the row value! ERROR %08X\n",
							tag_ftm5, ERROR_FILE_PARSE);
						ret = ERROR_FILE_PARSE;
						goto END;
					}

				} else {
					logError_ftm5(1,
						 "%s parseProductionTestLimits_ftm5 1: ERROR %08X\n",
						 tag_ftm5, ERROR_FILE_PARSE);
					ret = ERROR_FILE_PARSE;
					goto END;
				}
				token = strsep(&line2, ",");
				if (token != NULL) {
					if (sscanf(token, "%d", column) == 1)
						logError_ftm5(0, "%s Column = %d\n",
							tag_ftm5, *column);
					else {
						logError_ftm5(0, "%s ERROR while reading the column value! ERROR %08X\n",
							tag_ftm5, ERROR_FILE_PARSE);
						ret = ERROR_FILE_PARSE;
						goto END;
					}

				} else {
					logError_ftm5(1,
						 "%s parseProductionTestLimits_ftm5 2: ERROR %08X\n",
						 tag_ftm5, ERROR_FILE_PARSE);
					ret = ERROR_FILE_PARSE;
					goto END;
				}

				kfree(buf);
				buf = NULL;
				*data = (int *)kmalloc(((*row) * (*column)) *
						       sizeof(int), GFP_KERNEL);
			    /* allocate the memory for containing the data */
				j = 0;
				if (*data == NULL) {
					logError_ftm5(1,
						 "%s parseProductionTestLimits_ftm5: ERROR %08X\n",
						 tag_ftm5, ERROR_ALLOC);
					ret = ERROR_ALLOC;
					goto END;
				}


				/* start to read the data */
				for (i = 0; i < *row; i++) {
					if (readLine_ftm5(&data_file[pointer], line,
						     size - pointer, &n) < 0) {
						logError_ftm5(1,
							 "%s parseProductionTestLimits_ftm5 : ERROR %08X\n",
							 tag_ftm5, ERROR_FILE_READ);
						ret = ERROR_FILE_READ;
						goto END;
					}
					pointer += n;
					line2 = kstrdup(line, GFP_KERNEL);
					if (line2 == NULL) {
						logError_ftm5(1,
							 "%s parseProductionTestLimits_ftm5: kstrdup ERROR %08X\n",
							 tag_ftm5, ERROR_ALLOC);
						ret = ERROR_ALLOC;
						goto END;
					}
					buf = line2;
					token = strsep(&line2, ",");
					for (z = 0; (z < *column) && (token !=
								      NULL);
					     z++) {
						if (sscanf(token, "%d",
							((*data) + j)) == 1) {
							j++;
							token =
							    strsep(&line2, ",");
						}
					}
					kfree(buf);
					buf = NULL;
				}
				if (j == ((*row) * (*column))) {/* check that
								 * all the data
								 * are read */
					logError_ftm5(0, "%s READ DONE!\n", tag_ftm5);
					ret = OK;
					goto END;
				}
				logError_ftm5(1,
					 "%s parseProductionTestLimits_ftm5 3: ERROR %08X\n",
					 tag_ftm5, ERROR_FILE_PARSE);
				ret = ERROR_FILE_PARSE;
				goto END;
			}
			kfree(buf);
			buf = NULL;
		}
	}
	logError_ftm5(1, "%s parseProductionTestLimits_ftm5: ERROR %08X\n", tag_ftm5,
		 ERROR_LABEL_NOT_FOUND);
	ret = ERROR_LABEL_NOT_FOUND;
END:
	if (buf != NULL)
		kfree(buf);
	return ret;
}


/**
  * Read one line of a text file passed as array of byte and terminate it with a
  *termination character '\0'
  * @param data text file as array of bytes
  * @param line pointer to an array of char that will contain the line read
  * @param size size of data
  * @param n pointer to a int variable which will contain the number of
  *characters of the line
  * @return OK if success or an error code which specify the type of error
  */
int readLine_ftm5(char *data, char *line, int size, int *n)
{
	int i = 0;

	if (size < 1)
		return ERROR_OP_NOT_ALLOW;

	while (data[i] != '\n' && i < size) {
		line[i] = data[i];
		i++;
	}
	*n = i + 1;
	line[i] = '\0';

	return OK;
}
