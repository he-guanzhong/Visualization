#ifndef SHOW_LOAD_LOG_H_
#define SHOW_LOAD_LOG_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "visualization/extension_package/show_ext_load_log.h"
#include "visualization/show_data_storage.h"

// character nums of first row (variable names) shall not exceed MAX_LINE_SIZE
// colomns nums (variable count) shall not exceed MAX_COLUMNS
// row nums (total steps) which exceed MAX_VALUES_PER_COLUMN will be ignored
#define MAX_LINE_SIZE 22000  // 8192
#define MAX_COLUMNS 330
#define MAX_VALUES_PER_COLUMN DATA_NUM

void LoadLog(const char* const csvFileName, int* totalFrame);

void SpdPlanDataParsing(float** values,
                        const int numColumns,
                        char** columns,
                        const int* valuesCount,
                        int* totalFrame);

#endif  // SHOW_LOAD_LOG_H_
