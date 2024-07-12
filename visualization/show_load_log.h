#ifndef SHOW_LOAD_LOG_H_
#define SHOW_LOAD_LOG_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "visualization/show_data_storage.h"

// character nums of first row (variable names) shall not exceed MAX_LINE_SIZE
// colomns nums (variable count) shall not exceed MAX_COLUMNS
// row nums (total steps) which exceed MAX_VALUES_PER_COLUMN will be ignored
#define MAX_LINE_SIZE 4096
#define MAX_COLUMNS 192
#define MAX_VALUES_PER_COLUMN DATA_NUM

float readValue(float** values, int col_name, int t);

void LoadLog(const char csvFileName[], int* totalFrame);

void DataParsing(float** values,
                 const int numColumns,
                 char** columns,
                 const int* valuesCount,
                 int* totalFrame);

void RadarDataParsing(float** values,
                      int numColumns,
                      char** columns,
                      int* valuesCount,
                      int* totalFrame);
#endif  // SHOW_LOAD_LOG_H_