#ifndef SHOW_LOAD_LOG_H_
#define SHOW_LOAD_LOG_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// character nums of first row (variable names) shall not exceed MAX_LINE_SIZE
// colomns nums (variable count) shall not exceed MAX_COLUMNS
// row nums (total steps) which exceed MAX_VALUES_PER_COLUMN will be ignored
#define MAX_LINE_SIZE 4096
#define MAX_COLUMNS 168
#define MAX_VALUES_PER_COLUMN 20480
#define DATA_NUM MAX_VALUES_PER_COLUMN

float readValue(float** values, int col_name, int t);

void LoadLog();

#endif  // SHOW_LOAD_LOG_H_