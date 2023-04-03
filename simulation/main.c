#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_SIZE 1024
#define MAX_FIELD_SIZE 128
#define MAX_RECORDS 100

// Define a structure to hold the values from each line of the CSV file
typedef struct {
    volatile float prevTime, time;
    float acc;
    float prs;
} Record;
float kf_yukseklik_cm;
float kf_hiz_cm_s;
float sealevel;
int main() {
    char filename[] = "testData.csv";
    FILE* fp = fopen(filename, "r");
    if (fp == NULL) {
        printf("Error: could not open file '%s'\n", filename);
        return 1;
    }

    int num_records = 0;

    char line[MAX_LINE_SIZE];
    Record record = {0};
    while (fgets(line, MAX_LINE_SIZE, fp) != NULL) {
        // Parse the line into fields
        char* field = strtok(line, ",");
        int field_num = 0;
        while (field != NULL && field_num < 3) {
            // Store the field value in the record structure
            switch (field_num) {
                case 0:
                    record.time = atof(field);
                    break;
                case 1:
                    record.acc = atof(field);
                    break;
                case 2:
                    record.prs = atof(field);
                    break;
            }

            // Get the next field
            field = strtok(NULL, ",");
            field_num++;
        }

        // Add the record to the array
        printf("Time=%.4f, dt=%.3f, Acc=%.2f, Prs=%.2f\n",record.time, record.time - record.prevTime, record.acc, record.prs);
        record.prevTime = record.time;
    }

    fclose(fp);
    return 0;
}
void calcAlt(float pres, float acc)
{
    kalmanFilter4d_predict(0.005);
    float atmospheric = pres / 100.0F;
    float altitude = 44330.0 * (1.0 - pow(atmospheric / sealevel, 0.1903));
    altitude *= 100.0;
    float ivme = acc * 100.0;
    kalmanFilter4d_update(altitude, ivme, kf_yukseklik_cm, kf_hiz_cm_s);
}