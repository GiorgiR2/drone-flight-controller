
/* Read calibration registers (22 in total) */
void read_calibration_data(void);

/* Get uncompensated Temperature */
long Get_UTemp(void);

/* Get temperatur */
float BMP180_GetTemp(void);

/* Get uncompensated Pressure */
uint32_t Get_UPress(int oss);

/* oss - oversampling ratio of the pressure measurement (00b: single, 01b: 2 times, 10b: 4 times, 11b: 8 times) */
float BMP180_GetPress(int oss);

/* Get altitude */
float BMP180_GetAlt(int oss);

/* Start Sensor */
void BMP180_start(void);

/* Get Average Altitude Value */
float BMP180_AVGAlt(void);
