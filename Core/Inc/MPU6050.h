
/* initialize MPU */
void MPU6050_Init(void);

/* read all accel axis and transmit through UART (for testing purposes) */
void MPU6050_read_accel(void);

/* read all gyro axis and transmit through UART (for testing purposes) */
void MPU6050_read_gyro(void);

/* get raw linear acceleration data of a specific axis*/
int16_t get_x_a(void);
int16_t get_y_a(void);
int16_t get_z_a(void);

/* get raw angular acceleration data of a specific axis*/
int16_t get_x_g(void);
int16_t get_y_g(void);
int16_t get_z_g(void);
