#include "AtomFly.h"

AtomFly::AtomFly(/* args */)
{
}

AtomFly::~AtomFly()
{
}

void AtomFly::begin()
{
    Wire1.begin(25, 21, 10000);

    _bmp = new Adafruit_BMP280(&Wire1);

    for (int i = 0; i < 4; i++)
    {
        ledcSetup(i, 10000, 8);
        ledcAttachPin(_PWMPinMap[i], i);
    }
}

int AtomFly::initFly(void)
{
    unsigned char tempdata[1];
    unsigned char regdata;

    uint16_t addrsub = 0;
    for( int i = 0; i< 127; i++ )
    {
        Wire1.beginTransmission(i);
        if( Wire1.endTransmission() == 0 )
        {
            Serial.printf("Find %02X Addr\n",i);
            addrsub += i;
        }
    }
    if( addrsub != (0x29+0x69+0x76))
    {
        return -1;
    }

    if( _bmp->begin(0x76) == false )
    {
         Serial.printf("bmp280 init faild\n");
         return -1;
    }
    float temp = _bmp->readTemperature();
    Serial.printf("temp %.2f\n",temp);

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_WHOAMI, 1, tempdata);
    Serial.printf("%02X\r\n", tempdata[0]);
    if (tempdata[0] != 0x19)
        return -1;
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01 << 7);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01 << 0);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = 0x10;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x18;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x05;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
    delay(1);

    regdata = 0x22;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);

    delay(100);
    getGres();
    getAres();

    byte val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);

    val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);

    val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);

    val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);

    preInterval = millis();

    return 0;
}

void AtomFly::WritePWM(uint8_t Motor,uint8_t pwmData)
{
    ledcWrite(Motor,pwmData);
}

void AtomFly::WriteAllPWM(uint8_t pwmData)
{
    for (size_t i = 0; i < 4; i++)
    {
        ledcWrite(i,pwmData);
    }
}

void AtomFly::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer)
{

    Wire1.beginTransmission(driver_Addr);
    Wire1.write(start_Addr);
    Wire1.endTransmission(false);
    uint8_t i = 0;
    Wire1.requestFrom(driver_Addr, number_Bytes);

    //! Put read results in the Rx buffer
    while (Wire1.available())
    {
        read_Buffer[i++] = Wire1.read();
    }
}

void AtomFly::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
{

    Wire1.beginTransmission(driver_Addr);
    Wire1.write(start_Addr);
    Wire1.write(*write_Buffer);
    Wire1.endTransmission();
}

void AtomFly::getAccelAdc(int16_t *ax, int16_t *ay, int16_t *az)
{

    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, 6, buf);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}

void AtomFly::getGyroAdc(int16_t *gx, int16_t *gy, int16_t *gz)
{

    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, 6, buf);

    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
}

void AtomFly::getTempAdc(int16_t *t)
{

    uint8_t buf[2];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_TEMP_OUT_H, 2, buf);

    *t = ((uint16_t)buf[0] << 8) | buf[1];
}

//!俯仰，航向，横滚：pitch，yaw，roll，指三维空间中飞行器的旋转状态。
void AtomFly::getAhrsData(float *pitch, float *roll, float *yaw)
{
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;

    getGyroData(&gyroX, &gyroY, &gyroZ);
    getAccelData(&accX, &accY, &accZ);
    MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, pitch, roll, yaw);
}

void AtomFly::CalibrateGyro()
{
    float gyroX, gyroY, gyroZ;
    float sumX = 0, sumY = 0, sumZ = 0;
    const int CALIBRATE_ITER = 3000;

    Serial.printf("CalibrateGyro() started\n");

    for (int ix = 0; ix < CALIBRATE_ITER; ix++)
    {
        getGyroData(&gyroX, &gyroY, &gyroZ);
        sumX += gyroX * gRes;
        sumY += gyroY * gRes;
        sumZ += gyroZ * gRes;
    }

    gyroXoffset = sumX / CALIBRATE_ITER;
    gyroYoffset = sumY / CALIBRATE_ITER;
    gyroZoffset = sumZ / CALIBRATE_ITER;

    Serial.printf("CalibrateGyro() done x %.2f y %.2f z %.2f\n", gyroXoffset, gyroYoffset, gyroZoffset);
}

void AtomFly::getAttitude(float *pitch, float *roll, float *yaw)
{
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;

#if 0
    const int ACC_AVG = 5;

    double avg_aX = 0, avg_aY = 0, avg_aZ = 0;
    double avg_gX = 0, avg_gY = 0, avg_gZ = 0;

    for (int ix = 0; ix < ACC_AVG; ix++)
    {
        getAccelData(&accX, &accY, &accZ);
        getGyroData(&gyroX, &gyroY, &gyroZ);

        avg_aX += accX;
        avg_aY += accY;
        avg_aZ += accZ;

        avg_gX += gyroX;
        avg_gY += gyroY;
        avg_gZ += gyroZ;
    }

    accX = avg_aX / ACC_AVG;
    accY = avg_aY / ACC_AVG;
    accZ = avg_aZ / ACC_AVG;

    gyroX = avg_gX / ACC_AVG;
    gyroY = avg_gY / ACC_AVG;
    gyroZ = avg_gZ / ACC_AVG;
#else
    getAccelData(&accX, &accY, &accZ);
    getGyroData(&gyroX, &gyroY, &gyroZ);
#endif

    accX = accX * aRes;
    accY = accY * aRes;
    accZ = accZ * aRes;

    gyroX = gyroX * gRes;
    gyroY = gyroY * gRes;
    gyroZ = gyroZ * gRes;

    float angleAccX = atan2(accY, accZ + abs(accX)) * RAD_TO_DEG;
    float angleAccY = atan2(accX, accZ + abs(accY)) * -RAD_TO_DEG;

    gyroX -= gyroXoffset;
    gyroY -= gyroYoffset;
    gyroZ -= gyroZoffset;

    float interval = (millis() - preInterval) * 0.001;

    // angleGyroX += gyroX * interval;
    // angleGyroY += gyroY * interval;
    // angleGyroZ += gyroZ * interval;

    const float accCoef = 0.05f;
    const float gyroCoef = (1 - accCoef);

    angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
    angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
    angleZ = gyroZ * interval;

    preInterval = millis();

    *pitch = angleY;
    *roll = angleX;
    *yaw = angleZ;


// #if 0  //apply filter to pitch and roll
//     float alpha = 0.25f;

//     //reward small change, panalize large change
//     if (abs(last_pitch - pitch) > 1.0f)
//         last_pitch = pitch = (alpha * pitch ) + (1 - alpha) * last_pitch;
//     else
//         last_pitch = pitch = (alpha * last_pitch ) + (1 - alpha) * pitch;

//     //reward small change, panalize large change
//     if (abs(last_roll - roll) > 1.0f)
//         last_roll = roll = (alpha * roll ) + (1 - alpha) * last_roll;
//     else
//         last_roll = roll = (alpha * last_roll ) + (1 - alpha) * roll;
// #endif

}

void AtomFly::getGres()
{

    switch (Gyscale)
    {
        // Possible gyro scales (and their register bit settings) are:
    case GFS_250DPS:
        gRes = 250.0 / 32768.0;
        break;
    case GFS_500DPS:
        gRes = 500.0 / 32768.0;
        break;
    case GFS_1000DPS:
        gRes = 1000.0 / 32768.0;
        break;
    case GFS_2000DPS:
        gRes = 2000.0 / 32768.0;
        break;
    }
}

void AtomFly::getAres()
{
    switch (Acscale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
        aRes = 2.0 / 32768.0;
        break;
    case AFS_4G:
        aRes = 4.0 / 32768.0;
        break;
    case AFS_8G:
        aRes = 8.0 / 32768.0;
        break;
    case AFS_16G:
        aRes = 16.0 / 32768.0;
        break;
    }
}

void AtomFly::SetGyroFsr(Gscale scale)
{
    //return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);//设置陀螺仪满量程范围
    unsigned char regdata;
    regdata = (scale << 3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(10);

    Gyscale = scale;
    getGres();
}

void AtomFly::SetAccelFsr(Ascale scale)
{
    unsigned char regdata;
    regdata = (scale << 3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(10);

    Acscale = scale;
    getAres();
}

void AtomFly::getAccelData(float *ax, float *ay, float *az)
{

    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    getAccelAdc(&accX, &accY, &accZ);

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}

void AtomFly::getGyroData(float *gx, float *gy, float *gz)
{
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    getGyroAdc(&gyroX, &gyroY, &gyroZ);

    *gx = (float)gyroX * gRes;
    *gy = (float)gyroY * gRes;
    *gz = (float)gyroZ * gRes;
}

void AtomFly::getTempData(float *t)
{

    int16_t temp = 0;
    getTempAdc(&temp);

    *t = (float)temp / 326.8 + 25.0;
}


uint16_t AtomFly::bswap(byte b[])
{
    // Big Endian unsigned short to little endian unsigned short
    uint16_t val = ((b[0] << 8) & b[1]);
    return val;
}

uint16_t AtomFly::makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void AtomFly::write_byte_data(byte data)
{
    Wire1.beginTransmission(TOF_ADDR);
    Wire1.write(data);
    Wire1.endTransmission();
}

void AtomFly::write_byte_data_at(byte reg, byte data)
{
    // write data word at address and register
    Wire1.beginTransmission(TOF_ADDR);
    Wire1.write(reg);
    Wire1.write(data);
    Wire1.endTransmission();
}

void AtomFly::write_word_data_at(byte reg, uint16_t data)
{
    // write data word at address and register
    byte b0 = (data & 0xFF);
    byte b1 = ((data >> 8) && 0xFF);

    Wire1.beginTransmission(TOF_ADDR);
    Wire1.write(reg);
    Wire1.write(b0);
    Wire1.write(b1);
    Wire1.endTransmission();
}

byte AtomFly::read_byte_data()
{
    Wire1.requestFrom(TOF_ADDR, 1);
    while (Wire1.available() < 1)
        delay(1);
    byte b = Wire1.read();
    return b;
}

byte AtomFly::read_byte_data_at(byte reg)
{
    //write_byte_data((byte)0x00);
    write_byte_data(reg);
    Wire1.requestFrom(TOF_ADDR, 1);
    while (Wire1.available() < 1)
        delay(1);
    byte b = Wire1.read();
    return b;
}

uint16_t AtomFly::read_word_data_at(byte reg)
{
    write_byte_data(reg);
    Wire1.requestFrom(TOF_ADDR, 2);
    while (Wire1.available() < 2)
        delay(1);
    _gbuf[0] = Wire1.read();
    _gbuf[1] = Wire1.read();
    return bswap(_gbuf);
}

void AtomFly::read_block_data_at(byte reg, int sz)
{
    int i = 0;
    write_byte_data(reg);
    Wire1.requestFrom(TOF_ADDR, sz);
    for (i = 0; i < sz; i++)
    {
        while (Wire1.available() < 1)
            delay(1);
        _gbuf[i] = Wire1.read();
    }
}

uint16_t AtomFly::VL53L0X_decode_vcsel_period(short vcsel_period_reg)
{
    uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
    return vcsel_period_pclks;
}

uint16_t AtomFly::readTOF()
{
    write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

    byte val = 0;
    int cnt = 0;
    while (cnt < 100)
    { // 1 second waiting time max
        delay(10);
        val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
        if (val & 0x01)
            break;
        cnt++;
    }
    /*
    if (val & 0x01)
        Serial.println("ready");
    else
        Serial.println("not ready");
    */
    read_block_data_at(0x14, 12);
    //uint16_t acnt = makeuint16(_gbuf[7],  _gbuf[6]);
    //uint16_t scnt = makeuint16(_gbuf[9],  _gbuf[8]);
    uint16_t dist = makeuint16(_gbuf[11], _gbuf[10]);
    //byte DeviceRangeStatusInternal = ((_gbuf[0] & 0x78) >> 3);

    //Serial.print("distance ");
    //Serial.println(dist);

    return dist;
}

// R,G,B from 0-255, H from 0-360, S,V from 0-100

CRGB HSVtoRGB(uint16_t h, uint16_t s, uint16_t v)
{
    CRGB ReRGB(0, 0, 0);
    int i;
    float RGB_min, RGB_max;
    RGB_max = v * 2.55f;
    RGB_min = RGB_max * (100 - s) / 100.0f;

    i = h / 60;
    int difs = h % 60;
    float RGB_Adj = (RGB_max - RGB_min) * difs / 60.0f;

    switch (i)
    {
    case 0:

        ReRGB.r = RGB_max;
        ReRGB.g = RGB_min + RGB_Adj;
        ReRGB.b = RGB_min;
        break;
    case 1:
        ReRGB.r = RGB_max - RGB_Adj;
        ReRGB.g = RGB_max;
        ReRGB.b = RGB_min;
        break;
    case 2:
        ReRGB.r = RGB_min;
        ReRGB.g = RGB_max;
        ReRGB.b = RGB_min + RGB_Adj;
        break;
    case 3:
        ReRGB.r = RGB_min;
        ReRGB.g = RGB_max - RGB_Adj;
        ReRGB.b = RGB_max;
        break;
    case 4:
        ReRGB.r = RGB_min + RGB_Adj;
        ReRGB.g = RGB_min;
        ReRGB.b = RGB_max;
        break;
    default: // case 5:
        ReRGB.r = RGB_max;
        ReRGB.g = RGB_min;
        ReRGB.b = RGB_max - RGB_Adj;
        break;
    }

    return ReRGB;
}

AtomFly fly;
