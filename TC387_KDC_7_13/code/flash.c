#include "zf_common_headfile.h"

#define SECTION                 0                               //扇区编号,TC264只有一个扇区,编号为0
#define FLASH_PAGE              {0,1,2,3,4,5,6,7,8,9,10,11}     //页码编号,0~11
#define FLASH_PAGE_NUM          12                              //闪存页数
#define DATA_NUMBER             512                             //闪存数据个数,TC387的闪存,每一页可以存512个数据

uint8   cell_num = 100;

#define DATA_PAGE          (0)

#define GPS_SAVE_PAGE      (1)

#define IMU_X_1_SAVE_PAGE  (2)
#define IMU_X_2_SAVE_PAGE  (3)
#define IMU_Y_1_SAVE_PAGE  (4)
#define IMU_Y_2_SAVE_PAGE  (5)

#define IMU_X_SIGN_1_SAVE_PAGE  (6)
#define IMU_X_SIGN_2_SAVE_PAGE  (7)
#define IMU_Y_SIGN_1_SAVE_PAGE  (8)
#define IMU_Y_SIGN_2_SAVE_PAGE  (9)

#define GPS_MAX_NUM    (30)
#define IMU_MAX_NUM    (1024)
uint8   GPS_savenum = 0;
uint16  IMU_savenum = 0;
uint8   GPS_savenum_point = 0;
uint16  IMU_savenum_point = 0;
double GPS_lat[30];
double GPS_lon[30];
float  IMU_X[1024];
float  IMU_Y[1024];

float  IMU_X_Point[1024];
float  IMU_Y_Point[1024];



uint32 DATA_buffer[DATA_NUMBER];
uint32 GPS_data_buffer[DATA_NUMBER];
uint32 IMU_X_1_data_buffer[DATA_NUMBER];
uint32 IMU_X_2_data_buffer[DATA_NUMBER];
uint32 IMU_Y_1_data_buffer[DATA_NUMBER];
uint32 IMU_Y_2_data_buffer[DATA_NUMBER];
uint32 IMU_X_SIGN_1_data_buffer[DATA_NUMBER];
uint32 IMU_X_SIGN_2_data_buffer[DATA_NUMBER];
uint32 IMU_Y_SIGN_1_data_buffer[DATA_NUMBER];
uint32 IMU_Y_SIGN_2_data_buffer[DATA_NUMBER];

DataTypeUnion IMU_data;
DataTypeUnion GPS_data;
DataTypeUnion data;

uint8 FLASH_DATA_FLAG = 0;
uint8 FLASH_IMU_FLAG  = 0;
uint8 FLASH_GPS_FLAG  = 0;

uint8 flash_warning = 0;                       //flash数据警告标志位，用于判断闪存数据是否正常

//DATA中,[0]为GPS_savenum,[1] IMU_savenum,后续可扩展

void IMU_SAVE_ONE_POINT(void)
{
    if(IMU_savenum_point == 0)
    {
        IMU_buffer_clean();
        IMU_X_Point[0] = 0;
        IMU_Y_Point[0] = 0;
        IMU_savenum_point++;
    }
    else
    {
        IMU_X_Point[IMU_savenum_point] = 0;
        IMU_Y_Point[IMU_savenum_point] = 0;
        IMU_savenum_point++;
    }
}



void DATA_SAVE(void)
{
    DATA_buffer_clean();
    //uint8 datanum = 2;
    data.u32_type = (uint32)GPS_savenum;
    DATA_buffer[0]= data.u32_type;

    data.u32_type = (uint32)IMU_savenum;
    DATA_buffer[1]= data.u32_type;

    if(GPS_savenum > GPS_MAX_NUM)
    {
        GPS_savenum   = GPS_MAX_NUM;
        flash_warning = 1;
    }
    if(IMU_savenum > IMU_MAX_NUM)
    {
        IMU_savenum = IMU_MAX_NUM;
        flash_warning = 1;
    }

    flash_write_page(SECTION, DATA_PAGE,      DATA_buffer,      DATA_NUMBER);
}
void DATA_READ(void)
{
    uint8 datanum = 2;
    flash_read_page(SECTION, DATA_PAGE,      DATA_buffer,      DATA_NUMBER);

    for(uint8 i = 0;i<datanum;i++)
    {
        if(DATA_buffer[i] == 0xffffffff)
            DATA_buffer[i] = 0;
    }
    data.u32_type = DATA_buffer[0];
    GPS_savenum   = (uint8)data.u32_type;

    data.u32_type = DATA_buffer[1];
    IMU_savenum   = (uint16)data.u32_type;
}

void GPS_POINT_SAVE(void)
{
    GPS_buffer_clean();
    for(uint8 i=0;i<GPS_savenum;i++)
    {
        GPS_data.double_type = GPS_lat[i];
        GPS_data_buffer[cell_num * 0 + i] = (uint32)GPS_data.u64_type;
        GPS_data_buffer[cell_num * 1 + i] = (uint32)(GPS_data.u64_type >> 32);

        GPS_data.double_type = GPS_lon[i];
        GPS_data_buffer[cell_num * 2 + i] = (uint32)GPS_data.u64_type;
        GPS_data_buffer[cell_num * 3 + i] = (uint32)(GPS_data.u64_type >> 32);
    }
    flash_write_page(SECTION, GPS_SAVE_PAGE,      GPS_data_buffer,      DATA_NUMBER);
}

void GPS_POINT_READ(void)
{
    flash_read_page(SECTION, GPS_SAVE_PAGE,      GPS_data_buffer,      DATA_NUMBER);
    for(uint8 i = 0; i < GPS_savenum; i ++)
    {
        if(GPS_data_buffer[cell_num * 0 + i] == 0xffffffff)
            GPS_data_buffer[cell_num * 0 + i] = 0;

        if(GPS_data_buffer[cell_num * 1 + i] == 0xffffffff)
            GPS_data_buffer[cell_num * 1 + i] = 0;

        if(GPS_data_buffer[cell_num * 2 + i] == 0xffffffff)
            GPS_data_buffer[cell_num * 2 + i] = 0;

        if(GPS_data_buffer[cell_num * 3 + i] == 0xffffffff)
            GPS_data_buffer[cell_num * 3 + i] = 0;
    }
    for(uint8 i = 0; i < GPS_savenum; i ++)
    {
        GPS_data.u64_type = GPS_data_buffer[cell_num * 0 + i];
        GPS_data.u64_type = GPS_data.u64_type << 32 | GPS_data_buffer[cell_num * 1 + i];
        GPS_lat[i] = GPS_data.double_type;

        GPS_data.u64_type = GPS_data_buffer[cell_num * 3 + i];
        GPS_data.u64_type = GPS_data.u64_type << 32 | GPS_data_buffer[cell_num * 2 + i];
        GPS_lon[i] = GPS_data.double_type;
    }

}

void IMU_POINT_SAVE(void)
{
    IMU_buffer_clean();
    if(IMU_savenum <= 512)
    {
        for(uint16 i=0;i<IMU_savenum;i++)
        {
            IMU_data.float_type         =  fabs(IMU_X[i]);
            IMU_X_1_data_buffer[i]      =  IMU_data.u32_type;
            IMU_data.u32_type           = (IMU_X[i]>=0)?1:2;
            IMU_X_SIGN_1_data_buffer[i] =  IMU_data.u32_type;

            IMU_data.float_type         =  fabs(IMU_Y[i]);
            IMU_Y_1_data_buffer[i]      =  IMU_data.u32_type;
            IMU_data.u32_type           = (IMU_Y[i]>=0)?1:2;
            IMU_Y_SIGN_1_data_buffer[i] =  IMU_data.u32_type;
        }
        flash_write_page(SECTION, IMU_X_1_SAVE_PAGE,      IMU_X_1_data_buffer,      DATA_NUMBER);
        flash_write_page(SECTION, IMU_X_SIGN_1_SAVE_PAGE, IMU_X_SIGN_1_data_buffer, DATA_NUMBER);
        flash_write_page(SECTION, IMU_Y_1_SAVE_PAGE,      IMU_Y_1_data_buffer,      DATA_NUMBER);
        flash_write_page(SECTION, IMU_Y_SIGN_1_SAVE_PAGE, IMU_Y_SIGN_1_data_buffer, DATA_NUMBER);
    }
    else
    {
        for(uint16 i=0;i<DATA_NUMBER;i++)
        {
            IMU_data.float_type         =  fabs(IMU_X[i]);
            IMU_X_1_data_buffer[i]      =  IMU_data.u32_type;
            IMU_data.u32_type           = (IMU_X[i]>=0)?1:2;
            IMU_X_SIGN_1_data_buffer[i] =  IMU_data.u32_type;

            IMU_data.float_type         =  fabs(IMU_Y[i]);
            IMU_Y_1_data_buffer[i]      =  IMU_data.u32_type;
            IMU_data.u32_type           = (IMU_Y[i]>=0)?1:2;
            IMU_Y_SIGN_1_data_buffer[i] =  IMU_data.u32_type;
        }
        for(uint16 i=0;i<(IMU_savenum-512);i++)
        {
            IMU_data.float_type         =  fabs(IMU_X[i+512]);
            IMU_X_2_data_buffer[i]      =  IMU_data.u32_type;
            IMU_data.u32_type           = (IMU_X[i+512]>=0)?1:2;
            IMU_X_SIGN_2_data_buffer[i] =  IMU_data.u32_type;

            IMU_data.float_type         =  fabs(IMU_Y[i+512]);
            IMU_Y_2_data_buffer[i]      =  IMU_data.u32_type;
            IMU_data.u32_type           = (IMU_Y[i+512]>=0)?1:2;
            IMU_Y_SIGN_2_data_buffer[i] =  IMU_data.u32_type;

        }
        flash_write_page(SECTION, IMU_X_1_SAVE_PAGE,      IMU_X_1_data_buffer,      DATA_NUMBER);
        flash_write_page(SECTION, IMU_X_SIGN_1_SAVE_PAGE, IMU_X_SIGN_1_data_buffer, DATA_NUMBER);
        flash_write_page(SECTION, IMU_Y_1_SAVE_PAGE,      IMU_Y_1_data_buffer,      DATA_NUMBER);
        flash_write_page(SECTION, IMU_Y_SIGN_1_SAVE_PAGE, IMU_Y_SIGN_1_data_buffer, DATA_NUMBER);

        flash_write_page(SECTION, IMU_X_2_SAVE_PAGE,      IMU_X_2_data_buffer,      DATA_NUMBER);
        flash_write_page(SECTION, IMU_X_SIGN_2_SAVE_PAGE, IMU_X_SIGN_2_data_buffer, DATA_NUMBER);
        flash_write_page(SECTION, IMU_Y_2_SAVE_PAGE,      IMU_Y_2_data_buffer,      DATA_NUMBER);
        flash_write_page(SECTION, IMU_Y_SIGN_2_SAVE_PAGE, IMU_Y_SIGN_2_data_buffer, DATA_NUMBER);
    }
    FLASH_IMU_FLAG = 1;
}

void IMU_POINT_READ(void)
{

    if(IMU_savenum<=512 && IMU_savenum >= 0)
    {
        flash_read_page(SECTION, IMU_X_1_SAVE_PAGE,      IMU_X_1_data_buffer,      DATA_NUMBER);
        flash_read_page(SECTION, IMU_X_SIGN_1_SAVE_PAGE, IMU_X_SIGN_1_data_buffer, DATA_NUMBER);
        flash_read_page(SECTION, IMU_Y_1_SAVE_PAGE,      IMU_Y_1_data_buffer,      DATA_NUMBER);
        flash_read_page(SECTION, IMU_Y_SIGN_1_SAVE_PAGE, IMU_Y_SIGN_1_data_buffer, DATA_NUMBER);

        for(uint16 i = 0; i < IMU_savenum; i ++)
        {
            if(IMU_X_1_data_buffer[i] == 0xffffffff)
                IMU_X_1_data_buffer[i] = 0;
            if(IMU_X_SIGN_1_data_buffer[i] == 0xffffffff)
                IMU_X_SIGN_1_data_buffer[i] = 0;
            if(IMU_Y_1_data_buffer[i] == 0xffffffff)
                IMU_Y_1_data_buffer[i] = 0;
            if(IMU_Y_SIGN_1_data_buffer[i] == 0xffffffff)
                IMU_Y_SIGN_1_data_buffer[i] = 0;
        }

        for(uint16 i = 0; i < IMU_savenum; i ++)
        {
            uint32 a,b;

            IMU_data.u32_type = IMU_X_1_data_buffer[i];
            IMU_X[i] = IMU_data.float_type;
            IMU_data.u32_type = IMU_X_SIGN_1_data_buffer[i];
            a = (uint32)IMU_data.u32_type;
            IMU_X[i]*=(float)(a==2)?(-1):1;

            IMU_data.u32_type = IMU_Y_1_data_buffer[i];
            IMU_Y[i] = IMU_data.float_type;
            IMU_data.u32_type = IMU_Y_SIGN_1_data_buffer[i];
            b = (uint32)IMU_data.u32_type;
            IMU_Y[i]*=(float)(b==2)?(-1):1;
        }
    }
    else if(IMU_savenum > 512 && IMU_savenum <= 1024)
    {
        flash_read_page(SECTION, IMU_X_1_SAVE_PAGE,      IMU_X_1_data_buffer,      DATA_NUMBER);
        flash_read_page(SECTION, IMU_X_SIGN_1_SAVE_PAGE, IMU_X_SIGN_1_data_buffer, DATA_NUMBER);
        flash_read_page(SECTION, IMU_Y_1_SAVE_PAGE,      IMU_Y_1_data_buffer,      DATA_NUMBER);
        flash_read_page(SECTION, IMU_Y_SIGN_1_SAVE_PAGE, IMU_Y_SIGN_1_data_buffer, DATA_NUMBER);
        flash_read_page(SECTION, IMU_X_2_SAVE_PAGE,      IMU_X_2_data_buffer,      DATA_NUMBER);
        flash_read_page(SECTION, IMU_X_SIGN_2_SAVE_PAGE, IMU_X_SIGN_2_data_buffer, DATA_NUMBER);
        flash_read_page(SECTION, IMU_Y_2_SAVE_PAGE,      IMU_Y_2_data_buffer,      DATA_NUMBER);
        flash_read_page(SECTION, IMU_Y_SIGN_2_SAVE_PAGE, IMU_Y_SIGN_2_data_buffer, DATA_NUMBER);

        for(uint16 i = 0; i < 512; i ++)
        {
            if(IMU_X_1_data_buffer[i] == 0xffffffff)
                IMU_X_1_data_buffer[i] = 0;
            if(IMU_X_SIGN_1_data_buffer[i] == 0xffffffff)
                IMU_X_SIGN_1_data_buffer[i] = 0;
            if(IMU_Y_1_data_buffer[i] == 0xffffffff)
                IMU_Y_1_data_buffer[i] = 0;
            if(IMU_Y_SIGN_1_data_buffer[i] == 0xffffffff)
                IMU_Y_SIGN_1_data_buffer[i] = 0;
        }
        for(uint16 i = 0; i < IMU_savenum-512; i ++)
        {
            if(IMU_X_2_data_buffer[i] == 0xffffffff)
                IMU_X_2_data_buffer[i] = 0;
            if(IMU_X_SIGN_2_data_buffer[i] == 0xffffffff)
                IMU_X_SIGN_2_data_buffer[i] = 0;
            if(IMU_Y_2_data_buffer[i] == 0xffffffff)
                IMU_Y_2_data_buffer[i] = 0;
            if(IMU_Y_SIGN_2_data_buffer[i] == 0xffffffff)
                IMU_Y_SIGN_2_data_buffer[i] = 0;
        }

        for(uint16 i = 0; i < 512; i++)
        {
            uint32 a,b;

            IMU_data.u32_type = IMU_X_1_data_buffer[i];
            IMU_X[i] = IMU_data.float_type;
            IMU_data.u32_type = IMU_X_SIGN_1_data_buffer[i];
            a = (uint32)IMU_data.u32_type;
            IMU_X[i]*=(float)(a==2)?(-1):1;

            IMU_data.u32_type = IMU_Y_1_data_buffer[i];
            IMU_Y[i] = IMU_data.float_type;
            IMU_data.u32_type = IMU_Y_SIGN_1_data_buffer[i];
            b = (uint32)IMU_data.u32_type;
            IMU_Y[i]*=(float)(b==2)?(-1):1;
        }
        for(uint16 i = 0; i < IMU_savenum-512; i++)
        {
            uint32 a,b;

            IMU_data.u32_type = IMU_X_2_data_buffer[i];
            IMU_X[i+512] = IMU_data.float_type;
            IMU_data.u32_type = IMU_X_SIGN_2_data_buffer[i];
            a = (uint32)IMU_data.u32_type;
            IMU_X[i+512]*=(float)(a==2)?(-1):1;

            IMU_data.u32_type = IMU_Y_2_data_buffer[i];
            IMU_Y[i+512] = IMU_data.float_type;
            IMU_data.u32_type = IMU_Y_SIGN_2_data_buffer[i];
            b = (uint32)IMU_data.u32_type;
            IMU_Y[i+512]*=(float)(b==2)?(-1):1;
        }

    }
}


void DATA_buffer_clean(void)
{
    memset(DATA_buffer, 0, sizeof(DATA_buffer));
}
void GPS_buffer_clean(void)
{
    memset(GPS_data_buffer, 0, sizeof(GPS_data_buffer));
}
void IMU_buffer_clean(void)
{
    memset(IMU_X_1_data_buffer,      0, sizeof(IMU_X_1_data_buffer));
    memset(IMU_X_SIGN_1_data_buffer, 0, sizeof(IMU_X_SIGN_1_data_buffer));
    memset(IMU_X_2_data_buffer,      0, sizeof(IMU_X_2_data_buffer));
    memset(IMU_X_SIGN_2_data_buffer, 0, sizeof(IMU_X_SIGN_2_data_buffer));
    memset(IMU_Y_1_data_buffer,      0, sizeof(IMU_Y_1_data_buffer));
    memset(IMU_Y_SIGN_1_data_buffer, 0, sizeof(IMU_Y_SIGN_1_data_buffer));
    memset(IMU_Y_2_data_buffer,      0, sizeof(IMU_Y_2_data_buffer));
    memset(IMU_Y_SIGN_2_data_buffer, 0, sizeof(IMU_Y_SIGN_2_data_buffer));
}


void Flash_Test_Routine(void)
{
    uint16 test_num = 10;

    printf("\r\n=== Flash Read/Write Test Start ===\r\n");

    IMU_savenum = test_num;
    GPS_savenum = 0;

    printf("[1] Generating Test Data:\r\n");
    for(uint16 i = 0; i < IMU_savenum; i++)
    {
        IMU_X[i] = (float)(i * 1.5f - 5.0f);   //-5.0, -3.5, -2.0...
        IMU_Y[i] = (float)(i * -2.2f + 10.0f); //10.0, 7.8, 5.6...
        printf("Write -> X[%d]: %.2f, Y[%d]: %.2f\r\n", i, IMU_X[i],i, IMU_Y[i]);
    }

    printf("\r\n[2] Saving to Flash...\r\n");
    DATA_SAVE();
    IMU_POINT_SAVE();
    // ---------------------------------------------------------
    printf("\r\n[3] Clearing RAM Arrays and Buffers...\r\n");
    IMU_buffer_clean();
    memset(IMU_X, 0, sizeof(IMU_X));
    memset(IMU_Y, 0, sizeof(IMU_Y));
    IMU_savenum = 0;
    printf("RAM Cleared.Current IMU_X[0] = %.2f\r\n", IMU_X[0]);

    // ---------------------------------------------------------
    printf("\r\n[4] Reading from Flash...\r\n");
    DATA_READ();
    IMU_POINT_READ();

    // ---------------------------------------------------------
    printf("\r\n[5] Read Result(savenum = %d):\r\n", IMU_savenum);
    if(IMU_savenum == test_num)
    {
        for(uint16 i = 0; i < IMU_savenum; i++)
        {
            printf("X[%d]:%.2f, Y[%d]:%.2f\r\n", i, IMU_X[i],i, IMU_Y[i]);
        }
        printf("\r\n=== Flash Test PASSED! ===\r\n");
    }
    else
    {
         printf("\r\n=== Flash Test FAILED! (mismatch) ===\r\n");
    }
}

