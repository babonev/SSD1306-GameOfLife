#ifndef DRV_I2C_H
#define DRV_I2C_H
///---------------------------------------------------------------------------------------------------------------------

///---------------------------------------------------------------------------------------------------------------------
typedef enum {eError = 0, eSuccess = !eError } Status_t;
typedef enum {eFree = 0, eBusy = !eFree } State_t;
///---------------------------------------------------------------------------------------------------------------------
void        DrvI2C_Init(I2C_TypeDef* I2Cx, int ClockSpeed, int OwnAddress);
Status_t    DrvI2C_Read(I2C_TypeDef* I2Cx, uint8_t* buf, uint32_t nbuf, uint8_t SlaveAddress);
Status_t    DrvI2C_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbuf, uint8_t SlaveAddress);
Status_t    DrvI2C_WriteBuffer(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbyte);
State_t     DrvI2C_IsFree( void );
///---------------------------------------------------------------------------------------------------------------------
#endif  /// DRV_I2C_H
