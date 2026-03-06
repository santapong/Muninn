use embassy_rp::i2c::{I2c, Async};
use embassy_rp::peripherals::I2C0;

pub struct Mpu6050<'d> {
    i2c: I2c<'d, I2C0, Async>,
}

impl<'d> Mpu6050<'d> {
    pub fn new(i2c: I2c<'d, I2C0, Async>) -> Self {
        Self { i2c }
    }

    pub async fn init(&mut self) -> Result<(), embassy_rp::i2c::Error> {
        defmt::info!("Initializing MPU6050...");
        // Wake up MPU6050: write 0 to power management register (0x6B)
        self.i2c.write_async(0x68_u16, [0x6B, 0x00]).await?;
        Ok(())
    }

    pub async fn read_raw(&mut self) -> Result<[u8; 14], embassy_rp::i2c::Error> {
        let mut buffer = [0u8; 14];
        self.i2c.write_read_async(0x68_u16, [0x3B], &mut buffer).await?;
        Ok(buffer)
    }
}
