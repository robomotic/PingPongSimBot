//! Direct ST3215 bus access over the Waveshare/Feetech CH343 adapter, using the packet codec from `st3215::protocol`.
//! The station must be stopped: only one program can own the port.

use anyhow::{bail, Context, Result};
use bytes::Bytes;
use st3215::protocol::{normal_position, RamRegister, ST3215Request, ST3215Response};
use tokio_serial::{SerialPortBuilderExt, SerialStream};

/// Snapshot of one servo, from one bulk read of 0x38..0x46.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorState {
    pub position: u16,
    pub speed: i16,
    pub load: i16,
    pub voltage_dv: u8,
    pub temperature_c: u8,
    /// ST3215 current unit is 6.5 mA.
    pub current_ma: f64,
}

const STATE_START: u8 = 0x38; // PresentPosition
const STATE_LEN: u8 = 0x47 - STATE_START; // up to and including PresentCurrent (2 bytes at 0x45)

pub struct Bus {
    port: SerialStream,
    timeout_ms: u64,
}

/// Feetech sign-magnitude: bit 15 is the sign.
fn sm16(raw: u16) -> i16 {
    let mag = (raw & 0x7FFF) as i16;
    if raw & 0x8000 != 0 {
        -mag
    } else {
        mag
    }
}

impl Bus {
    pub fn open(path: &str, timeout_ms: u64) -> Result<Self> {
        // Same settings as the station driver (software/drivers/st3215/src/port.rs, St3215Port::open).
        let port = tokio_serial::new(path, 1_000_000)
            .timeout(std::time::Duration::from_millis(timeout_ms))
            .open_native_async()
            .with_context(|| {
                format!("opening {path}; is the station still running? (sudo systemctl stop normacore-station)")
            })?;
        Ok(Self { port, timeout_ms })
    }

    async fn request(&mut self, req: ST3215Request) -> Result<ST3215Response> {
        // One retry: a single corrupted reply is common on a busy bus and not worth failing the tick for.
        match req.async_readwrite(&mut self.port, self.timeout_ms).await {
            Ok(r) => Ok(r),
            Err(_) => Ok(req.async_readwrite(&mut self.port, self.timeout_ms).await?),
        }
    }

    pub async fn ping(&mut self, id: u8) -> bool {
        ST3215Request::Ping { motor: id }
            .async_readwrite(&mut self.port, self.timeout_ms)
            .await
            .is_ok()
    }

    pub async fn read_state(&mut self, id: u8) -> Result<MotorState> {
        let resp = self
            .request(ST3215Request::Read { motor: id, address: STATE_START, length: STATE_LEN })
            .await
            .with_context(|| format!("reading motor {id}"))?;
        let ST3215Response::Read { data, .. } = resp else { bail!("motor {id}: unexpected reply") };
        if data.len() < STATE_LEN as usize {
            bail!("motor {id}: short reply ({} bytes)", data.len());
        }
        let u16_at = |a: u8| {
            let i = (a - STATE_START) as usize;
            u16::from_le_bytes([data[i], data[i + 1]])
        };
        let u8_at = |a: u8| data[(a - STATE_START) as usize];
        Ok(MotorState {
            position: normal_position(u16_at(RamRegister::PresentPosition.address())),
            speed: sm16(u16_at(RamRegister::PresentSpeed.address())),
            load: {
                // Load uses bit 10 as its sign (0..1000 = 0..100%).
                let raw = u16_at(RamRegister::PresentLoad.address());
                let mag = (raw & 0x3FF) as i16;
                if raw & 0x400 != 0 { -mag } else { mag }
            },
            voltage_dv: u8_at(RamRegister::PresentVoltage.address()),
            temperature_c: u8_at(RamRegister::PresentTemperature.address()),
            current_ma: (sm16(u16_at(RamRegister::PresentCurrent.address())) as f64).abs() * 6.5,
        })
    }

    pub async fn read_positions(&mut self, ids: &[u8]) -> Result<Vec<u16>> {
        let mut out = Vec::with_capacity(ids.len());
        for &id in ids {
            out.push(self.read_state(id).await?.position);
        }
        Ok(out)
    }

    /// Raw read of `len` bytes from `address`.
    pub async fn read_bytes(&mut self, id: u8, address: u8, len: u8) -> Result<Bytes> {
        let resp = self.request(ST3215Request::Read { motor: id, address, length: len }).await?;
        let ST3215Response::Read { data, .. } = resp else { bail!("motor {id}: unexpected reply") };
        if data.len() < len as usize {
            bail!("motor {id}: short reply ({} of {len} bytes)", data.len());
        }
        Ok(data)
    }

    pub async fn read_torque_enabled(&mut self, id: u8) -> Result<bool> {
        let resp = self
            .request(ST3215Request::Read { motor: id, address: RamRegister::TorqueEnable.address(), length: 1 })
            .await?;
        let ST3215Response::Read { data, .. } = resp else { bail!("motor {id}: unexpected reply") };
        Ok(data.first().copied().unwrap_or(0) != 0)
    }

    /// Broadcast sync write of a 1- or 2-byte register. No reply is sent by the servos.
    async fn sync_write(&mut self, address: u8, data: Vec<(u8, Bytes)>) -> Result<()> {
        if data.is_empty() {
            return Ok(());
        }
        ST3215Request::SyncWrite { address, data }
            .async_write(&mut self.port, self.timeout_ms)
            .await?;
        Ok(())
    }

    pub async fn sync_write_goals(&mut self, goals: &[(u8, u16)]) -> Result<()> {
        let data = goals
            .iter()
            .map(|&(id, t)| (id, Bytes::copy_from_slice(&t.min(4095).to_le_bytes())))
            .collect();
        self.sync_write(RamRegister::GoalPosition.address(), data).await
    }

    pub async fn sync_write_u16(&mut self, reg: RamRegister, values: &[(u8, u16)]) -> Result<()> {
        let data = values.iter().map(|&(id, v)| (id, Bytes::copy_from_slice(&v.to_le_bytes()))).collect();
        self.sync_write(reg.address(), data).await
    }

    /// Acc (1 byte) and Goal Speed (2 bytes) of one servo: the profile it uses for position moves.
    pub async fn read_profile(&mut self, id: u8) -> Result<(u8, u16)> {
        let acc = self.read_bytes(id, RamRegister::Acc.address(), 1).await?[0];
        let sp = self.read_bytes(id, RamRegister::GoalSpeed.address(), 2).await?;
        Ok((acc, u16::from_le_bytes([sp[0], sp[1]])))
    }

    pub async fn sync_write_u8(&mut self, reg: RamRegister, values: &[(u8, u8)]) -> Result<()> {
        let data = values.iter().map(|&(id, v)| (id, Bytes::copy_from_slice(&[v]))).collect();
        self.sync_write(reg.address(), data).await
    }

    /// Make sure M1..M7 hold their present position with torque on, without a jump:
    /// goal := present first, then torque on.
    /// M8 (gripper): with a saved `grip` goal within `GRIP_SNAP_TICKS` of where it is, squeeze to it; otherwise
    /// hold where it is. Either way M8 ends with torque on, so the paddle cannot work loose.
    pub async fn hold_present(&mut self, grip: Option<u16>) -> Result<Vec<u16>> {
        let ids: Vec<u8> = (1..=8).collect();
        let present = self.read_positions(&ids).await?;
        let mut goals: Vec<(u8, u16)> = (1..=8u8).map(|id| (id, present[id as usize - 1])).collect();
        match grip {
            Some(g) if (g as i32 - present[7] as i32).abs() <= GRIP_SNAP_TICKS => goals[7].1 = g,
            Some(g) => log::warn!(
                "M8 is at {} but the saved grip is {g}: holding where it is. Re-run `grip` if the paddle was re-seated.",
                present[7]
            ),
            None => {}
        }
        self.sync_write_goals(&goals).await?;
        let torque: Vec<(u8, u8)> = (1..=8u8).map(|id| (id, 1)).collect();
        self.sync_write_u8(RamRegister::TorqueEnable, &torque).await?;
        Ok(present)
    }
}

/// How far a saved grip may be from the present M8 position and still be applied automatically.
pub const GRIP_SNAP_TICKS: i32 = 60;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_write_of_three_goals_matches_feetech_layout() {
        let req = ST3215Request::SyncWrite {
            address: RamRegister::GoalPosition.address(),
            data: vec![
                (5, Bytes::copy_from_slice(&2048u16.to_le_bytes())),
                (6, Bytes::copy_from_slice(&1000u16.to_le_bytes())),
                (7, Bytes::copy_from_slice(&3000u16.to_le_bytes())),
            ],
        };
        let b = req.to_bytes();
        // FF FF FE len 83 2A 02 [id lo hi]x3 chk ; len = (2+1)*3 + 4 = 13
        assert_eq!(&b[..7], &[0xFF, 0xFF, 0xFE, 13, 0x83, 0x2A, 0x02]);
        assert_eq!(&b[7..16], &[5, 0x00, 0x08, 6, 0xE8, 0x03, 7, 0xB8, 0x0B]);
        let sum: u32 = b[2..b.len() - 1].iter().map(|&x| x as u32).sum();
        assert_eq!(b[b.len() - 1], !(sum as u8));
    }

    #[test]
    fn sign_magnitude() {
        assert_eq!(sm16(0x0005), 5);
        assert_eq!(sm16(0x8005), -5);
    }
}
