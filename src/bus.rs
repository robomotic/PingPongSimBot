//! Direct ST3215 bus access over the Waveshare/Feetech CH343 adapter, using the packet codec from `st3215::protocol`.
//! The station must be stopped: only one program can own the port.

use anyhow::{bail, Context, Result};
use bytes::Bytes;
use st3215::protocol::{normal_position, Error as ProtoError, RamRegister, ST3215Request, ST3215Response, ServoError};
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
    /// Error flags the servo reported with this reply (see `fault_names`); 0 = none. The data is still valid.
    pub fault: u8,
}

/// Bit per `ServoError` kind, in `FAULTS` order.
const FAULTS: [(ServoError, &str); 7] = [
    (ServoError::Overload, "overload"),
    (ServoError::Overheat, "overheat"),
    (ServoError::Voltage, "voltage"),
    (ServoError::AngleLimit, "angle limit"),
    (ServoError::Range, "range"),
    (ServoError::Checksum, "checksum"),
    (ServoError::Instruction, "instruction"),
];

fn fault_bits(errors: &[ServoError]) -> u8 {
    FAULTS.iter().enumerate().filter(|(_, (e, _))| errors.contains(e)).fold(0, |m, (i, _)| m | (1 << i))
}

/// Names of the flags in a `MotorState::fault` mask, e.g. ["overload"].
pub fn fault_names(mask: u8) -> Vec<&'static str> {
    FAULTS.iter().enumerate().filter(|(i, _)| mask & (1 << i) != 0).map(|(_, (_, n))| *n).collect()
}

const STATE_START: u8 = 0x38; // PresentPosition
const STATE_LEN: u8 = 0x47 - STATE_START; // up to and including PresentCurrent (2 bytes at 0x45)

pub struct Bus {
    port: SerialStream,
    timeout_ms: u64,
    /// Error flags of the last reply (0 = none).
    last_fault: u8,
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
        Ok(Self { port, timeout_ms, last_fault: 0 })
    }

    /// One request, one retry (a single corrupted reply is common on a busy bus). A reply that carries servo error
    /// flags (e.g. overload) is still a valid reply: its data is used and the flags are kept in `last_fault`.
    /// The protocol library treats such a reply as an error, which made an overloaded gripper look unreadable.
    async fn request(&mut self, req: ST3215Request) -> Result<ST3215Response> {
        self.last_fault = 0;
        let mut last = None;
        for _ in 0..2 {
            match req.async_readwrite(&mut self.port, self.timeout_ms).await {
                Ok(r) => return Ok(r),
                Err(ProtoError::Servo { errors, response_data, .. }) => {
                    self.last_fault = fault_bits(&errors);
                    return Ok(match &req {
                        ST3215Request::Read { .. } => ST3215Response::Read { data: response_data, source_bytes: Bytes::new() },
                        ST3215Request::Ping { .. } => ST3215Response::Ping { source_bytes: Bytes::new() },
                        _ => ST3215Response::Write { source_bytes: Bytes::new() },
                    });
                }
                Err(e) => last = Some(e),
            }
        }
        Err(last.map(anyhow::Error::from).unwrap_or_else(|| anyhow::anyhow!("no reply")))
    }

    pub async fn ping(&mut self, id: u8) -> bool {
        self.request(ST3215Request::Ping { motor: id }).await.is_ok()
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
            fault: self.last_fault,
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

    /// Write one byte to one servo and wait for its reply.
    pub async fn write_u8(&mut self, id: u8, address: u8, value: u8) -> Result<()> {
        self.request(ST3215Request::Write { motor: id, address, data: Bytes::copy_from_slice(&[value]) })
            .await
            .with_context(|| format!("writing motor {id} @0x{address:02X}"))?;
        Ok(())
    }

    /// Persistent (EEPROM) one-byte write: unlock, write, lock again; returns the value read back. Used for the P gain.
    /// A servo whose Response Status Level only answers reads sends no reply to a write, so a missing reply is not an
    /// error here: the read-back decides. The lock is always restored and checked.
    pub async fn write_eeprom_u8(&mut self, id: u8, address: u8, value: u8) -> Result<u8> {
        let lock = RamRegister::Lock.address();
        let _ = self.write_u8(id, lock, 0).await;
        let _ = self.write_u8(id, address, value).await;
        let _ = self.write_u8(id, lock, 1).await;
        let back = self.read_bytes(id, address, 1).await?[0];
        let locked = self.read_bytes(id, lock, 1).await?[0];
        if locked != 1 {
            let _ = self.write_u8(id, lock, 1).await;
            bail!("motor {id}: EEPROM lock reads {locked} after the write");
        }
        Ok(back)
    }

    /// Position-loop gains and torque limits of one servo (see `ServoGains`).
    pub async fn read_gains(&mut self, id: u8) -> Result<ServoGains> {
        let pdi = self.read_bytes(id, EEPROM_P, 3).await?;
        let mt = self.read_bytes(id, EEPROM_MAX_TORQUE, 2).await?;
        let tl = self.read_bytes(id, RamRegister::TorqueLimit.address(), 2).await?;
        Ok(ServoGains {
            p: pdi[0],
            d: pdi[1],
            i: pdi[2],
            max_torque: u16::from_le_bytes([mt[0], mt[1]]),
            torque_limit: u16::from_le_bytes([tl[0], tl[1]]),
        })
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

/// EEPROM addresses (ST3215): P, D, I coefficients (1 byte each, consecutive) and Max Torque (2 bytes, 0..1000).
pub const EEPROM_P: u8 = 0x15;
pub const EEPROM_MAX_TORQUE: u8 = 0x10;

/// Position-loop gains and torque limits of one servo. P/D/I and Max Torque live in EEPROM (persistent);
/// Torque Limit is the RAM copy (reset from Max Torque at power-up).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ServoGains {
    pub p: u8,
    pub d: u8,
    pub i: u8,
    pub max_torque: u16,
    pub torque_limit: u16,
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
    fn fault_flags_round_trip() {
        let m = fault_bits(&[ServoError::Overload, ServoError::Overheat]);
        assert_eq!(fault_names(m), vec!["overload", "overheat"]);
        assert_eq!(fault_names(0), Vec::<&str>::new());
    }

    #[test]
    fn sign_magnitude() {
        assert_eq!(sm16(0x0005), 5);
        assert_eq!(sm16(0x8005), -5);
    }
}
