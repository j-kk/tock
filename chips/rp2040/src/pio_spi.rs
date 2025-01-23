use crate::gpio::DriveStrength;
use crate::pio::{LoadedProgram, PioRxClient, PioTxClient};
use crate::{
    gpio::RPGpioPin,
    pio::{Pio, SMNumber},
};
use kernel::hil::gpio::{Configure, FloatingState, Output};
use kernel::utilities::cells::{MapCell, OptionalCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

pub(crate) trait PioSpiClient {
    fn on_cmd_read(&self, read: SubSliceMut<'static, u32>, status: u32);
    fn on_cmd_write(&self, write: SubSliceMut<'static, u32>, status: u32);
}

#[derive(Debug, PartialEq)]
enum SpiState {
    Busy,
    Idle,
    ReadCmdSent(SubSliceMut<'static, u32>),
    ReadWaitForData,
    ReadWaitForStatus(SubSliceMut<'static, u32>),
    WriteCmdSent,
    WriteWaitForStatus(SubSliceMut<'static, u32>),
}

impl SpiState {
    fn is_idle(&self) -> bool {
        match self {
            SpiState::Idle => true,
            _ => false,
        }
    }
}

pub(crate) struct PioSpi<'a> {
    pio: &'a Pio,
    sm_number: SMNumber,
    dio: &'a RPGpioPin<'a>,
    clk: &'a RPGpioPin<'a>,
    cs: &'a RPGpioPin<'a>,
    state: MapCell<SpiState>,
    program: OptionalCell<LoadedProgram>,
    client: OptionalCell<&'a dyn PioSpiClient>,
}

#[allow(unused)]
impl<'a> PioSpi<'a> {
    pub fn new(
        pio: &'a Pio,
        sm_number: SMNumber,
        dio: &'a RPGpioPin<'a>,
        clk: &'a RPGpioPin<'a>,
        cs: &'a RPGpioPin<'a>,
    ) -> Self {
        Self {
            pio,
            sm_number,
            dio,
            clk,
            cs,
            state: MapCell::new(SpiState::Idle),
            program: OptionalCell::empty(),
            client: OptionalCell::empty(),
        }
    }

    pub fn init(&'static self, client: &'a dyn PioSpiClient) -> Result<(), ErrorCode> {
        self.client.set(client);
        self.pio.init();
        let sm = self.pio.sm(self.sm_number);
        let prg = [
            0x6001_u16, 0x1040_u16, 0xe080_u16, 0xa042_u16, 0x5001_u16, 0x0084_u16, 0x20a0_u16,
            0xc000_u16,
        ];
        let Ok(prg) = self.pio.add_program16(None, &prg) else {
            return Err(ErrorCode::FAIL);
        };

        self.program.set(prg);

        self.pio.gpio_init(self.dio);
        self.dio.set_floating_state(FloatingState::PullNone);
        self.pio.set_input_sync_bypass(self.dio, true);
        self.dio.set_drive_strength(DriveStrength::_12mA);
        self.dio.set_slew_rate(true);

        self.pio.gpio_init(self.clk);
        self.clk.set_drive_strength(DriveStrength::_12mA);
        self.clk.set_slew_rate(true);

        sm.set_out_pins(self.dio.pin() as u32, 1);
        sm.set_in_pins(self.dio.pin() as u32);
        sm.set_set_pins(self.dio.pin() as u32, 1);
        sm.set_out_shift(false, true, 0);
        sm.set_in_shift(false, true, 0);
        sm.set_clkdiv_int_frac(2, 0); // 62.5Mhz

        sm.set_pin_dirs(self.dio.pin() as u32, 1, true);
        sm.set_pin_dirs(self.clk.pin() as u32, 1, true);
        sm.set_pins(&[self.dio, self.clk], false);

        sm.set_rx_client(self);
        sm.set_tx_client(self);
        Ok(())
    }

    pub(crate) fn cmd_read(
        &self,
        cmd: u32,
        read: SubSliceMut<'static, u32>,
    ) -> Result<(), ErrorCode> {
        if self.state.map_or(false, |state| !state.is_idle()) {
            return Err(ErrorCode::BUSY);
        }
        self.state.replace(SpiState::Busy);
        self.cs.clear();

        let sm = self.pio.sm(self.sm_number);
        sm.set_enabled(false);

        let write_bits = 31;
        let read_bits = read.len() * 32 + 32 - 1;

        sm.push(read_bits as u32)?;
        sm.exec(0x6040); // SET Y
        sm.push(write_bits as u32)?;
        sm.exec(0x6020); // SET X
        sm.exec(0xe081); // SET PINDIR 0b1

        // set again the program (optional)
        let program = self.program.take().ok_or(ErrorCode::OFF)?;
        sm.exec_program(&program, true); // JMP program
        self.program.set(program);

        sm.set_enabled(true);

        sm.push(cmd)?;
        self.state.replace(SpiState::ReadCmdSent(read));
        Ok(())
    }

    pub(crate) fn cmd_write(&self, write: SubSliceMut<'static, u32>) -> Result<(), ErrorCode> {
        if self.state.map_or(false, |state| !state.is_idle()) {
            return Err(ErrorCode::BUSY);
        }
        self.state.replace(SpiState::Busy);
        self.cs.clear();

        let sm = self.pio.sm(self.sm_number);
        sm.set_enabled(false);

        let write_bits = write.len() * 32 - 1;
        let read_bits = 31;

        sm.push(read_bits as u32)?;
        sm.exec(0x6040); // SET Y
        sm.push(write_bits as u32)?;
        sm.exec(0x6020); // SET X
        sm.exec(0xe081); // SET PINDIR 0b1

        // set again the program (optional)
        let program = self.program.take().ok_or(ErrorCode::OFF)?;
        sm.exec_program(&program, true); // JMP program
        self.program.set(program);

        sm.set_enabled(true);

        sm.push_bulk(write)?;

        self.state.replace(SpiState::WriteCmdSent);
        Ok(())
    }

    fn process_state_change(&self, buffer: Option<SubSliceMut<'static, u32>>) {
        let Some(state) = self.state.take() else {
            return;
        };
        match (state, buffer) {
            (SpiState::ReadCmdSent(data_buffer), None) => {
                let sm = self.pio.sm(self.sm_number);
                if sm.pull_bulk(data_buffer).is_err() {
                    self.state.replace(SpiState::Busy);
                    return;
                }
                self.state.put(SpiState::ReadWaitForData);
            }
            (SpiState::ReadWaitForData, Some(read)) => {
                let sm = self.pio.sm(self.sm_number);
                if sm.pull().is_err() {
                    self.state.put(SpiState::Busy);
                    return;
                }
                self.state.put(SpiState::ReadWaitForStatus(read));
            }
            (SpiState::WriteCmdSent, Some(buffer)) => {
                let sm = self.pio.sm(self.sm_number);
                if sm.pull().is_err() {
                    self.state.replace(SpiState::Busy);
                    return;
                }
                self.state.put(SpiState::WriteWaitForStatus(buffer));
            }
            _ => {}
        };
    }

    fn update_status(&self, status: u32) {
        self.client.map(|client| {
            let Some(state) = self.state.take() else {
                return;
            };
            match state {
                SpiState::ReadWaitForStatus(read) => {
                    self.cs.set();
                    client.on_cmd_read(read, status);
                    self.state.replace(SpiState::Idle);
                }
                SpiState::WriteWaitForStatus(buffer) => {
                    self.cs.set();
                    client.on_cmd_write(buffer, status);
                    self.state.replace(SpiState::Idle);
                }
                _ => {}
            }
        });
    }
}

impl<'a> PioRxClient for PioSpi<'a> {
    fn read_complete(&self, data: u32) {
        self.update_status(data);
    }

    fn read_bulk_complete(&self, buffer: SubSliceMut<'static, u32>) {
        self.process_state_change(Some(buffer));
    }
}

impl<'a> PioTxClient for PioSpi<'a> {
    fn write_complete(&self) {
        self.process_state_change(None)
    }

    fn write_bulk_complete(&self, buffer: SubSliceMut<'static, u32>) {
        self.process_state_change(Some(buffer));
    }
}
