# Copyright 2022 Mark Leyden <mark.leyden@gmail.com>
# Copyright 2014 Ron Wellsted <ron@m0rnw.uk> M0RNW
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Abbree AR F8 radio management module"""

import time
import logging
from chirp import util, chirp_common, bitwise, memmap, errors, directory
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueBoolean, RadioSettingValueString, RadioSettingValueList, \
    RadioSettingValueMap, RadioSettings, zero_indexed_seq_map, \
    InvalidValueError

from textwrap import dedent

LOG = logging.getLogger(__name__)

# memory slot 0 is not used, start at 1 (so need 1000 slots, not 999)
# structure elements whose name starts with x are currently unidentified
_AR_F8_MEM_FORMAT = """
    #seekto 0x0044;     // TODO: This doesn't exist in radio memory
    struct {            // Can get from CMD_ID
        u32    rx_start;
        u32    rx_stop;
        u32    tx_start;
        u32    tx_stop;
    } uhf_limits;

    #seekto 0x0054;     // TODO: This doesn't exist in radio memory
    struct {            // Can get from CMD_ID
        u32    rx_start;
        u32    rx_stop;
        u32    tx_start;
        u32    tx_stop;
    } vhf_limits;

    #seekto 0x0400;     // TODO: This doesn't exist in radio memory
    struct {            // Don't know where to get this info.
        u8     model[8];
        u8     unknown[2];
        u8     oem1[10];
        u8     oem2[10];
        u8     unknown2[8];
        u8     version[10];
        u8     unknown3[6];
        u8     date[8];
    } oem_info;

    #seekto 0x0480;
    struct {
        u16    lower;
        u16    upper;
    } scan_groups[10];

    #seekto 0x0500;
    struct {
        u8    call_code[6];
    } call_groups[20];

    #seekto 0x0580;
    struct {
        char    call_name[6];
    } call_group_name[20];

    #seekto 0x0800;
    struct {
        u8      ponmsg;
        char    dispstr[15];
        u8 x0810;
        u8 x0811;
        u8 x0812;
        u8 x0813;
        u8 x0814;
        u8      voice;
        u8      timeout;
        u8      toalarm;
        u8      channel_menu;   // Not sure of this exists? Not in radio menu system
        u8      power_save;
        u8      autolock;
        u8      keylock;
        u8      beep;
        u8      stopwatch;
        u8      vox;
        u8      scan_rev;
        u8      backlight;      // 0x0820
        u8      roger_beep;
        u8      mode_sw_pwd[6]; // Don't think this feature is on this radio
        u8      reset_pwd[6];
        u16     pri_ch;
        u8      ani_sw;         // 0x0830
        u8      ptt_delay;
        u8      ani[6];
        u8      dtmf_st;
        u8 x0839;
        u8      bcl;
        u8      ptt_id;
        u8      prich_sw;
        u8      rpt_set;
        u8      rpt_spk;
        u8      rpt_ptt;
        u8      alert;          // 0x0840
        u8      pf2_func;
        u8      pf3_func;
        u8      screen_style;   // LCD Background colour
        u8      workmode_a;
        u8      workmode_b;
        u8      dtmf_tx_time;
        u8      dtmf_interval;
        u8      main_ab;
        u16     work_cha;
        u16     work_chb;
        u8 x084d;
        u8      adj_time;   // GPS local time adjustment from UTC
        u8 x084f;
        u8 x0850;           // 0x0850
        u8 x0851;
        u8 x0852;
        u8 x0853;
        u8 x0854;
        u8      rpt_mode;
        u8      language;
        u8 x0857;
        u8 x0858;
        u8 x0859;
        u8 x085a;
        u8 x085b;
        u8 x085c;
        u8 x085d;
        u8 x085e;
        u8      single_display;
        u8      ring_time;  // 0x0860
        u8      scg_a;
        u8      scg_b;
        u8 x0863;
        u8      rpt_tone;
        u8      rpt_hold;
        u8      scan_det;
        u8      sc_qt;
        u8 x0868;
        u8      smuteset;
        u8      callcode;
    } settings;

    #seekto 0x0880;
    struct {
        u32     rxfreq;
        u32     txoffset;
        u16     rxtone;
        u16     txtone;
        u8      unknown1:6,
                power:1,
                unknown2:1;
        u8      unknown3:1,
                shift_dir:2
                unknown4:2,
                mute_mode:2,
                iswide:1;
        u8      step;
        u8      squelch;
      } vfoa;

    #seekto 0x08c0;
    struct {
        u32     rxfreq;
        u32     txoffset;
        u16     rxtone;
        u16     txtone;
        u8      unknown1:3,
                dcs_type:1,     // 0x0X - Normal, 0x1X - Special
                unknown2:2,
                power:1,
                unknown3:1;
        u8      unknown4:1,
                shift_dir:2
                unknown5:2,
                mute_mode:2,
                iswide:1;
        u8      step;
        u8      squelch;
    } vfob;

    #seekto 0x0900;
    struct {
        u32     rxfreq;         // 0x0900 is always empty, starts at 0x0910
        u32     txfreq;
        u16     rxtone;         // R-CTCS & R-DCS
        u16     txtone;         // T-CTCS & T-DCS
        u8      unknown1:3,
                dcs_type:1,     // 0x0X - Normal, 0x1X - Special
                unknown2:2,
                power:1,
                unknown3:1;
        u8      unknown4:2,     
                scan_add:1,     // 0 = Don't add to scan, 1 = add to scan
                unknown5:2,
                mute_mode:2,    // 0 = "QT", 1 = "QT+DTMF", 2 = "QT*DTMF"
                iswide:1;       // 0 = Narrow, 1 = Wide
        u8      step;
        u8      squelch;
    } memory[1000];

    #seekto 0x4780;     // Data at 0x4740???
    struct {
        u8    name[8];
    } names[1000];

    #seekto 0x6700;     // List of valid channels: 0 if unpopulated, 0x9E if populated
    u8          valid[1000];
    """

CMD_ID = 0x80  # 128
CMD_END = 0x81  # 129
CMD_RD = 0x82  # 130
CMD_WR = 0x83  # 131

# Radio settings
MEM_VALID = 0x9E

FM_RANGE = (76000000, 108000000)
VHF_RANGE = (136000000, 174000000)      # Lowest freq. after reset = 12302500
# VHF_RANGE_1 = (175000000, 220000000)  # From https://www.abbree.cn/product/abbree-ar-f8-gps-6-bands/
# VHF_RANGE_2 = (230000000, 250000000)
# UHF_RANGE_1 = (330000000, 350000000)
# UHF_RANGE_2 = (350000000, 400000000)
UHF_RANGE = (400000000, 520000000)

AB_LIST = ["A", "B"]

STEPS = [5.0, 6.25, 10.0, 12.5, 25.0, 50.0, 100.0]
STEP_TUPLE_LIST = [(str(x) + "kHz", pos + 1) for pos, x in enumerate(STEPS)]
SQL_LEVEL = ["Off"] + [str(x) for x in range(1, 10)]

POWER_LEVELS = [chirp_common.PowerLevel("Low", watts=1.00),
                chirp_common.PowerLevel("High", watts=5.00)]

ROGER_LIST = ["Off", "BOT", "EOT", "Both"]
TIMEOUT_TUPLE_LIST = [("%ds" % (x * 15), x) for x in range(1, 61)]
VOX_LIST = ["Off"] + ["%ds" % x for x in range(1, 11)]
BANDWIDTH_LIST = ["Narrow", "Wide"]

# Tones are numeric, Defined in \chirp\chirp_common.py
CTCSS_TONES = sorted(chirp_common.TONES)
# Converted to strings
TONES_LIST = [""] + [str(x) for x in CTCSS_TONES]
DTCS_CODES = sorted(chirp_common.DTCS_CODES)
# Now append the DxxxN and DxxxI DTCS codes
for x in DTCS_CODES:
    TONES_LIST.append("D{:03d}N".format(x))
for x in DTCS_CODES:
    TONES_LIST.append("D{:03d}I".format(x))
TONE_MODE_LIST = ["", "Tone", "TSQL", "DTCS", "Cross"]
CROSS_MODE_LIST = [
            "Tone->Tone",
            "Tone->DTCS",
            "DTCS->Tone",
            "DTCS->",
            "->Tone",
            "->DTCS",
            "DTCS->DTCS",
        ]
TOA_LIST = ["Off"] + ["%ds" % x for x in range(1, 11)]  # Transmit overtime alarm

LANGUAGE_LIST = ["Chinese", "English"]

SCANMODE_LIST = ["Time Operation(TO)", "Carrier Operation(CO)", "Search Operation(SE)"]
DCSTYPE_LIST = ["Normal", "Special"]

PF2KEY_LIST = ["Disable", "Scan", "Lamp", "Tele Alarm", "SOS-CH", "Radio", "MONI", "CALL"]
PF3KEY_LIST = ["Disable", "Scan", "Lamp", "Tele Alarm", "SOS-CH", "Radio", "MONI", "CALL"]
WORKMODE_LIST = ["Frequency", "Channel No.", "Ch. No.+Freq.", "Channel Name"]
BACKLIGHT_LIST = ["Always On"] + [str(x) + "s" for x in range(1, 21)] + \
                 ["Always Off"]

OFFSET_LIST = ["Off", "+", "-"]

SCANCD_LIST = ["CTCSS", "DCS"]
PONMSG_TUPLE_LIST = [("Bitmap", 0x00), ("Battery Volts", 0x01)]
SPMUTE_LIST = ["QT", "QT+DTMF", "QT*DTMF"]

DTMFST_LIST = ["Off", "DT-ST", "ANI-ST", "DT+ANI"]

ALERTS = [1750, 2100, 1000, 1450]
ALERTS_LIST = ["%dkHz" % x for x in ALERTS]
PTTDELAY_TUPLE_LIST = [("%dms" % (x * 100), x) for x in range(1, 31)]
PTTID_LIST = ["BOT", "EOT", "Both"]
RINGTIME_LIST = ["Off"] + ["%s" % x for x in range(1, 106)]  # Time to ring before speaking
SCANGRP_LIST = ["All"] + ["%s" % x for x in range(1, 11)]

SCQT_LIST = ["Decoder", "Encoder", "All"]
SMUTESET_LIST = ["Off", "Rx", "Tx", "Rx/Tx & mute"]
CALLCODE_LIST = ["%s" % x for x in range(1, 21)]
STYLES_LIST = ["White", "Red", "Green", "Blue", "Purple", "Black"]  # Display background style
ADJTIME_LIST = ["+%0d" % x for x in range(0, 13)] + ["-%0d" % y for y in range(12, 0, -1)]

CHAN_TUPLE_LIST = [("Ch-%03d" % x, x) for x in range(1, 1000)]

# TODO: The following aren't in the radio menu - remove???
DTMF_TIMES = ["%s" % x for x in range(50, 501, 10)]
RPTSET_LIST = ["X-TWRPT", "X-DIRRPT"]
HOLD_TIMES = ["Off"] + ["%ds" % x for x in range(100, 5001, 100)]
RPTMODE_LIST = ["Radio", "Repeater"]

# Appears to be nothing above this location
MEM_SIZE = 0x6B00

def do_download(radio):
    """Talk to an Abbree AR-F8 and do a download"""
    image = bytearray([0xFF] * MEM_SIZE)
    try:
        result = identify_radio(radio)
        if len(result) == 32:
            image = bytearray(read_mem(radio, 0x00, MEM_SIZE, 64))
            # It doesn't appear that the following
            # are stored in 'normal' memory, so get
            # them from the ID process and store them
            # in 'memory' where the app expects them
            image[0x0044:0x0064] = result
        else:
            raise errors.RadioError("Incorrect radio limits size")

        LOG.debug("download complete.")
        return memmap.MemoryMapBytes(bytes(image))  # MemoryMapBytes expects bytes
    except errors.RadioError:
        raise
    except Exception as e:
        LOG.exception('Unknown error during download process')
        raise errors.RadioError("Failed to communicate with radio: %s" % e)


def do_upload(radio):
    """Talk to an Abbree AR-F8 and do an upload"""
    try:
        # Check we are talking to the correct radio
        result = identify_radio(radio)
        if len(result) == 33:
            upload(radio, 0x0480, MEM_SIZE, 64)   # Only writing to valid radio memory areas
    except errors.RadioError:
        raise
    except Exception as e:
        raise errors.RadioError("Failed to communicate with radio: %s" % e)
    return


def upload(radio, start, end, blocksize):
    ptr = start
    for i in range(start, end, blocksize):
        req = bytes([i // 256, i % 256])
        chunk = radio.get_mmap()[ptr:ptr + blocksize]
        write_record(radio, CMD_WR, req + chunk)
        LOG.debug(util.hexprint(req + chunk))
        cserr, ack = read_record(radio, CMD_WR)
        LOG.debug(util.hexprint(ack))
        j = ack[0] * 256 + ack[1]
        if cserr or j != ptr:
            raise Exception("Radio did not ack block %i" % ptr)
        ptr += blocksize
        if radio.status_fn:
            status = chirp_common.Status()
            status.cur = i
            status.max = end
            status.msg = "Cloning to radio"
            radio.status_fn(status)
    write_record(radio, CMD_END)


def identify_radio(radio):
    """Identify the radio"""
    #
    # The ID record returned by the radio also includes the current frequency range
    # as 4 bytes big-endian in 10Hz increments
    #
    # Offset
    #  0:10     Model, zero padded (Use first 9 chars for 'KG-UV8D-A')
    #  11:14    UHF rx lower limit (in units of 10Hz)
    #  15:18    UHF rx upper limit
    #  19:22    UHF tx lower limit
    #  23:26    UHF tx upper limit
    #  27:30    VHF rx lower limit
    #  31:34    VHF rx upper limit
    #  35:38    VHF tx lower limit
    #  39:42    VHF tx upper limit
    #
    for i in range(0, 5):
        write_record(radio, CMD_ID)
        chksum_err, resp = read_record(radio, CMD_ID)
        LOG.debug("Got:\n%s" % util.hexprint(resp))
        if chksum_err:
            LOG.error("Checksum error: retrying ident...")
            time.sleep(0.100)
            continue
        # Should get 0x2B of data
        if len(resp) != 0x2B:
            raise Exception("Incorrect data length")
        LOG.debug("Model %s" % util.hexprint(resp[0:9]))
        if resp[0:9] == bytes(radio.mem_model, 'ascii'):
            # Identified radio, send back the radio rx/tx limits
            return resp[11:]
        if len(resp) == 0:
            raise Exception("Radio not responding")
        else:
            raise Exception("Unable to identify radio")


def calc_checksum(data):
    cs = 0
    for byte in data:
        cs += byte
    return cs % 256


def read_mem(radio, start, end, blocksize):
    # allocate & fill memory
    image = bytearray()
    for i in range(start, end, blocksize):
        req = bytearray(i.to_bytes(2, byteorder='big'))
        req.append(blocksize)
        write_record(radio, CMD_RD, req)
        cs_error, resp = read_record(radio, CMD_RD)
        if cs_error:
            # TODO: probably should retry a few times here
            LOG.debug(util.hexprint(resp))
            raise Exception("Checksum error on read")
        LOG.debug("Got:\n%s" % util.hexprint(resp))
        # Check we got back the requested memory location
        if resp[0:2] != req[0:2]:
            raise Exception("Request error on read")
        # Strip off the memory location
        image += resp[2:]
        if radio.status_fn:
            status = chirp_common.Status()
            status.cur = i
            status.max = end
            status.msg = "Cloning from radio"
            radio.status_fn(status)
    write_record(radio, CMD_END)
    return bytes(image)


def write_record(radio, cmd, payload=None):
    # build the packet
    # 0x7D: Magic
    # cmd: Command to execute
    # 0xFF: PC -> Radio
    _packet = bytearray([0x7D, int(cmd), 0xFF])
    _length = 0
    if payload:
        _length = len(payload)
    # update the length field
    _packet.extend([_length])
    if payload:
        # add the chars to the packet
        _packet.extend(payload)
    # calculate and add the checksum to the packet
    checksum = calc_checksum(_packet[1:])
    _packet.extend([checksum])
    LOG.debug("Sent:\n%s" % util.hexprint(_packet.hex()))
    radio.pipe.write(bytes(_packet))


def read_record(radio, cmd):
    # read 4 chars for the header
    header = radio.pipe.read(4)
    if len(header) != 4:
        raise errors.RadioError('Radio did not respond')
    # Check we are actually getting a read response meant for us
    # 0x00: Radio -> PC
    if header[1:3] != bytes([int(cmd), 0x00]):
        raise errors.RadioError('Radio responded with incorrect data')
    length = header[3]
    packet = radio.pipe.read(length)
    cs = calc_checksum(header[1:]) + calc_checksum(packet)
    cs %= 256
    rcs = int.from_bytes(radio.pipe.read(1), byteorder='big')
    LOG.debug("cs =%x", cs)
    LOG.debug("rcs=%x", rcs)
    return rcs != cs, packet


def add_radio_setting(radio_setting_group, mem_field, ui_name, option_map,
                      current, doc=None):
    setting = RadioSetting(mem_field, ui_name,
                           RadioSettingValueMap(option_map, current))
    if doc is not None:
        setting.set_doc(doc)
    radio_setting_group.append(setting)


def add_radio_bool(radio_setting_group, mem_field, ui_name, current, doc=None):
    setting = RadioSetting(mem_field, ui_name,
                           RadioSettingValueBoolean(bool(current)))
    radio_setting_group.append(setting)


def format_freq(freq):
    """Format a frequency given in Hz as a string"""
    return "%.05f" % (freq / 1000000)


def freq2int(val, min, max):
    """Convert a frequency as a string to a u32. Units is Hz
    """
    _freq = chirp_common.parse_freq(str(val))
    if _freq > max or _freq < min:
        raise InvalidValueError("Frequency %s is not within range %s-%s" %
                                (chirp_common.format_freq(_freq),
                                 chirp_common.format_freq(min),
                                 chirp_common.format_freq(max)))
    return _freq


def int2freq(freq):
    """
    Convert a u32 frequency to a string for UI data entry/display
    This is stored in the radio as units of 10Hz which we compensate to Hz.
    A value of -1 indicates <no frequency>, i.e. unused channel.
    """
    if int(freq) > 0:
        f = format_freq(freq * 10)
        return f
    else:
        return ""


def set_freq(setting, obj, atrb, min, max):
    """ Callback to set frequency as MHz/10"""
    f = freq2int(setting.value, min, max)/10
    setattr(obj, atrb, f)
    return


def set_offset(setting, obj, atrb, min, max):
    """ Callback to set offset frequency"""
    f = freq2int(setting.value, min, max)
    if f > 0:
        setattr(obj, atrb, f)
    return


# Validate memory value
def validate_mem(obj, attr, min_val, max_val, default):
    _val = getattr(obj, attr)
    if _val < min_val or _val > max_val:
        LOG.info("%s.%s out of range. Read %d and set to %d" % (obj._name, attr, _val, default))
        return default
    else:
        return _val


# Support for the Abbree AR-F8 radio
# based on the Wouxun KG-UV8D radio
# Serial coms are at 19200 baud
# The data is passed in variable length records
# Record structure:
#  Offset   Usage
#    0      start of record (\x7d)
#    1      Command (\x80 Identify \x81 End/Reboot \x82 Read \x83 Write)
#    2      direction (\xff PC-> Radio, \x00 Radio -> PC)
#    3      length of payload (excluding header/checksum) (n)
#    4      payload (n bytes)
#    4+n+1  checksum - byte sum (% 256) of bytes 1 -> 4+n
#
# Memory Read Records:
# the payload is 3 bytes, first 2 are offset (big endian),
# 3rd is number of bytes to read
# Memory Write Records:
# the maximum payload size (from the Wouxun software) seems to be 66 bytes
#  (2 bytes location + 64 bytes data).

@directory.register
class AbbreeARF8Radio(chirp_common.CloneModeRadio,
                chirp_common.ExperimentalRadio):
    """Abbree AR-F8"""
    VENDOR = "Abbree"
    MODEL = "AR-F8"
    ALIAS = []  # TODO: There are many aliases of this radio e.g. RADTEL, JIANPAI,
                # JJCC (some with bluetooth may have similar memory layout)
    mem_model = "KG-UV8D-A"  # An ABBREE AR-F8 is identified as 'KG-UV8D-A' in radio mem
    FILE_IDENT = "Abbree_AR-F8"
    BAUD_RATE = 19200
    NEEDS_COMPAT_SERIAL = False

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        # General
        rf.has_bank_index = False
        rf.has_dtcs = True
        rf.has_rx_dtcs = True
        rf.has_dtcs_polarity = True
        rf.has_mode = True
        rf.has_offset = True
        rf.has_name = True
        rf.has_bank = False
        rf.has_bank_names = False
        rf.has_tuning_step = False
        rf.has_ctone = True
        rf.has_cross = True
        rf.has_infinite_number = False
        rf.has_nostep_tuning = True
        rf.has_comment = False
        rf.has_settings = True

        # Attributes
        rf.valid_modes = ["FM", "NFM"]
        rf.valid_tmodes = TONE_MODE_LIST
        rf.valid_duplexes = ["", "+", "-"]
        rf.valid_tuning_steps = STEPS
        rf.valid_bands = [VHF_RANGE, UHF_RANGE]
        rf.valid_skips = ["", "S"]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_name_length = 8
        rf.valid_cross_modes = CROSS_MODE_LIST
        rf.valid_dtcs_pols = ['NN', 'NR', 'RN', 'RR']
        rf.valid_dtcs_codes = DTCS_CODES

        rf.has_sub_devices = False
        rf.memory_bounds = (1, 999)
        rf.can_odd_split = False
        rf.can_delete = True

        return rf

    def sync_in(self):
        try:
            self._mmap = do_download(self)
        except errors.RadioError:
            raise
        except Exception as e:
            raise errors.RadioError("Failed to communicate with radio: %s" % e)
        self.process_mmap()

    def sync_out(self):
        do_upload(self)

    def process_mmap(self):
        self._memobj = bitwise.parse(_AR_F8_MEM_FORMAT, self._mmap)

    @classmethod
    def match_model(cls, filedata, filename):
        return cls.FILE_IDENT in filedata[0x400:0x40C]

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = _(dedent("""
                            The Abbree AR-F8 radio driver is currently under development.
                            There are no known issues with it, but you should 
                            proceed with caution.
                           
                            Please save an unedited copy of your first successful
                            download to a CHIRP Radio Images (*.img) file.
                            """))
        return rp

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number])

    def _get_tone(self, _mem, mem):
        def _get_dcs(val):
            code = int("%03o" % (val & 0x07FF))
            pol = (val & 0x8000) and "R" or "N"
            return code, pol

        tpol = False
        if _mem.txtone != 0xFFFF and (_mem.txtone & 0x2800) == 0x2800:
            tcode, tpol = _get_dcs(_mem.txtone)
            mem.dtcs = tcode
            txmode = "DTCS"
        elif _mem.txtone != 0xFFFF and _mem.txtone != 0x0:
            if (_mem.txtone/10) not in CTCSS_TONES:
                LOG.info("%s out of range. Read %d and "
                         "set to %d" % ("_mem.txtone", _mem.txtone, CTCSS_TONES[0]*10))
                _mem.txtone = CTCSS_TONES[0] * 10
            mem.rtone = (_mem.txtone & 0x7fff) / 10.0
            txmode = "Tone"
        else:
            txmode = ""

        rpol = False

        if _mem.rxtone != 0xFFFF and (_mem.rxtone & 0x2800) == 0x2800:
            rcode, rpol = _get_dcs(_mem.rxtone)
            mem.rx_dtcs = rcode
            rxmode = "DTCS"
        elif _mem.rxtone != 0xFFFF and _mem.rxtone != 0x0:
            if (_mem.rxtone / 10) not in CTCSS_TONES:
                LOG.info("%s out of range. Read %d and "
                         "set to %d" % ("_mem.rxtone", _mem.rxtone, CTCSS_TONES[0] * 10))
                _mem.rxtone = CTCSS_TONES[0] * 10
            mem.ctone = (_mem.rxtone & 0x7fff) / 10.0
            rxmode = "Tone"
        else:
            rxmode = ""

        if txmode == "Tone" and not rxmode:
            mem.tmode = "Tone"
        elif txmode == rxmode and txmode == "Tone" and mem.rtone == mem.ctone:
            mem.tmode = "TSQL"
        elif txmode == rxmode and txmode == "DTCS" and mem.dtcs == mem.rx_dtcs:
            mem.tmode = "DTCS"
        elif rxmode or txmode:
            mem.tmode = "Cross"
            mem.cross_mode = "%s->%s" % (txmode, rxmode)

        # always set it even if no dtcs is used
        mem.dtcs_polarity = "%s%s" % (tpol or "N", rpol or "N")

        LOG.debug("Got TX %s (%i) RX %s (%i)" %
                  (txmode, _mem.txtone, rxmode, _mem.rxtone))

    def get_memory(self, number):
        _mem = self._memobj.memory[number]
        _nam = self._memobj.names[number]

        mem = chirp_common.Memory()
        mem.number = number

        _valid = self._memobj.valid[mem.number]

        LOG.debug("%d %s", number, _valid == MEM_VALID)
        if _valid != MEM_VALID:
            mem.empty = True
            return mem
        else:
            mem.empty = False

        # Freq. is stored in 10's of Hz
        mem.freq = int(_mem.rxfreq) * 10

        if _mem.txfreq == 0xFFFFFFFF:
            # TX freq not set
            mem.duplex = "off"
            mem.offset = 0
        elif int(_mem.rxfreq) == int(_mem.txfreq):
            mem.duplex = ""
            mem.offset = 0
        elif abs(int(_mem.rxfreq) * 10 - int(_mem.txfreq) * 10) > 70000000:
            mem.duplex = "split"
            mem.offset = int(_mem.txfreq) * 10
        else:
            mem.duplex = int(_mem.rxfreq) > int(_mem.txfreq) and "-" or "+"
            mem.offset = abs(int(_mem.rxfreq) - int(_mem.txfreq)) * 10

        for char in _nam.name:
            if char != 0:
                mem.name += chr(char)
        mem.name = mem.name.rstrip()

        self._get_tone(_mem, mem)

        mem.power = POWER_LEVELS[_mem.power]
        mem.skip = "" if bool(_mem.scan_add) else "S"
        mem.mode = _mem.iswide and "FM" or "NFM"

        mem.extra = RadioSettingGroup('Extra', 'extra')
        mem.extra.append(RadioSetting('dcs_type', 'DCS Type',
                                      RadioSettingValueMap(zero_indexed_seq_map(DCSTYPE_LIST), _mem.dcs_type)))

        _mem.mute_mode = validate_mem(_mem, "mute_mode", 0, len(SPMUTE_LIST)-1, 0)
        mem.extra.append(RadioSetting('mute_mode', 'Speaker mute mode',
                                      RadioSettingValueMap(zero_indexed_seq_map(SPMUTE_LIST), _mem.mute_mode)))
        _mem.step = validate_mem(_mem, "step", 1, len(STEPS), 1)
        mem.extra.append(RadioSetting('step', 'Frequency step',
                                      RadioSettingValueMap(STEP_TUPLE_LIST, _mem.step)))
        _mem.squelch = validate_mem(_mem, "squelch", 0, len(SQL_LEVEL)-1, 0)
        mem.extra.append(RadioSetting('squelch', 'Squelch level',
                                      RadioSettingValueMap(zero_indexed_seq_map(SQL_LEVEL), _mem.squelch)))

        return mem

    def _set_tone(self, mem, _mem):
        def _set_dcs(code, pol):
            val = int("%i" % code, 8) + 0x2800
            if pol == "R":
                val += 0x8000
            return val

        rx_mode = tx_mode = None
        rxtone = txtone = 0xFFFF

        if mem.tmode == "Tone":
            tx_mode = "Tone"
            rx_mode = None
            txtone = int(mem.rtone * 10)
        elif mem.tmode == "TSQL":
            rx_mode = tx_mode = "Tone"
            rxtone = txtone = int(mem.ctone * 10)
        elif mem.tmode == "DTCS":
            tx_mode = rx_mode = "DTCS"
            txtone = _set_dcs(mem.dtcs, mem.dtcs_polarity[0])
            rxtone = _set_dcs(mem.dtcs, mem.dtcs_polarity[1])
        elif mem.tmode == "Cross":
            tx_mode, rx_mode = mem.cross_mode.split("->")
            if tx_mode == "DTCS":
                txtone = _set_dcs(mem.dtcs, mem.dtcs_polarity[0])
            elif tx_mode == "Tone":
                txtone = int(mem.rtone * 10)
            if rx_mode == "DTCS":
                rxtone = _set_dcs(mem.rx_dtcs, mem.dtcs_polarity[1])
            elif rx_mode == "Tone":
                rxtone = int(mem.ctone * 10)

        _mem.rxtone = rxtone
        _mem.txtone = txtone

        LOG.debug("Set TX %s (%i) RX %s (%i)" %
                  (tx_mode, _mem.txtone, rx_mode, _mem.rxtone))

    def set_memory(self, mem):
        number = mem.number

        _mem = self._memobj.memory[number]
        _nam = self._memobj.names[number]

        if mem.empty:
            _mem.set_raw("\x00" * int(_mem.size() / 8))
            self._memobj.valid[number] = 0
            self._memobj.names[number].set_raw("\x00" * int(_nam.size() / 8))
            return

        _mem.rxfreq = int(mem.freq / 10)
        if mem.duplex == "off":
            _mem.txfreq = 0xFFFFFFFF
        elif mem.duplex == "split":
            _mem.txfreq = mem.offset / 10
        elif mem.duplex == "off":
            for i in range(0, 4):
                _mem.txfreq[i].set_raw("\xFF")
        elif mem.duplex == "+":
            _mem.txfreq = (mem.freq / 10) + (mem.offset / 10)
        elif mem.duplex == "-":
            _mem.txfreq = (mem.freq / 10) - (mem.offset / 10)
        else:
            _mem.txfreq = mem.freq / 10
        _mem.scan_add = (mem.skip != "S")
        _mem.iswide = (mem.mode == "FM")
        # set the tone
        self._set_tone(mem, _mem)

        # set the power
        if mem.power:
            _mem.power = POWER_LEVELS.index(mem.power)
        else:
            _mem.power = True

        # Now set the 'extras'
        for setting in mem.extra:
            if type(setting.value) == 'tuple':
                name, val = setting.value
                setting.value = val
            setattr(_mem, setting.get_name(), int(setting.value))

        for i in range(0, len(_nam.name)):
            if i < len(mem.name) and mem.name[i]:
                _nam.name[i] = ord(mem.name[i])
            else:
                _nam.name[i] = 0x0
        self._memobj.valid[mem.number] = MEM_VALID

    def _get_settings(self):
        _settings = self._memobj.settings
        _vfoa = self._memobj.vfoa
        _vfob = self._memobj.vfob
        _vhf_limits = self._memobj.vhf_limits
        _uhf_limits = self._memobj.uhf_limits

        # Create the tabs
        cfg_grp = RadioSettingGroup("cfg_grp", "Configuration")
        vfo_grp = RadioSettingGroup("vfo_grp", "VFO Settings")
        key_grp = RadioSettingGroup("key_grp", "Key Settings")
        scan_grp = RadioSettingGroup("scan_grp", "Scan Group")
        call_grp = RadioSettingGroup("call_grp", "Call Settings")
        info_grp = RadioSettingGroup("info", "Model Info")

        group = RadioSettings(cfg_grp, vfo_grp, key_grp, scan_grp, call_grp, info_grp)

        #
        # Configuration Settings
        #
        # Menu Number:
        # 1
        # Step - see VFO Group
        # 2
        # SQL Level - see VFO Group
        # 3
        add_radio_bool(cfg_grp, "power_save", "Battery Save", _settings.power_save,
                       doc="Select to activate or deactivate battery saver.")
        # 4
        # Transmitting power - set per Channel
        # 5
        add_radio_setting(cfg_grp, "roger_beep", "Roger Beep",
                          zero_indexed_seq_map(ROGER_LIST), _settings.roger_beep,
                          doc="Sends an end-of-transmission tone to indicate to other "
                              "stations that the transmission has ended.")
        # 6
        add_radio_setting(cfg_grp, "timeout", "Timeout TimerOFFSET (s)",
                          TIMEOUT_TUPLE_LIST, _settings.timeout,
                          doc="Limit transmission time to a set value.")
        # 7
        add_radio_setting(cfg_grp, "vox", "Voice Operated TX",
                          zero_indexed_seq_map(VOX_LIST), _settings.vox,
                          doc="Voice activated transmit gain setting.")
        # 8
        # Wideband / Narrowband - set per Channel
        # 9
        add_radio_bool(cfg_grp, "voice", "Voice Prompt", _settings.voice,
                       doc="Menu voice prompt.")
        # 10
        add_radio_setting(cfg_grp, "toalarm", "Timeout Alarm (s)",
                          zero_indexed_seq_map(TOA_LIST), _settings.toalarm,
                          doc="Alarm duration when nearing transmission timeout.")
        # 11
        add_radio_bool(cfg_grp, "beep", "Keypad Beep", _settings.beep,
                       doc="Audible confirmation of key press.")
        # 12
        add_radio_setting(cfg_grp, "language", "Language",
                          zero_indexed_seq_map(LANGUAGE_LIST), _settings.language,
                          doc="Menu and prompt language.")
        # 13
        add_radio_bool(cfg_grp, "bcl", "Busy Channel Lock-out", _settings.bcl,
                       doc="Disable [PTT] key on a channel already in use.")
        # 14
        add_radio_setting(cfg_grp, "scan_rev", "Scanner Resume Method",
                          zero_indexed_seq_map(SCANMODE_LIST), _settings.scan_rev,
                          doc="Channel scanning resume method - resume after fixed time, "
                              "after signal is gone or stop scanning.")
        # 15
        # Receiver CTCSS - set per Channel
        # 16
        # Transmitter CTCSS - set per Channel
        # 17
        # Receiver DCS - set per Channel
        # 18
        # Transmitter DCS - set per Channel
        # 19
        # DCS Type - set per channel
        # 20
        # PF2 Defn. - see Key Group
        # 21
        # PF3 Defn. - see Key Group
        # 22
        # Working Mode
        add_radio_setting(cfg_grp, "workmode_a", "VFO/MR A Workmode",
                          zero_indexed_seq_map(WORKMODE_LIST), _settings.workmode_a,
                          doc="What info. is displayed for working channel A.")
        add_radio_setting(cfg_grp, "workmode_b", "VFO/MR B Workmode",
                          zero_indexed_seq_map(WORKMODE_LIST), _settings.workmode_b,
                          doc="What info. is displayed for working channel B.")
        # 23
        add_radio_setting(cfg_grp, "backlight", "LCD Backlight timeout (s)",
                          zero_indexed_seq_map(BACKLIGHT_LIST), _settings.backlight,
                          doc="Select the time to activate the LCD backlight.")
        # 24
        # Offset - see VFOx Settings
        # 25
        # Frequency shift - see VFOx Settings
        # 26
        add_radio_bool(cfg_grp, "stopwatch", "Stopwatch", _settings.stopwatch,
                       doc="Activate or deactivate stopwatch function. If activated, "
                           "press [#LOCK] key to start / stop timer.")
        # 27
        # Channel Name - set per channel
        # 28
        # Channel Memory - set per channel
        # 29
        # Delete Channel
        # 30
        # TODO: SCNCD (Can't change on radio)
        # add_radio_setting(cfg_grp, "scncd", "CTCSS / DCS scanning",
        #                  zero_indexed_seq_map(SCANCD_LIST), _settings.scncd,
        #                  doc="Select CTCSS or DCS scanning.")
        # 31
        _settings.ponmsg = validate_mem(_settings, "ponmsg", 0, len(PONMSG_TUPLE_LIST), 1)
        add_radio_setting(cfg_grp, "ponmsg", "Poweron message",
                          PONMSG_TUPLE_LIST, _settings.ponmsg,
                          doc="Radio boot display option.")
        # 32
        # Mute Mode - set per channel
        # TODO: Add mute_mode to channel settings
        # 33
        # ANI-SW - see Key Settings
        # 34
        # ANI Code Edit - see Key Settings
        # 35
        add_radio_setting(cfg_grp, "dtmf_st", "DTMF Sidetone",
                          zero_indexed_seq_map(DTMFST_LIST), _settings.dtmf_st,
                          doc="Transmit DTMF, ANI ID or both.")
        # 36
        # Autlock Keypad - see Keys Group
        # 37
        # TODO: Implement Priority Channel logic - see https://chirp.danplanet.com/projects/chirp/wiki/MemoryEditorColumns
        add_radio_bool(cfg_grp, "prich_sw", "Priority Channel Switch", _settings.prich_sw,
                       doc="Priority scan channel enable.")
        add_radio_setting(cfg_grp, "pri_ch", "Priority Channel",
                          CHAN_TUPLE_LIST, _settings.pri_ch,
                          doc="Priority Channel.")
        # 38
        # Scan Add - set per channel
        # 39
        add_radio_setting(cfg_grp, "alert", "Alert Tone",
                          zero_indexed_seq_map(ALERTS_LIST), _settings.alert,
                          doc="Repeater single tone pulse frequency.")
        # 40
        add_radio_setting(cfg_grp, "ptt_delay", "PTT-ID Delay (ms)",
                          PTTDELAY_TUPLE_LIST, _settings.ptt_delay,
                          doc="Delay between press of [PTT] key and transmit of ANI ID.")
        # 41
        add_radio_setting(cfg_grp, "ptt_id", "PTT-ID Mode",
                          zero_indexed_seq_map(PTTID_LIST), _settings.ptt_id,
                          doc="Caller ID transmission mode - at beginning, end or both.")
        # 42
        add_radio_setting(cfg_grp, "ring_time", "RX Ringing Duration (s)",
                          zero_indexed_seq_map(RINGTIME_LIST), _settings.ring_time,
                          doc="Time to ring when receiving signal.")
        # 43
        # Scanning channel group A - see Scan Group
        # 44
        # Scanning channel group B - see Scan Group
        # 45
        add_radio_bool(cfg_grp, "rpt_tone", "Repeater Tone", _settings.rpt_tone,
                       doc="Reception confirmation tone when receiving repeater is off.")
        # 46
        add_radio_setting(cfg_grp, "sc_qt", "SC-QT",
                          zero_indexed_seq_map(SCQT_LIST), _settings.sc_qt,
                          doc="Scanning QT options - Decoder(save RX CTCSS/DCS),"
                              "Encoder(save TX CTCSS/DCS) or both.")
        # 47
        add_radio_setting(cfg_grp, "smuteset", "SubFreq Mute",
                          zero_indexed_seq_map(SMUTESET_LIST), _settings.smuteset,
                          doc="When radio operating on main band, mute sub-band.")
        # 48
        # Call Code - see Call Group Settings
        # 49
        # LCD Background styles
        _settings.screen_style = validate_mem(_settings, "screen_style", 0, len(STYLES_LIST), 0)
        add_radio_setting(cfg_grp, "screen_style", "Display colour",
                          zero_indexed_seq_map(STYLES_LIST), _settings.screen_style,
                          doc="LCD background colour.")
        # 50
        # GPS: Not in memory < 0x8000
        # 51
        # Adjust Time
        # 52
        _settings.adj_time = validate_mem(_settings, "adj_time", 0, len(ADJTIME_LIST), 0)
        add_radio_setting(cfg_grp, "adj_time", "GPS Time Adjust (hours)",
                          zero_indexed_seq_map(ADJTIME_LIST), _settings.adj_time,
                          doc="Adjust display time from GPS UTC(GMT).")

        #
        # VFO A Settings
        #
        # struct {
        #         u32     rxfreq;
        #         u32     txoffset;
        #         u16     rxtone;
        #         u16     txtone;
        #         u8      unknown1:6,
        #                 power:1,
        #                 unknown2:1;
        #         u8      unknown3:1,
        #                 shift_dir:2
        #                 unknown4:2,
        #                 mute_mode:2,
        #                 iswide:1;
        #         u8      step;
        #         u8      squelch;
        #       } vfoa;
        add_radio_setting(vfo_grp, "work_cha", "VFO A Channel",
                          CHAN_TUPLE_LIST, _settings.work_cha,
                          doc="VFO A Channel.")

        min_vhf, max_vhf = VHF_RANGE
        min_uhf, max_uhf = UHF_RANGE

        rsvs = RadioSettingValueString(0, 10, int2freq(_vfoa.rxfreq))
        rs = RadioSetting("vfoa.rxfreq", "VFO A Rx Frequency (MHz)", rsvs)
        rs.set_apply_callback(set_freq, _vfoa, "rxfreq", min_vhf, max_uhf)
        vfo_grp.append(rs)

        rsvs = RadioSettingValueString(0, 10, int2freq(_vfoa.txoffset))
        rs = RadioSetting("vfoa.txoffset", "VFO A Tx Offset (MHz)", rsvs)
        # TODO: What is the max an offset can be?
        rs.set_apply_callback(set_offset, _vfoa, "txoffset", 0, max_uhf-min_vhf)
        vfo_grp.append(rs)

        # TODO: VFOA RX/TX Tones

        add_radio_setting(vfo_grp, "vfoa.power", "VFO A Power",
                          zero_indexed_seq_map(["Low", "High"]), _vfoa.power,
                          doc="VFO A TX Power.")
        #       shift_dir:2
        add_radio_setting(vfo_grp, "vfoa.shift_dir", "VFO A Shift Direction",
                          zero_indexed_seq_map(OFFSET_LIST), _vfoa.shift_dir,
                          doc="VFO A TX freq. shift direction.")
        #       mute_mode:2,
        add_radio_setting(vfo_grp, "vfoa.mute_mode", "VFO A Mute",
                          zero_indexed_seq_map(SPMUTE_LIST), _vfoa.mute_mode,
                          doc="VFO A Mute.")
        add_radio_setting(vfo_grp, "vfoa.iswide", "VFO A Wide/Narrow FM",
                          zero_indexed_seq_map(BANDWIDTH_LIST), _vfoa.iswide,
                          doc="VFO A Wide/Narrow FM.")

        add_radio_setting(vfo_grp, "vfoa.step", "VFO A Freq. Step (kHz)",
                          STEP_TUPLE_LIST, _vfoa.step,
                          doc="VFO A Freq. Step.")
        add_radio_setting(vfo_grp, "vfoa.squelch", "VFO A Squelch",
                          zero_indexed_seq_map([str(x) for x in range(0, 10)]), _vfoa.squelch,
                          doc="VFO A Squelch.")
        #
        # VFO B Settings
        #
        add_radio_setting(vfo_grp, "work_chb", "VFO B Channel",
                          CHAN_TUPLE_LIST, _settings.work_chb,
                          doc="VFO B Channel.")

        rsvs = RadioSettingValueString(0, 10, int2freq(_vfob.rxfreq))
        rs = RadioSetting("vfob.rxfreq", "VFO B Rx Frequency (MHz)", rsvs)
        rs.set_apply_callback(set_freq, _vfob, "rxfreq", min_vhf, max_uhf)
        vfo_grp.append(rs)

        rsvs = RadioSettingValueString(0, 10, int2freq(_vfob.txoffset))
        rs = RadioSetting("vfob.txoffset", "VFO B Tx Offset (MHz)", rsvs)
        # TODO: What is the max an offset can be?
        rs.set_apply_callback(set_offset, _vfob, "txoffset", 0, max_uhf - min_vhf)
        vfo_grp.append(rs)

        # TODO: VFOB RX/TX Tones

        add_radio_setting(vfo_grp, "vfob.power", "VFO B Power",
                          zero_indexed_seq_map(["Low", "High"]), _vfob.power,
                          doc="VFO B TX Power.")

        #       shift_dir:2
        add_radio_setting(vfo_grp, "vfob.shift_dir", "VFO B Shift Direction",
                          zero_indexed_seq_map(OFFSET_LIST), _vfob.shift_dir,
                          doc="VFO B TX freq. shift direction.")
        #       mute_mode:2,
        add_radio_setting(vfo_grp, "vfob.mute_mode", "VFO B Mute",
                          zero_indexed_seq_map(SPMUTE_LIST), _vfob.mute_mode,
                          doc="VFO B Mute.")
        add_radio_setting(vfo_grp, "vfob.iswide", "VFO B Wide/Narrow FM",
                          zero_indexed_seq_map(BANDWIDTH_LIST), _vfob.iswide,
                          doc="VFO B Wide/Narrow FM.")

        add_radio_setting(vfo_grp, "vfob.step", "VFO B Freq. Step (kHz)",
                          STEP_TUPLE_LIST, _vfob.step,
                          doc="VFO B Freq. Step.")
        add_radio_setting(vfo_grp, "vfob.squelch", "VFO B Squelch",
                          zero_indexed_seq_map([str(x) for x in range(0, 10)]), _vfob.squelch,
                          doc="VFO B Squelch.")

        #
        # Key Settings
        #
        add_radio_bool(key_grp, "autolock", "Autolock Keypad", _settings.autolock,
                       doc="Autolock Keypad if unused for 8sec.")
        add_radio_bool(key_grp, "keylock", "Lock Keypad", _settings.keylock,
                       doc="Lock Keypad.")

        _msg = str(_settings.dispstr).split("\0")[0]
        val = RadioSettingValueString(0, 15, _msg)
        val.set_mutable(True)
        rs = RadioSetting("dispstr", "Display Message", val)
        key_grp.append(rs)

        add_radio_bool(key_grp, "ani_sw", "ANI-ID Switch", _settings.ani_sw,
                       doc="Select to transmit the ANI ID.")

        # ANI ID is stored as byte values with 0x0F as terminator
        # Convert to ascii chars for display
        def get_ani(ani):
            _ani = ""
            for i in range(0, 6):
                if ani[i] < 10:
                    _ani += chr(ani[i] + 0x30)
                else:
                    break
            return str(_ani)

        # ANI ID cannot start with 0 and
        # must use digits 0-9, be between 3 and
        # 6 digits in length
        # ANI ID is stored as byte values with 0x0F as terminator
        def set_ani(setting, obj, atrb):
            """ Callback to set ANI ID."""
            _ani = bytearray([0xF0] * 6)    # Unused bytes are set to 0xF0 in radio
            ani = str(setting.value).strip()
            if ani.isnumeric():
                if ani[0] != '0':
                    length = len(ani)
                    if length > 2:
                        # Convert bytes to their ascii rep.
                        for i in range(0, length):
                            _ani[i] = int(ani[i])
                        # Add terminator
                        _ani[length] = 0x0F
                    else:
                        raise InvalidValueError("ANI ID must be longer than 2 digits")
                else:
                    raise InvalidValueError("ANI ID cannot begin with a 0")
            else:
                raise InvalidValueError("ANI ID must be numeric only")

            setattr(obj, atrb, _ani)

        _ani = get_ani(_settings.ani)
        val = RadioSettingValueString(0, 6, _ani)
        val.set_mutable(True)
        rs = RadioSetting("ani", "ANI code", val)
        rs.set_apply_callback(set_ani, _settings, "ani")
        key_grp.append(rs)

        add_radio_setting(key_grp, "pf2_func", "PF2 Key function",
                          zero_indexed_seq_map(PF2KEY_LIST), _settings.pf2_func,
                          doc="PF2 Key function.")
        add_radio_setting(key_grp, "pf3_func", "PF3 Key function",
                          zero_indexed_seq_map(PF3KEY_LIST), _settings.pf3_func,
                          doc="PF3 Key function.")
        #
        # Scan Group Settings
        #
        # settings:
        #   u8    scg_a;
        #   u8    scg_b;
        #
        #   struct {
        #       u16    lower;
        #       u16    upper;
        #   } scan_groups[10];
        # TODO: Is there a grid widget that can be used for these?
        add_radio_setting(scan_grp, "scg_a", "Scanning Channel Group A",
                          zero_indexed_seq_map(SCANGRP_LIST), _settings.scg_a,
                          doc="Scanning Channel Group A.")
        add_radio_setting(scan_grp, "scg_b", "Scanning Channel Group B",
                          zero_indexed_seq_map(SCANGRP_LIST), _settings.scg_b,
                          doc="Scanning Channel Group B.")

        for i in range(0, 10):
            # CH-From
            add_radio_setting(scan_grp, "scan_groups/%d.lower" % i, "%d. From Channel" % i,
                              CHAN_TUPLE_LIST, self._memobj.scan_groups[i].lower,
                              doc="Scan Group %d - From Channel." % i)
            # CH-To
            add_radio_setting(scan_grp, "scan_groups/%d.upper" % i, "To Channel",
                              CHAN_TUPLE_LIST, self._memobj.scan_groups[i].upper,
                              doc="Scan Group %d - From Channel." % i)
        #
        # Call group settings
        #
        #     #seekto 0x0500;
        #     struct {
        #         u8    call_code[6];
        #     } call_groups[20];
        #
        #    #seekto 0x0580;
        #    struct {
        #        char    call_name[6];
        #    } call_group_name[20];
        # TODO: Is there a grid widget that can be used for this?
        for g in range(0, 20):
            _call_code = ""
            for i in self._memobj.call_groups[g].call_code:
                if i < 10:
                    _call_code += chr(i + 0x30)
                else:
                    break
            val = RadioSettingValueString(0, 6, _call_code)
            val.set_mutable(True)

            rs = RadioSetting("call_groups/%d.call_code" % g, "%d. Call Code" % (g+1), val)
            call_grp.append(rs)

            #_call_name = "".join(map(chr, self._memobj.call_group_name[g].call_name))
            _call_name = str(self._memobj.call_group_name[g].call_name)
            val = RadioSettingValueString(0, 6, _call_name)
            val.set_mutable(True)

            rs = RadioSetting("call_group_name/%d.call_name" % g, "Call Name", val)
            call_grp.append(rs)
        # OEM info
        #
        # Display the settings read from the ID process
        # TODO: Need to find a serial number in the device

        rsvs = RadioSettingValueString(0, 9, self.mem_model)
        rsvs.set_mutable(False)
        rs = RadioSetting("model", "Model Name", rsvs)
        info_grp.append(rs)

        # #seekto 0x0044;         // This doesn't exist in radio memory
        #     struct {            // Can get from CMD_ID
        #         u32    rx_start;
        #         u32    rx_stop;
        #         u32    tx_start;
        #         u32    tx_stop;
        #     } uhf_limits;
        #
        #     #seekto 0x0054;     // This doesn't exist in radio memory
        #     struct {            // Can get from CMD_ID
        #         u32    rx_start;
        #         u32    rx_stop;
        #         u32    tx_start;
        #         u32    tx_stop;
        #     } vhf_limits;

        # Show frequency range of this radio
        rsvs = RadioSettingValueString(0, 10, int2freq(_vhf_limits.rx_start))
        rsvs.set_mutable(False)
        rs = RadioSetting("vhf_limits.rx_start", "VHF RX from (MHz)", rsvs)
        info_grp.append(rs)
        rsvs = RadioSettingValueString(0, 10, int2freq(_vhf_limits.rx_stop))
        rsvs.set_mutable(False)
        rs = RadioSetting("vhf_limits.rx_stop", "VHF RX to (MHz)", rsvs)
        info_grp.append(rs)

        rsvs = RadioSettingValueString(0, 10, int2freq(_vhf_limits.tx_start))
        rsvs.set_mutable(False)
        rs = RadioSetting("vhf_limits.tx_start", "VHF TX from (MHz)", rsvs)
        info_grp.append(rs)
        rsvs = RadioSettingValueString(0, 10, int2freq(_vhf_limits.tx_stop))
        rsvs.set_mutable(False)
        rs = RadioSetting("vhf_limits.tx_stop", "VHF TX to (MHz)", rsvs)
        info_grp.append(rs)

        rsvs = RadioSettingValueString(0, 10, int2freq(_uhf_limits.rx_start))
        rsvs.set_mutable(False)
        rs = RadioSetting("uhf_limits.rx_start", "UHF RX from (MHz)", rsvs)
        info_grp.append(rs)
        rsvs = RadioSettingValueString(0, 10, int2freq(_uhf_limits.rx_stop))
        rsvs.set_mutable(False)
        rs = RadioSetting("uhf_limits.rx_stop", "UHF RX to (MHz)", rsvs)
        info_grp.append(rs)

        rsvs = RadioSettingValueString(0, 10, int2freq(_uhf_limits.tx_start))
        rsvs.set_mutable(False)
        rs = RadioSetting("uhf_limits.tx_start", "UHF TX from (MHz)", rsvs)
        info_grp.append(rs)
        rsvs = RadioSettingValueString(0, 10, int2freq(_uhf_limits.tx_stop))
        rsvs.set_mutable(False)
        rs = RadioSetting("uhf_limits.tx_stop", "UHF TX to (MHz)", rsvs)
        info_grp.append(rs)

        return group

    def get_settings(self):
        try:
            return self._get_settings()
        except:
            import traceback
            LOG.error("Failed to parse settings: %s", traceback.format_exc())
            return None

    def set_settings(self, settings):
        _settings = self._memobj.settings
        _mem = self._memobj

        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            else:
                try:
                    name = element.get_name()
                    if "." in name:
                        bits = name.split(".")
                        obj = self._memobj
                        for bit in bits[:-1]:
                            if "/" in bit:
                                bit, index = bit.split("/", 1)
                                index = int(index)
                                obj = getattr(obj, bit)[index]
                            else:
                                obj = getattr(obj, bit)
                        setting = bits[-1]
                    else:
                        obj = _settings
                        setting = element.get_name()

                    if element.has_apply_callback():
                        LOG.debug("Using apply callback")
                        element.run_apply_callback()
                    elif element.value.get_mutable():
                        LOG.debug("Setting %s = %s" % (setting, element.value))
                        setattr(obj, setting, element.value)
                except Exception as e:
                    LOG.info("Error setting %s - %s" % (element.get_name(), e))
                    # Note: This will stop execution as nothing handles this
                    # in higher levels of code
                    # TODO: either remove or create popup dialog to alert user?
                    raise

