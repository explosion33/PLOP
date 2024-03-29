#include <cinttypes>
#include <string>
#include "Radio.h"

Radio::Radio(Stream * pc) : radio(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK, PIN_CS, PIN_RST, pc) {
    this->pc = pc;
    this->max_msg_size = 100;
    this->debug = false;
    this->frequency = -1;
    this->power = 0;

    radio.setOnReceiveState(CC1200::State::RX, CC1200::State::RX);
    radio.setOnTransmitState(CC1200::State::RX);
}

Radio::~Radio(){
}

bool Radio::checkExistance() {
    pc->printf("\n\nChecking for radio...\n");
    return radio.begin();
}

bool Radio::init() {
    return this->radio.begin();
}

void Radio::setup_443() {
    const CC1200::PacketMode mode = CC1200::PacketMode::VARIABLE_LENGTH;
	const CC1200::Band band = CC1200::Band::BAND_410_480MHz;
	const float frequency = 435e6;
	const float txPower = 0;
	const float fskDeviation = 49896;
	const float symbolRate = 100000;
	const float rxFilterBW = 208300;
	const CC1200::ModFormat modFormat = CC1200::ModFormat::GFSK_2;
	const CC1200::IFCfg ifCfg = CC1200::IFCfg::POSITIVE_DIV_8;
	const bool imageCompEnabled = true;
	const bool dcOffsetCorrEnabled = true;
	const uint8_t agcRefLevel = 0x2A;

	const uint8_t dcFiltSettingCfg = 1; // default chip setting
	const uint8_t dcFiltCutoffCfg = 4; // default chip setting
    const uint8_t agcSettleWaitCfg = 2;

	const CC1200::SyncMode syncMode = CC1200::SyncMode::SYNC_32_BITS;
	const uint32_t syncWord = 0x930B51DE; // default sync word
	const uint8_t syncThreshold = 8; // TI seems to recommend this threshold for most configs above 100ksps

	const uint8_t preableLengthCfg = 5; // default chip setting
	const uint8_t preambleFormatCfg = 0; // default chip setting


    radio.configureFIFOMode();
    radio.setPacketMode(mode);
    
    radio.setModulationFormat(modFormat);
    radio.setFSKDeviation(fskDeviation);
    radio.setSymbolRate(symbolRate);
    radio.setRXFilterBandwidth(rxFilterBW);
    radio.setOutputPower(txPower);
    radio.setRadioFrequency(band, frequency);
    
    radio.setIFCfg(ifCfg, imageCompEnabled);
    radio.configureDCFilter(dcOffsetCorrEnabled, dcFiltSettingCfg, dcFiltCutoffCfg);
    radio.configureSyncWord(syncWord, syncMode, syncThreshold);
    radio.configurePreamble(preableLengthCfg, preambleFormatCfg);

    // AGC configuration (straight from SmartRF)
    radio.setAGCReferenceLevel(agcRefLevel);
    radio.setAGCSyncBehavior(CC1200::SyncBehavior::FREEZE_NONE);
    radio.setAGCSettleWait(agcSettleWaitCfg);


    if(ifCfg == CC1200::IFCfg::ZERO)
    {
        radio.setAGCGainTable(CC1200::GainTable::ZERO_IF, 11, 0);
    }
    else
    {
        // enable all AGC steps for NORMAL mode
        radio.setAGCGainTable(CC1200::GainTable::NORMAL, 17, 0);
    }
    radio.setAGCHysteresis(0b10);
    radio.setAGCSlewRate(0);


    this->frequency = 435e6;

}

float Radio::get_frequency() {
    return this->frequency;
}


float Radio::get_power() {
    return this->power;
}

void Radio::transmit(const char* message, size_t len) {
    if (debug) {
        pc->printf("\n---------------------------------\n");

        pc->printf("SENDING: \"%s\"\n", message);
    }

    bool success = radio.enqueuePacket(message, len);
    radio.startTX();

    Timer timeCnt;
    timeCnt.start();

    // if packet successfully enqueued wait for TXFIFO to empty (packet has transmitted) before continuing
    if (success) {
        while (radio.getTXFIFOLen() > 0) {
            ThisThread::sleep_for(1ms);
            if (timeCnt.read_ms() > 200)
                break;
            pc->printf("fifo-len: %d | state: %d | time: %d\n", radio.getTXFIFOLen(), radio.getState(), timeCnt.read_ms());
        }
    }
    timeCnt.stop();

    if (debug) {
        pc->printf("sent: %s\n", (success ? "true" : "false")); 

        // update TX radio
        pc->printf("TX radio: state = 0x%" PRIx8 ", TX FIFO len = %zu, FS lock = 0x%u\n",
                    static_cast<uint8_t>(radio.getState()), radio.getTXFIFOLen(), radio.readRegister(CC1200::ExtRegister::FSCAL_CTRL) & 1);
    }

}

char* Radio::recieve(size_t* len) {
    size_t message_size;
    char receiveBuffer[this->max_msg_size];

    switch (radio.getState()) {
        case CC1200::State::RX_FIFO_ERROR: {
            radio.receivePacket(receiveBuffer, this->max_msg_size);
            radio.idle();
            
            *len = 0;
            return new char[0];

            break;
        }
        case CC1200::State::RX: {
            break;
        }
        default: {
            radio.startRX();
        }
    }

    bool hp = radio.hasReceivedPacket();


    if (debug && radio.getRXFIFOLen() > 0) {
        pc->printf("\n---------------------------------\n");

        pc->printf("RX radio: state = 0x%" PRIx8 ", RX FIFO len = %zu | hasPacket: %d\n",
                    static_cast<uint8_t>(radio.getState()), radio.getRXFIFOLen(), hp);
    }


    if (hp) {
        size_t bytes = radio.receivePacket(receiveBuffer, this->max_msg_size);
        //radio.idle();

        if (bytes > max_msg_size) {
            if (debug) {
                pc->printf("recieved %d bytes, max size is %d ... discarding", bytes, this->max_msg_size);
            }

            *len = 0;
            return new char[0];
        }

        char* out = new char[bytes];

        for (int i = 0; i<bytes; i++) {
            out[i] = receiveBuffer[i];
        }
        
        
        *len = bytes;
        return out;
    }

    *len = 0;
    return new char[0];
}


void Radio::set_debug(bool debug) {
    this->debug = debug;
}


bool Radio::set_frequency(float frequency) {
    CC1200::Band band = CC1200::Band::BAND_136_160MHz;
    if (frequency >= 820e6 && frequency <= 960e6) {
        band = CC1200::Band::BAND_820_960MHz;
    }
    else if (frequency >= 410e6 && frequency <= 480e6) {
        band = CC1200::Band::BAND_273_320MHz;
    }
    else if (frequency >= 273.3e6 && frequency <= 320e6) {
        band = CC1200::Band::BAND_273_320MHz;
    }
    else if (frequency >= 205e6 && frequency <= 240e6) {
        band = CC1200::Band::BAND_205_240MHz;
    }
    else if (frequency >= 164e6 && frequency <= 192e6) {
        band = CC1200::Band::BAND_164_192MHz;
    }
    else if (frequency >= 136.7e6 && frequency <= 160e6) {
        band = CC1200::Band::BAND_136_160MHz;
    }
    else {
        return false;
    }

    radio.setRadioFrequency(band, frequency);
    this->frequency = frequency;

    return true;
}

bool Radio::set_power(float power) {
    if (power < -16 || power > 14)
        return false;

    radio.setOutputPower(power);
    return true;
}


bool Radio::set_deviation(float deviation) {
    this->deviation = deviation;
    this->radio.setFSKDeviation(deviation);
    return true;
}
bool Radio::set_symbol_rate(float symbol_rate) {
    this->symbol_rate = symbol_rate;
    this->radio.setSymbolRate(symbol_rate);
    return true;
}
bool Radio::set_rx_filter(float filter_bandwith) {
    this->filter_bandwith = filter_bandwith;
    this->radio.setRXFilterBandwidth(filter_bandwith);
    return true;
}

bool Radio::set_modulation(char modulation) {
    if (modulation < 0x0 || modulation > 0x05 || modulation == 0x02) {
        return false;
    }

    CC1200::ModFormat mode = static_cast<CC1200::ModFormat>(modulation);
    this->radio.setModulationFormat(mode);

    return true;
}
