/*
 * PL1167_nRF24.cpp
 *
 *  Created on: 29 May 2015
 *      Author: henryk
 */

#include "PL1167_nRF24.h"

static uint16_t calc_crc(uint8_t *data, size_t data_length);
static uint8_t reverse_bits(uint8_t data);

PL1167_nRF24::PL1167_nRF24(RF24 &radio)
  : _radio(radio)
{ }

// kh: initialize the radio, for purposes of speaking Mi.Light.  "highpower" radio users should strongly consider RF24_PA_HIGH instead.
int PL1167_nRF24::open()
{
  _radio.begin();
  _radio.setAutoAck(false);
  _radio.setPALevel(RF24_PA_MAX);
  _radio.setDataRate(RF24_1MBPS);
  _radio.disableCRC();

  _syncwordLength=5;
  _radio.setAddressWidth(_syncwordLength);

  return recalc_parameters();
}

// kh: henryk created a very precise emulation, but we don't need all that.  solve the "trailer" alignment problem by statically shifting the syncword and reduce the preamble to 12 bits.
int PL1167_nRF24::recalc_parameters()
{
  int packet_length = _maxPacketLength + 2; // (_crc ? 2 : 0);
  if ( _receive_length != packet_length ) {
    _receive_length = packet_length;
    _radio.setPayloadSize( _receive_length );
  }

  int nrf_address_pos = _syncwordLength;
  if (_syncword0 & 0x01) {
    _nrf_pipe[ --nrf_address_pos ] = reverse_bits( ( (_syncword0 << 4) & 0xf0 ) + 0x05 ); // &1 yes
  } else {
    _nrf_pipe[ --nrf_address_pos ] = reverse_bits( ( (_syncword0 << 4) & 0xf0 ) + 0x0a ); // &1 no
  }
  _nrf_pipe[ --nrf_address_pos ] = reverse_bits( (_syncword0 >> 4) & 0xff);
  _nrf_pipe[ --nrf_address_pos ] = reverse_bits( ( (_syncword0 >> 12) & 0x0f ) + ( (_syncword3 << 4) & 0xf0) );
  _nrf_pipe[ --nrf_address_pos ] = reverse_bits( (_syncword3 >> 4) & 0xff);

  if (_syncword3 & 0x8000) {
    _nrf_pipe[ --nrf_address_pos ] = reverse_bits( ( (_syncword3 >> 12) & 0x0f ) + 0xa0 ); // &8000 yes
  } else {
    _nrf_pipe[ --nrf_address_pos ] = reverse_bits( ( (_syncword3 >> 12) & 0x0f ) + 0x50 ); // &8000 no
  }

  _radio.openWritingPipe(_nrf_pipe);
  _radio.openReadingPipe(1, _nrf_pipe);

  return 0;
}

// kh: no thanks, I'll take care of this
int PL1167_nRF24::setPreambleLength(uint8_t preambleLength)
{ return 0; }

int PL1167_nRF24::setSyncword(uint16_t syncword0, uint16_t syncword3)
{
  _syncwordLength = 5;
  if ( (_syncword0 != syncword0) | (_syncword3 != syncword3) ) {
    _syncword0 = syncword0;
    _syncword3 = syncword3;
    return recalc_parameters();
  }
  return 0;
}

// kh: no thanks, I'll take care of that
int PL1167_nRF24::setTrailerLength(uint8_t trailerLength)
{ return 0; }

// note that CRCs are always added on TX, this is only to control RX validation
int PL1167_nRF24::setCRC(bool crc)
{
  _crc = crc;
  return 0;
}

// this has to stay because of order-of-operations compatibility with 3rd party code
int PL1167_nRF24::setMaxPacketLength(uint8_t maxPacketLength)
{
  if ( _maxPacketLength != maxPacketLength ) {
    _maxPacketLength = maxPacketLength;
    return recalc_parameters();
  }
  return 0;
}

int PL1167_nRF24::receive(uint8_t channel)
{
  if (_channel != channel) {
    _channel = channel;
    _radio.setChannel(2 + channel);
  }

  _radio.startListening();
  if (_radio.available()) {
#ifdef DEBUG_PRINTF
  printf("Radio is available\n");
#endif
    internal_receive();
  }

  if(_received) {
#ifdef DEBUG_PRINTF
  if (_packet_length > 0) {
    printf("Received packet (len = %d)!\n", _packet_length);
  }
#endif
    return _packet_length;
  } else {
    return 0;
  }
}

int PL1167_nRF24::readFIFO(uint8_t data[], size_t &data_length)
{
  if (data_length > _packet_length) {
    data_length = _packet_length;
  }
  memcpy(data, _packet, data_length);
  _packet_length -= data_length;
  if (_packet_length) {
    memmove(_packet, _packet + data_length, _packet_length);
  }
  return _packet_length;
}

int PL1167_nRF24::writeFIFO(const uint8_t data[], size_t data_length)
{
  if (data_length > sizeof(_packet)) {
    data_length = sizeof(_packet);
  }
  memcpy(_packet, data, data_length);
  _packet_length = data_length;
  _received = false;

  return data_length;
}

int PL1167_nRF24::transmit(uint8_t channel)
{
  // make bits
  uint8_t tmp[sizeof(_packet)];
  int outp=0;

  uint16_t crc;
  if (_crc) {
    crc = calc_crc(_packet, _packet_length);
  }

  for (int inp = 0; inp < _packet_length + (_crc ? 2 : 0) + 1; inp++) {
    if (inp < _packet_length) {
      tmp[outp++] = reverse_bits(_packet[inp]);}
    else if (_crc && inp < _packet_length + 2) {
      tmp[outp++] = reverse_bits((crc >> ( (inp - _packet_length) * 8)) & 0xff);
    }
  }

  //send bits
  _radio.stopListening();

  if (_channel != channel) {
    _channel = channel;
    _radio.setChannel(2 + channel);
  }

  _radio.write(tmp, outp);
  _radio.write(tmp, outp);	// this is probably "free"
  _radio.write(tmp, outp);	// this is probably also free
  return 0;
}

int PL1167_nRF24::internal_receive()
{
  uint8_t tmp[sizeof(_packet)];
  int outp = 0;

  _radio.read(tmp, _receive_length);

  // HACK HACK HACK: Reset radio
//  open();

#ifdef DEBUG_PRINTF
  printf("Packet received: ");
  for (int i = 0; i < _receive_length; i++) {
    printf("%02X", tmp[i]);
  }
  printf("\n");
#endif

  for (int inp = 0; inp < _receive_length; inp++) {
      tmp[outp++] = reverse_bits(tmp[inp]);
  }


#ifdef DEBUG_PRINTF
  printf("Packet transformed: ");
  for (int i = 0; i < outp; i++) {
    printf("%02X", tmp[i]);
  }
  printf("\n");
#endif

  if (_crc) {
    if (outp < 2) {
#ifdef DEBUG_PRINTF
  printf("Failed CRC: outp < 2\n");
#endif
      return 0;
    }
    uint16_t crc = calc_crc(tmp, outp - 2);
    if ( ((crc & 0xff) != tmp[outp - 2]) || (((crc >> 8) & 0xff) != tmp[outp - 1]) ) {
#ifdef DEBUG_PRINTF
  uint16_t recv_crc = ((tmp[outp - 2] & 0xFF) << 8) | (tmp[outp - 1] & 0xFF);
  printf("Failed CRC: expected %d, got %d\n", crc, recv_crc);
#endif
      return 0;
    }
    outp -= 2;
  }

  memcpy(_packet, tmp, outp);

  _packet_length = outp;
  _received = true;

#ifdef DEBUG_PRINTF
  printf("Successfully parsed packet of length %d\n", _packet_length);
#endif

  return outp;
}

#define CRC_POLY 0x8408

static uint16_t calc_crc(uint8_t *data, size_t data_length) {
  uint16_t state = 0;
  for (size_t i = 0; i < data_length; i++) {
    uint8_t byte = data[i];
    for (int j = 0; j < 8; j++) {
      if ((byte ^ state) & 0x01) {
        state = (state >> 1) ^ CRC_POLY;
      } else {
        state = state >> 1;
      }
      byte = byte >> 1;
    }
  }
  return state;
}

static uint8_t reverse_bits(uint8_t data) {
  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    result <<= 1;
    result |= data & 1;
    data >>= 1;
  }
  return result;
}
