/*
    USART0.h - Functions and classes to use USART0 on AVR systems for
    serial I/O (includes buffering).
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
    Copyright (c) 2014 Igor Mikolic-Torreira.  All right reserved.
    Functions readlong() and readFloat() adapted from Arduino code that
    is Copyright (c) 2005-2006 David A. Mellis and licensed under LGPL.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*!
 * \file
 *
 * \brief This file provides functions that offer high-level interfaces to %USART0 hardware, which is
 * available on the Arduino Uno (ATmega328) and Arduino Mega (ATmega2560).
 *
 * These interfaces are buffered for both input and output and operate using interrupts associated
 * with %USART0.  This means the transmit functions return immediately after queuing data in the output buffer
 * for transmission and the transmission happens asynchronously, using
 * dedicated %USART0 hardware. Similarly, data is received asynchronously and placed into the input buffer.
 *
 * The transmit and receive buffers are both ring buffers.  If you try to queue more data than the transmit
 * buffer can hold, the write functions will block until there is room in the buffer (as a result of
 * data being transmitted).  The receive buffer, however, will overwrite if it gets full.  You must clear
 * the receive buffer by reading it regularly when receiving significant amounts of data.
 *
 * The sizes of the transmit and receive buffers can be set at compile time via macro constants.  The default
 * sizes are 32 bytes for the receive buffer and 64 bytes for the transmit buffer.  To change these,
 * define the macros \c USART0_RX_BUFFER_SIZE (for the receive buffer) and \c USART0_TX_BUFFER_SIZE
 * (for the transmit buffer) to whatever sizes you need.  You need to make these define these macros prior
 * to compiling the file USART0.cpp.
 *
 * Two interfaces are provided.  USART0 is a functional interface that makes use of the buffering and
 * asynchronous transmit and receive capabilities of the microcontrollers.  However, USART0 is limited to
 * transmitting and receiving byte and character streams.
 *
 * Serial0 is the most advanced and capable interface to the %USART0 hardware.  Serial0 provides a
 * object-oriented interface that includes the ability to read and write numbers of various types and
 * in various formats, all asynchronously.
 *
 * To use these functions, include USART0.h in your source code and link against USART0.cpp.
 *
 * \note Linking against USART0.cpp installs interrupt functions for transmit and receive on %USART0
 * (interrupts USART_UDRE and USART_RX on Arduino Uno/ATmega328; interrupts USART0_UDRE and USART0_RX on
 * Arduino Mega/ATmega2560).  You cannot use the minimal interface to %USART0 (from USARTMinimal.h)
 * if you link against USART0.cpp.  In particular, do \e not call initUSART0() or clearUSART0()
 * if you link against USART0.cpp.
 */

#ifndef USART0_h
#define USART0_h

#include "Writer.h"
#include "Reader.h"

#include <stdint.h>
#include <stddef.h>


#ifndef USART_SERIAL_CONFIG
#define USART_SERIAL_CONFIG

/*!
 * \brief This enum lists serial configuration in terms of data bits, parity, and stop bits.
 *
 * The format is kSerial_XYZ where
 * - X = the number of data bits
 * - Y = N, E, or O; where N = none, E = even, and O = odd
 * - Z = the number of stop bits
 *
 * \hideinitializer
 */
enum UsartSerialConfiguration
{
    kSerial_5N1 = 0x00,     //!< 5 data bits, no parity, 1 stop bit  \hideinitializer
    kSerial_6N1 = 0x02,     //!< 6 data bits, no parity, 1 stop bit  \hideinitializer
    kSerial_7N1 = 0x04,     //!< 7 data bits, no parity, 1 stop bit  \hideinitializer
    kSerial_8N1 = 0x06,     //!< 8 data bits, no parity, 1 stop bit  \hideinitializer
    kSerial_9N1 = 0x07,     //!< 9 data bits, no parity, 1 stop bit  \hideinitializer
    kSerial_5N2 = 0x08,     //!< 5 data bits, no parity, 2 stop bits  \hideinitializer
    kSerial_6N2 = 0x0A,     //!< 6 data bits, no parity, 2 stop bits  \hideinitializer
    kSerial_7N2 = 0x0C,     //!< 7 data bits, no parity, 2 stop bits  \hideinitializer
    kSerial_8N2 = 0x0E,     //!< 8 data bits, no parity, 2 stop bits  \hideinitializer
    kSerial_9N2 = 0x0F,     //!< 9 data bits, no parity, 2 stop bits  \hideinitializer
    kSerial_5E1 = 0x20,     //!< 5 data bits, even parity, 1 stop bit  \hideinitializer
    kSerial_6E1 = 0x22,     //!< 6 data bits, even parity, 1 stop bit  \hideinitializer
    kSerial_7E1 = 0x24,     //!< 7 data bits, even parity, 1 stop bit  \hideinitializer
    kSerial_8E1 = 0x26,     //!< 8 data bits, even parity, 1 stop bit  \hideinitializer
    kSerial_9E1 = 0x27,     //!< 9 data bits, even parity, 1 stop bit  \hideinitializer
    kSerial_5E2 = 0x28,     //!< 5 data bits, even parity, 2 stop bits  \hideinitializer
    kSerial_6E2 = 0x2A,     //!< 6 data bits, even parity, 2 stop bits  \hideinitializer
    kSerial_7E2 = 0x2C,     //!< 7 data bits, even parity, 2 stop bits  \hideinitializer
    kSerial_8E2 = 0x2E,     //!< 8 data bits, even parity, 2 stop bits  \hideinitializer
    kSerial_9E2 = 0x2F,     //!< 9 data bits, even parity, 2 stop bits  \hideinitializer
    kSerial_5O1 = 0x30,     //!< 5 data bits, odd parity, 1 stop bit  \hideinitializer
    kSerial_6O1 = 0x32,     //!< 6 data bits, odd parity, 1 stop bit  \hideinitializer
    kSerial_7O1 = 0x34,     //!< 7 data bits, odd parity, 1 stop bit  \hideinitializer
    kSerial_8O1 = 0x36,     //!< 8 data bits, odd parity, 1 stop bit  \hideinitializer
    kSerial_9O1 = 0x37,     //!< 9 data bits, odd parity, 1 stop bit  \hideinitializer
    kSerial_5O2 = 0x38,     //!< 5 data bits, odd parity, 2 stop bits  \hideinitializer
    kSerial_6O2 = 0x3A,     //!< 6 data bits, odd parity, 2 stop bits  \hideinitializer
    kSerial_7O2 = 0x3C,     //!< 7 data bits, odd parity, 2 stop bits  \hideinitializer
    kSerial_8O2 = 0x3E,     //!< 8 data bits, odd parity, 2 stop bits  \hideinitializer
    kSerial_9O2 = 0x3F      //!< 9 data bits, odd parity, 2 stop bits  \hideinitializer
};

#endif


/*!
 * \brief This namespace bundles a high-level buffered interface to the %USART0 hardware.  It provides logical cohesion and
 * prevents namespace collisions.
 */

namespace USART0
{

    /*!
    * \brief Initialize %USART0 for buffered, asynchronous serial communications using interrupts.
    *
    * You must call this function before using any of the other USART0 functions.
    *
    * \arg \c baudRate the baud rate for the communications, usually one of the following
    * values: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200
    * (although other values below can be specified).
    *
    * \arg \c config sets the configuration in term of data bits, parity, and stop bits.
    * If omitted, the default is 8 data bits, no parity, and 1 stop bit.
    */

    void start( unsigned long baudRate, UsartSerialConfiguration config = kSerial_8N1 );


    /*!
    * \brief Stops buffered serial communications using interrupts on %USART0.
    *
    * After calling this function, Arduino pins 0 and 1 are released and available for
    * use as ordinary digital pins.
    *
    * If you want to use %USART0 again for buffered, asynchronous serial communications, you must
    * again call start().
    */

    void stop();



    /*!
    * \brief Write a single byte to the transmit buffer.
    *
    * This function attempts to queue the data into the transmit buffer.  If there is
    * room in the transmit buffer, the function returns immediately.  If not, the function blocks
    * waiting for room to become available in the transmit buffer.
    *
    * The data is transmitted asynchronously via %USART0-related interrupts.
    *
    * \arg \c c the char (byte) to write into the transmit buffer
    *
    * \returns the number of bytes written into the output buffer.
    */

    size_t write( char c );


    /*!
    * \brief Write a null-terminated string to the transmit buffer.
    *
    * This function attempts to queue the data into the transmit buffer.  If there is
    * room in the transmit buffer, the function returns immediately.  If not, the function blocks
    * waiting for room to become available in the transmit buffer.
    *
    * The data is transmitted asynchronously via %USART0-related interrupts.
    *
    * \arg \c c the null-terminated string to write into the transmit buffer.
    *
    * \returns the number of bytes written into the output buffer.
    */

    size_t write( const char* c );


    /*!
    * \brief Write a character array of given size to the transmit buffer.
    *
    * This function attempts to queue the data into the transmit buffer.  If there is
    * room in the transmit buffer, the function returns immediately.  If not, the function blocks
    * waiting for room to become available in the transmit buffer.
    *
    * The data is transmitted asynchronously via %USART0-related interrupts
    *
    * \arg \c c the character array to write into the transmit buffer.
    *
    * \arg \c n the number of elements from the array to write into the transmit buffer.
    *
    * \returns the number of characters written into the output buffer.
    */

    size_t write( const char* c, size_t n );


    /*!
    * \brief Write a byte array of given size to the transmit buffer.
    *
    * This function attempts to queue the data into the transmit buffer.  If there is
    * room in the transmit buffer, the function returns immediately.  If not, the function blocks
    * waiting for room to become available in the transmit buffer.
    *
    * The data is transmitted asynchronously via %USART0-related interrupts
    *
    * \arg \c c the byte array to write into the transmit buffer.
    *
    * \arg \c n the number of elements from the array to write into the transmit buffer.
    *
    * \returns the number of bytes written into the output buffer.
    */

    size_t write( const uint8_t* c, size_t n );


    /*!
    * \brief Flush transmit buffer.
    *
    * This function blocks until the transmit buffer is empty and the last byte has been transmitted by %USART0.
    * flush() doesn't actually do anything to make the transmit happen; it simply waits for the transmission to
    * complete.
    */

    void flush();


    /*!
    * \brief Examine the next character in the receive buffer without removing it from the buffer.
    *
    * \returns if there is a value in the receive buffer, it returns the value (a number between 0 and 255); if
    * the receive buffer is empty, it returns -1;
    */

    int peek();


    /*!
    * \brief Return the next character in the receive buffer, removing it from the buffer.
    *
    * \returns if there is a value in the receive buffer, it returns the value (a number between 0 and 255)
    * and removes the value from the receive buffer; if the receive buffer is empty, it returns -1;
    */

    int read();


    size_t read( char *buffer, size_t length );

    /*!
    * \brief Determine if there is data in the receive buffer..
    *
    * \returns if the receive buffer contains data, it returns TRUE;
    * if the receive buffer is empty, it returns FALSE;
    */

    bool available();
  
};



/*!
 * \brief Provides a high-end interface to serial communications using %USART0.
 *
 * The functions in this class are buffered for both input and output and operate using interrupts associated
 * with %USART0.  This means the write functions return immediately after queuing data in the output buffer
 * for transmission and the transmission happens asynchronously, using
 * dedicated %USART0 hardware. Similarly, data is received asynchronously and placed into the read buffer.
 *
 * The read and write buffers are both ring buffers.  If you try to queue more data than the transmit
 * buffer can hold, the write functions will block until there is room in the buffer (as a result of
 * data being transmitted).  The read buffer, however, will overwrite if it gets full.  You must clear
 * the read buffer by actually reading the data regularly when receiving significant amounts of data.
 */

class Serial0 : public Writer, public Reader
{
public:

    /*!
     * \brief Configure the hardware for two-way serial communications, including turning on associated
     * interrupts.  You must call this
     * function before reading from or writing to Serial0 on %USART0.
     *
     * \arg \c baudRate the baud rate for the communications, usually one of the following
     * values: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200
     * (although other values below can be specified).
     *
     * \arg \c config sets the configuration in term of data bits, parity, and stop bits.
     * If omitted, the default is 8 data bits, no parity, and 1 stop bit.
     */
  void start( unsigned long baudRate, UsartSerialConfiguration config = kSerial_8N1 )
    { USART0::start( baudRate, config ); }


    /*!
     * \brief Stops buffered serial communications using Serial0 on %USART0 by deconfiguring
     * the hardware and turning off interrupts.
     *
     * After calling this function, Arduino pins 0 and 1 are released and available for
     * use as ordinary digital pins.
     *
     * If you want to use Serial0 again for communications, you must call start() again.
     */
    void stop()
    { USART0::stop(); }


    /*!
     * \brief Write a single character to the output stream.  This implements the pure virtual function
     * Writer::write( char c ).
     *
     * \arg \c  the character to be written.
     *
     * \returns the number of bytes written.
     */
    virtual size_t write( char c ) override;

    /*!
     * \brief Write a null-terminated string to the output stream.  This implements the pure virtual function
     * Writer::write( char* str ).
     *
     * \arg \c str the string to be written.
     *
     * \returns the number of bytes written.
     */
    virtual size_t write( const char* str ) override;

    /*!
     * \brief Write a given number of characters from a buffer to the output stream.  This implements the pure virtual function
     * Writer::write( const char* buffer, size_t size ).
     *
     * \arg \c buffer the buffer of characters to write.
     * \arg \c size the number of characters to write
     *
     * \returns the number of bytes written.
     */
    virtual size_t write( const char* buffer, size_t size ) override;

    /*!
     * \brief Write a given number of bytes from a buffer to the output stream.  This implements the pure virtual function
     * Writer::write( const uint8_t* buffer, size_t size ).
     *
     * \arg \c buffer the buffer of bytes to write.
     * \arg \c size the number of bytes to write
     *
     * \returns the number of bytes written.
     */
    virtual size_t write( const uint8_t* buffer, size_t size ) override;

    /*!
     * \brief Flush the output stream.  When this function returns, all previously
     * written data will have been transmitted through the underlying output stream.
     * This implements the pure virtual function Writer::flush().
     */
    virtual void flush() override;



    // Virtual functions from Reader

    /*!
     * \brief Read and remove the next byte from the input stream.  This implements the pure virtual function
     * Reader::read().
     *
     * \returns the next byte, or -1 if there is nothing to read in the input stream
     * before timeout expires.
     */
    virtual int read() override;

    size_t readBytes( char *buffer, size_t length );


    /*!
     * \brief Examine the next byte from the input stream, without removing it.  This implements the pure
     * virtual function Reader::peek().
     *
     * \returns the next byte, or -1 if there is nothing to read in the input stream
     * before timeout expires.
     */
    virtual int peek() override;


    /*!
     * \brief Determine if data is available in the input stream.  This implements the pure
     * virtual function Reader::available().
     *
     * \returns True if data is available in the stream; false if not.
     */
    virtual bool available() override;
};



#endif

