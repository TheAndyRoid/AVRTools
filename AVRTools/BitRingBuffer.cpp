/*
    BitRingBuffer.cpp - A bit ring buffer class for AVR processors.
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).

    Based upon RingBuffer.cpp
    Copyright (c) 2014 Igor Mikolic-Torreira.  All right reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "BitRingBuffer.h"

#include <util/atomic.h>


BitRingBuffer::BitRingBuffer( unsigned char *buffer, unsigned short size )
  : mBuffer( buffer ), mSize( (size > 32)?(255):(size*8) ), mLength( 0 ), mIndex( 0 )
{
}


signed short BitRingBuffer::pull()
{
    signed short element = -1;
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( mLength )
	  {
            element = getBit( mIndex );
            mIndex++;
            if ( mIndex >= mSize )
            {
                mIndex -= mSize;
            }
            --mLength;
        }
    }
    return element;
}

signed short BitRingBuffer::peek( unsigned short index )
{
    int element = -1;
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( index < mLength )
        {
	  element = getBit( ( mIndex + index ) % mSize );
        }
    }
    return element;
}


bool BitRingBuffer::push( bool element )
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( mLength < mSize )
        {
	    setBit( ( mIndex + mLength ) % mSize , element);
            ++mLength;
            return 0;
        }
    }
    // If buffer is full, ignore the push()
    return 1;
}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"

bool BitRingBuffer::isFull()
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        return ( mSize - mLength ) <= 0;
    }
}


bool BitRingBuffer::isNotFull()
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        return ( mSize - mLength ) > 0;
    }
}

#pragma GCC diagnostic pop


void BitRingBuffer::clear()
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        mLength = 0;
    }
}

void BitRingBuffer::setBit(unsigned short index, bool bit)
{
    // read byte
    unsigned short byte = mBuffer[index >> 3];

    // modify bit
    unsigned short bitShift = index & 0x7;

    if ( bit ){
        // Set bit
        byte |=  1<< bitShift ;
    }else{
        // Clear Bit
        byte &= ~( 1<< bitShift );
    }
    // write byte
    mBuffer[index] = byte;
}

bool BitRingBuffer::getBit(unsigned short index)
{
    unsigned short byte = mBuffer[index >> 3];
  
    // must be a bit 0-7
    unsigned short bitShift = index & 0x7; 

    // Get the bit
    byte = byte & ( 1<< bitShift );

    if ( byte ){ 
        return true;
    } else {
        return false;
    }
}


