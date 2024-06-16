/*
    RingBuffer.cpp - A ring buffer class for AVR processors.
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
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




#include "RingBuffer.h"

#include <util/atomic.h>



RingBuffer::RingBuffer( unsigned char *buffer, unsigned short size )
: mBuffer( buffer ), mSize( size ), mLength( 0 ), mIndex( 0 )
{
}

bool RingBuffer::pull(uint8_t *dst, uint8_t len)
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( mLength >= len)
        {
	  while ( len ){
            *dst = mBuffer[ mIndex ];
            mIndex++;
            if ( mIndex >= mSize )
            {
                mIndex -= mSize;
            }
            --mLength;
	    --len;
	    ++dst;
	  }
	  return 0;
        }
    }
    return 1;
}

int RingBuffer::pull()
{
    int element = -1;
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( mLength )
        {
            element = mBuffer[ mIndex ];
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


int RingBuffer::peek( unsigned short index )
{
    int element = -1;
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( index < mLength )
        {
            element = mBuffer[ ( mIndex + index ) % mSize ];
        }
    }
    return element;
}


bool RingBuffer::push( unsigned char element )
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        if ( mLength < mSize )
        {
            mBuffer[ ( mIndex + mLength ) % mSize ] = element;
            ++mLength;
            return 0;
        }
    }
    // If buffer is full, ignore the push()
    return 1;
}


bool RingBuffer::push( const uint8_t *src, uint8_t len )
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
      if ( (mLength + len - 1) < mSize )
        {
	  while(len){
            mBuffer[ ( mIndex + mLength ) % mSize ] = *src;
            ++mLength;
	    --len;
	    ++src;
	  }
	  return 0;
        }
    }
    // If buffer is full, ignore the push()
    return 1;
}
 

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"

bool RingBuffer::isFull(uint8_t len){
   --len;
   ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
      return ( mSize - mLength ) <= len;
    }
}

bool RingBuffer::isFull()
{
  return isFull(1);
}

bool RingBuffer::isNotFull(uint8_t len){
  --len;
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        return ( mSize - mLength ) > len;
    }
}


bool RingBuffer::isNotFull()
{
  return isNotFull(1);
}

#pragma GCC diagnostic pop


void RingBuffer::clear()
{
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        mLength = 0;
	for ( uint8_t i = 0 ; i < mSize; ++i)
	{
	    // do a full clear 
	    mBuffer[i] = 0;
	}
    }
}





