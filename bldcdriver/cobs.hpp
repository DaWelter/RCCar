#pragma once

#include <cstdio>
#include <stdint.h>

template<class Callback>
class COBSIncremental
{
public:
  //using Callback = void (const uint8_t *buffer, size_t count);
    
private:
  static constexpr size_t BUFFER_SIZE = 0xff+1;
  uint8_t encodedBuffer[BUFFER_SIZE];
  size_t write_index;
  //size_t code_index;
  static constexpr size_t code_index = 0;
  uint8_t code;
  Callback callback;
  
  void finish_block()
  {
    // Always invoke callback on end of block.
    // Note that this is very inefficient for a number of successive 0's
    // because each 0 will create a new block, thus invoking the callback
    // every time! This could be improved by using a larger buffer at the
    // cost of higher code complexity.
    //assert(write_index <= BUFFER_SIZE);
    encodedBuffer[code_index] = code;
    callback(encodedBuffer, write_index);
  }
  
  void start_block()
  {
    write_index = 1;
    //code_index = 0;
    code = 1;
  }
  
public:  
  COBSIncremental(Callback callback) : callback(callback) 
  {
    start_block();
  }
  
  ~COBSIncremental()
  {
    finish_block();
  }
  
  void encode_incremental(const uint8_t* buffer, size_t size)
  {
    size_t read_index  = 0;
    while (read_index < size)
    {
      uint8_t c = 0;
      if (code != 0xFF)
      {
        c = buffer[read_index++];
        if (c != 0)
        {
          //assert(write_index < BUFFER_SIZE);
          encodedBuffer[write_index++] = c;
          code++;
        }
      }
      if (c == 0) // Either we actually read a 0, or code reached 0xFF
      {
        finish_block();
        start_block();
      }  
    }
  }
};
