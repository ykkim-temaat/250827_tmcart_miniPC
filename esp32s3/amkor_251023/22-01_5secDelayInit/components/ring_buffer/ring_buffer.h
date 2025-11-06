#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

// Ring buffer structure
typedef struct
{  
    uint8_t *buffer; // Buffer
    uint16_t head; // Head pointer
    uint16_t tail; // Tail pointer
    uint16_t size; // Size of the ring buffer
} Ring_Buffer_t;

// Create a ring buffer
void RingBuffer_Init(Ring_Buffer_t* ringBuf, uint16_t capacity);
// Reset the ring buffer
void RingBuffer_Reset(Ring_Buffer_t *ringBuf);
// Destroy the ring buffer
void RingBuffer_Destory(Ring_Buffer_t *ringBuf);
// Get the capacity of the ring buffer
uint16_t RingBuffer_Get_Capacity(Ring_Buffer_t *ringBuf);
// Get the number of bytes used in the ring buffer
uint16_t RingBuffer_Get_Used_Count(Ring_Buffer_t *ringBuf);
// Get the number of free bytes in the ring buffer
uint16_t RingBuffer_Get_Free_Count(Ring_Buffer_t *ringBuf);
// Is the ring buffer empty?
uint8_t RingBuffer_IsEmpty(Ring_Buffer_t *ringBuf);
// Is the ring buffer full?
uint8_t RingBuffer_IsFull(Ring_Buffer_t *ringBuf);
// Get the i-th element by index
uint8_t RingBuffer_GetValue_ByIndex(Ring_Buffer_t *ringBuf, uint16_t index);
// Clear the ring buffer (read all stored data)
void RingBuffer_Clean_Queue(Ring_Buffer_t *ringBuf);
// Write an element to the tail (push)
void RingBuffer_Push(Ring_Buffer_t *ringBuf, uint8_t value);
// Pop an element from the head (pull)
uint8_t RingBuffer_Pop(Ring_Buffer_t *ringBuf);

// Read a single byte
uint8_t RingBuffer_ReadByte(Ring_Buffer_t *ringBuf);
// Read a byte array
void RingBuffer_ReadByteArray(Ring_Buffer_t *ringBuf, uint8_t* dest, uint16_t size);
// Read a signed short integer (two bytes)
int16_t RingBuffer_ReadShort(Ring_Buffer_t *ringBuf);
// Read an unsigned short integer (two bytes)
uint16_t RingBuffer_ReadUShort(Ring_Buffer_t *ringBuf);
// Read a signed long integer (four bytes)
int32_t RingBuffer_ReadLong(Ring_Buffer_t *ringBuf);
// Read an unsigned long integer (four bytes)
uint32_t RingBuffer_ReadULong(Ring_Buffer_t *ringBuf);
// Read a float (four bytes)
float RingBuffer_ReadFloat(Ring_Buffer_t *ringBuf);


// Write a single byte
void RingBuffer_WriteByte(Ring_Buffer_t *ringBuf, uint8_t value);
// Write a byte array
void RingBuffer_WriteByteArray(Ring_Buffer_t *ringBuf, uint8_t* src, uint16_t size);
// Write a signed short integer (two bytes)
void RingBuffer_WriteShort(Ring_Buffer_t *ringBuf, int16_t value);
// Write an unsigned short integer (two bytes)
void RingBuffer_WriteUShort(Ring_Buffer_t *ringBuf, uint16_t value);
// Write a signed long integer (four bytes)
void RingBuffer_WriteLong(Ring_Buffer_t *ringBuf, int32_t value);
// Write an unsigned long integer (four bytes)
void RingBuffer_WriteULong(Ring_Buffer_t *ringBuf, uint32_t value);
// Write a float (four bytes)
void RingBuffer_WriteFloat(Ring_Buffer_t *ringBuf, float value);


#ifdef __cplusplus
}
#endif
