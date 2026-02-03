#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Reset queue
void RingBuffer_Reset(Ring_Buffer_t *ringBuf)
{
    // Head and tail pointers point to the start of the buffer
    ringBuf->head = 0;
    ringBuf->tail = 0;
    // Initialize all values to 0
    for (uint16_t i = 0; i < ringBuf->size; i++)
    {
        ringBuf->buffer[i] = 0;
    }
}

// Initialize queue
void RingBuffer_Init(Ring_Buffer_t *ringBuf, uint16_t capacity)
{
    // The size needs to be one greater than the capacity
    ringBuf->size = capacity + 1;

    // Allocate memory for the buffer
    ringBuf->buffer = (uint8_t *)malloc(ringBuf->size);

    // Reset queue data
    RingBuffer_Reset(ringBuf);
    // printf("[RingBuffer_Init] ring buffer init pbuffer=%p\n", ringBuf->buffer);
}

// Destroy queue
void RingBuffer_Destory(Ring_Buffer_t *ringBuf)
{
	free(ringBuf->buffer);
	free(ringBuf);
}

// Get queue capacity
uint16_t RingBuffer_Get_Capacity(Ring_Buffer_t *ringBuf)
{
    return ringBuf->size - 1;
}

// Get used bytes in the ring buffer
uint16_t RingBuffer_Get_Used_Count(Ring_Buffer_t *ringBuf)
{
    if (ringBuf->head > ringBuf->tail)
    {
        return RingBuffer_Get_Capacity(ringBuf) - (ringBuf->head - ringBuf->tail - 1);
    }
    else
    {
        return ringBuf->tail - ringBuf->head;
    }
}

// Get free bytes in the ring buffer
uint16_t RingBuffer_Get_Free_Count(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Get_Capacity(ringBuf) - RingBuffer_Get_Used_Count(ringBuf);
}

// Check if the queue is empty
uint8_t RingBuffer_IsEmpty(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Get_Free_Count(ringBuf) == RingBuffer_Get_Capacity(ringBuf);
}

// Check if the queue is full
uint8_t RingBuffer_IsFull(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Get_Free_Count(ringBuf) == 0;
}

// Read the i-th element by index
uint8_t RingBuffer_GetValue_ByIndex(Ring_Buffer_t *ringBuf, uint16_t index)
{
    if (index >= RingBuffer_Get_Used_Count(ringBuf))
    {
        // Index out of range
        return 0;
    }
    uint16_t rbIdx = (ringBuf->head + index + 1) % ringBuf->size;
    return ringBuf->buffer[rbIdx];
}

// Clear queue data (read out all stored data)
void RingBuffer_Clean_Queue(Ring_Buffer_t *ringBuf)
{
    while (RingBuffer_Get_Used_Count(ringBuf))
    {
        RingBuffer_Pop(ringBuf);
    }
}

// Pop the front element
uint8_t RingBuffer_Pop(Ring_Buffer_t *ringBuf)
{
    uint8_t temp = 0;
    if (!RingBuffer_IsEmpty(ringBuf))
    {
        ringBuf->head = (ringBuf->head + 1) % ringBuf->size;
        temp = ringBuf->buffer[ringBuf->head];
        // Pop the front element and clear it
        ringBuf->buffer[ringBuf->head] = 0;
    }
    return temp;
}

// Write the tail element
void RingBuffer_Push(Ring_Buffer_t *ringBuf, uint8_t value)
{
    if (RingBuffer_IsFull(ringBuf))
    {
        // The queue is full, can only pop the front element first
        RingBuffer_Pop(ringBuf);
    }
    ringBuf->tail = (ringBuf->tail + 1) % ringBuf->size;
    ringBuf->buffer[ringBuf->tail] = value;

    // if (!RingBuffer_IsFull(ringBuf))
    // {
    //     ringBuf->tail = (ringBuf->tail + 1) % ringBuf->size;
    //     ringBuf->buffer[ringBuf->tail] = value;
    // }
}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

// Read a single byte
uint8_t RingBuffer_ReadByte(Ring_Buffer_t *ringBuf)
{
    return RingBuffer_Pop(ringBuf);
}

// Read byte array
void RingBuffer_ReadByteArray(Ring_Buffer_t *ringBuf, uint8_t *dest, uint16_t size)
{
    for (uint16_t idx = 0; idx < size; idx++)
    {
        dest[idx] = RingBuffer_Pop(ringBuf);
    }
}

// Write a single byte
void RingBuffer_WriteByte(Ring_Buffer_t *ringBuf, uint8_t value)
{
    RingBuffer_Push(ringBuf, value);
}

// Write a byte array
void RingBuffer_WriteByteArray(Ring_Buffer_t *ringBuf, uint8_t *src, uint16_t size)
{
    for (uint16_t idx = 0; idx < size; idx++)
    {
        RingBuffer_Push(ringBuf, src[idx]);
    }
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////


// Read a signed short integer (two bytes)
int16_t RingBuffer_ReadShort(Ring_Buffer_t *ringBuf)
{
    int16_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 2);
    return value;
}

// Read an unsigned short integer (two bytes)
uint16_t RingBuffer_ReadUShort(Ring_Buffer_t *ringBuf)
{
    uint16_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 2);
    return value;
}

// Read a signed long integer (four bytes)
int32_t RingBuffer_ReadLong(Ring_Buffer_t *ringBuf)
{
    int32_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 4);
    return value;
}

// Read an unsigned long integer (four bytes)
uint32_t RingBuffer_ReadULong(Ring_Buffer_t *ringBuf)
{
    uint32_t value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 4);
    return value;
}

// Read a float (four bytes)
float RingBuffer_ReadFloat(Ring_Buffer_t *ringBuf)
{
    float value;
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_ReadByteArray(ringBuf, p, 4);
    return value;
}


// Write a signed short integer (two bytes)
void RingBuffer_WriteShort(Ring_Buffer_t *ringBuf, int16_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 2);
}

// Write an unsigned short integer (two bytes)
void RingBuffer_WriteUShort(Ring_Buffer_t *ringBuf, uint16_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 2);
}

// Write a signed long integer (four bytes)
void RingBuffer_WriteLong(Ring_Buffer_t *ringBuf, int32_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 4);
}

// Write an unsigned long integer (four bytes)
void RingBuffer_WriteULong(Ring_Buffer_t *ringBuf, uint32_t value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 4);
}

// Write a float (four bytes)
void RingBuffer_WriteFloat(Ring_Buffer_t *ringBuf, float value)
{
    uint8_t *p = (uint8_t *)&value;
    RingBuffer_WriteByteArray(ringBuf, p, 4);
}
