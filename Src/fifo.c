/**
  ******************************************************************************
  * @file    fifo.c 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    July-2015
  * @brief   FIFO management sources
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
#include "fifo.h"
#include "string.h"

#define MIN(a, b) ((a <= b) ? (a) : (b))
#define MAX(a, b) ((a < b) ? (b) : (a))
#define FIFO_ALIGN(value, alignment) ((value + alignment - 1) & ~(alignment - 1))
#define ADVANCE_QUEUE(index, size, max_size) ((index + size) % max_size) /* (((index + size) >= max_size) ? 0 : (index + size))*/
#define FIFO_ALIGNMENT fifo->alignment
#define FIFO_GET_SIZE(fifo) ((fifo->tail>=fifo->head) ? (fifo->tail - fifo->head) : (fifo->max_size - (fifo->head - fifo->tail)))
#define VAR_LEN_ITEM_SIZE_LENGTH 2

/**
* @brief  Initiliaze a circular fifo specfiyng also elements alignment
* The buffer allocated memory should max_size+maximum length of element, 
* so that no wrapping occurs and each element is made of only linear buffer segments.
* @retval None
*/

void fifo_init(circular_fifo_t *fifo, uint16_t max_size, uint8_t  *buffer, uint8_t alignment)
{
  fifo->tail = fifo->head = 0;
  fifo->max_size = max_size;
  fifo->buffer = buffer;
  fifo->alignment = alignment;
  memset(buffer, 0, max_size);
}
/**
* @brief  Return number of bytes held in the FIFO
* @retval None
*/

uint16_t fifo_size(circular_fifo_t *fifo)
{
  return FIFO_GET_SIZE(fifo);
}

// left that can be used
uint16_t fifo_left_size(circular_fifo_t *fifo)
{
  uint16_t left = fifo->max_size - FIFO_GET_SIZE(fifo);

  left = (left > (FIFO_ALIGN(VAR_LEN_ITEM_SIZE_LENGTH, FIFO_ALIGNMENT) + 1)) ? 
    (left - FIFO_ALIGN(VAR_LEN_ITEM_SIZE_LENGTH, FIFO_ALIGNMENT) - 1) : 0;

  return left;
}

uint16_t fifo_left_total_size(circular_fifo_t *fifo)
{
  return fifo->max_size - FIFO_GET_SIZE(fifo);
}

void fifo_flush(circular_fifo_t *fifo)
{
  fifo->tail = fifo->head = 0;
}
 
static uint8_t _fifo_put(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer, uint16_t index)
{
  uint16_t size_aligned = FIFO_ALIGN(size, FIFO_ALIGNMENT);
  uint16_t i = 0;
  if ((FIFO_GET_SIZE(fifo) + size_aligned) < fifo->max_size) { /* <= */
    for (i = 0; i < size; i++)
    {
      fifo->buffer[(index + i) % fifo->max_size] = buffer[i];
    }
    // memcpy(&fifo->buffer[index], buffer, size);
    fifo->tail = ADVANCE_QUEUE(fifo->tail, size_aligned, fifo->max_size);
    return 0;
  }
  return 1;
}

//static uint8_t fifo_put_no_wrap(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer)
//{
//  uint16_t index = (fifo->tail == 0) ? fifo->max_size : fifo->tail;
//  return (_fifo_put(fifo, size, buffer, index));
//}

/**
* @brief  Put the buffer in the fifo (no wrapping)
* @retval None
*/
uint8_t fifo_put(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer)
{
  return (_fifo_put(fifo, size, buffer, fifo->tail));
}

static uint8_t _fifo_get(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer, uint16_t index)
{
  uint16_t size_aligned = FIFO_ALIGN(size, FIFO_ALIGNMENT);
  uint16_t i = 0;
  if (FIFO_GET_SIZE(fifo) >= size_aligned) {
    for (i = 0; i < size; i++)
    {
      buffer[i] = fifo->buffer[(index + i) % fifo->max_size];
    }
    // memcpy(buffer, &fifo->buffer[index], size);
    // fifo->head = ADVANCE_QUEUE(fifo->head, size_aligned, fifo->max_size);

    return 0;
  }
  return 1;
}

// static uint8_t fifo_get_no_wrap(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer)
// {
//   uint16_t index = (fifo->head == 0) ? fifo->max_size : fifo->head;
//   return (_fifo_get(fifo, size, buffer, index));
// }

uint8_t fifo_get(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer)
{
  return (_fifo_get(fifo, size, buffer, fifo->head));
}

uint8_t fifo_put_var_len_item(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer)
{
  uint8_t ret_val = 0;
  uint16_t size_aligned = FIFO_ALIGN(size, FIFO_ALIGNMENT);
  uint8_t  length_size_aligned = FIFO_ALIGN(VAR_LEN_ITEM_SIZE_LENGTH, FIFO_ALIGNMENT);
  
  if ((FIFO_GET_SIZE(fifo) + size_aligned + length_size_aligned) < fifo->max_size) {
    ret_val = fifo_put(fifo, VAR_LEN_ITEM_SIZE_LENGTH, (uint8_t *)&size);
    ret_val = fifo_put(fifo, size, buffer);
    // memcpy(&fifo->buffer[fifo->tail], &size, VAR_LEN_ITEM_SIZE_LENGTH);  
    // memcpy(&fifo->buffer[fifo->tail + length_size_aligned], buffer, size);
    
    // fifo->tail = ADVANCE_QUEUE(fifo->tail, size_aligned + length_size_aligned, fifo->max_size);
    
  } else {
    ret_val = 1;
  }
  return ret_val;
}

extern uint16_t Cal_CRC16(const uint8_t *data, uint32_t size);
uint8_t fifo_put_var_len_item_crc16(circular_fifo_t *fifo, uint16_t size, uint8_t  *buffer)
{
  uint8_t ret_val = 0;
  uint16_t crc16 = 0;
  uint16_t size_aligned = FIFO_ALIGN((size + sizeof(crc16)), FIFO_ALIGNMENT);
  uint8_t  length_size_aligned = FIFO_ALIGN(VAR_LEN_ITEM_SIZE_LENGTH, FIFO_ALIGNMENT);
  
  if ((FIFO_GET_SIZE(fifo) + size_aligned + length_size_aligned) < fifo->max_size) {
    crc16 = Cal_CRC16(buffer, size);
    size += sizeof(crc16);
    ret_val = fifo_put(fifo, VAR_LEN_ITEM_SIZE_LENGTH, (uint8_t *)&size);
    ret_val = fifo_put(fifo, size - sizeof(crc16), buffer);
    ret_val = fifo_put(fifo, sizeof(crc16), (uint8_t *)&crc16);
    // memcpy(&fifo->buffer[fifo->tail], &size, VAR_LEN_ITEM_SIZE_LENGTH);  
    // memcpy(&fifo->buffer[fifo->tail + length_size_aligned], buffer, size);
    
    // fifo->tail = ADVANCE_QUEUE(fifo->tail, size_aligned + length_size_aligned, fifo->max_size);
    
  } else {
    ret_val = 1;
  }
  return ret_val;
}

uint8_t fifo_discard(circular_fifo_t *fifo, uint16_t size)
{
  uint16_t size_aligned = FIFO_ALIGN(size, FIFO_ALIGNMENT);
  uint16_t i = 0;
  if (FIFO_GET_SIZE(fifo) >= size_aligned) {
    for (i = 0; i < size_aligned; i++)
    {
      fifo->buffer[(fifo->head + i) % fifo->max_size] = 0;
    }
    fifo->head = ADVANCE_QUEUE(fifo->head, size_aligned, fifo->max_size);
    return 0;
  }
  return 1;
}

uint8_t fifo_get_ptr(circular_fifo_t *fifo, uint16_t size, uint8_t **ptr)
{
  uint16_t size_aligned = FIFO_ALIGN(size, FIFO_ALIGNMENT);
  if (FIFO_GET_SIZE(fifo) >= size_aligned) {
    *ptr = &fifo->buffer[fifo->head];
    return 0;
  }
  return 1;
}

uint8_t fifo_get_var_len_item(circular_fifo_t *fifo, uint16_t *size, uint8_t  *buffer)
{
  uint8_t ret_val = fifo_get(fifo, VAR_LEN_ITEM_SIZE_LENGTH, (uint8_t *) size);
  if (ret_val == 0) {
    ret_val = _fifo_get(fifo, *size, buffer, 
    (fifo->head + FIFO_ALIGN(VAR_LEN_ITEM_SIZE_LENGTH, FIFO_ALIGNMENT)) % fifo->max_size);
    // ret_val = fifo_get_no_wrap(fifo, *size, buffer);
  }
  return ret_val;
}

uint8_t fifo_get_ptr_var_len_item(circular_fifo_t *fifo, uint16_t *size, uint8_t  **ptr)
{
  uint8_t ret_val = fifo_get_ptr(fifo, VAR_LEN_ITEM_SIZE_LENGTH, ptr);
  if (ret_val == 0)  {
    *size = *((uint16_t *) *ptr);
  }
  return ret_val;
}

uint8_t fifo_discard_var_len_item(circular_fifo_t *fifo)
{
  uint16_t size;
  uint8_t ret_val = fifo_get(fifo, VAR_LEN_ITEM_SIZE_LENGTH, (uint8_t *) &size);
  if (ret_val == 0) {
    ret_val = fifo_discard(fifo, size + FIFO_ALIGN(VAR_LEN_ITEM_SIZE_LENGTH, FIFO_ALIGNMENT));
  }
  return ret_val;
}
