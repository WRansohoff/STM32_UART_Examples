/*
 * Minimal statically-allocated ringbuffer.
 * © Vivonomicon 2019
 * Distributed under the MIT license. (Free for any use)
 */
#ifndef __VVC_RINGBUF
#define __VVC_RINGBUF

// Simple ring buffer.
typedef struct {
  int len;
  volatile char* buf;
  volatile int  pos;
  volatile int  ext;
} ringbuf;

// Helper macro to write to a buffer.
#define ringbuf_write( rb, x ) \
  rb.buf[ rb.ext ] = x; \
  if ( ( rb.ext + 1 ) >= rb.len ) { rb.ext = 0; } \
  else { rb.ext = rb.ext + 1; }

// Read from a buffer. Returns '\0' if there is nothing to read.
static inline char ringbuf_read( ringbuf* buf ) {
  if ( buf->pos == buf->ext ) { return '\0'; }
  char read = buf->buf[ buf->pos ];
  buf->pos = ( buf->pos < ( buf->len - 1 ) ) ? ( buf->pos + 1 ) : 0;
  return read;
}

#endif
